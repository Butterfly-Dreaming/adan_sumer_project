// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <vector>
#include "detector.h"
using namespace std;
using namespace cv;
namespace rm_auto_aim
{
void Detector::setParam()
{
    color_th_ = 55;
    gray_th_ = 50;
    enemy_color = RED;
    a.max_angle = 0.5;
    a.min_light_ratio = 0.7;
    a.max_light_ratio = 1.3;
    a.max_large_center_distance = 4.3;
    a.min_large_center_distance = 3.5;
    a.max_small_center_distance = 2.7;
    a.min_small_center_distance = 1.8;
}

void Detector::preprocessImage(const cv::Mat & rgb_img)
{
    using std::vector;
    cv::Mat gray_img;
    cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);
    cv::GaussianBlur(gray_img, gray_img, cv::Size(3,3), 0);
    cv::Mat binary_brightness_img, binary_color_img;
    vector<cv::Mat> bgr;
    split(rgb_img, bgr);
    cv::Mat result_img;
    if(enemy_color == 0)
    {
        subtract(bgr[2], bgr[0], result_img);
    }else
    {
        subtract(bgr[0], bgr[2], result_img);
    }

    cv::threshold(gray_img, binary_brightness_img, gray_th_, 255, cv::THRESH_BINARY);
    cv::threshold(result_img, binary_color_img, color_th_, 255, cv::THRESH_BINARY);
#ifdef SHOW_BINARY_IMAGE
    imshow("binary_brightness_img", binary_brightness_img);
    imshow("binary_color_img", binary_color_img);
#endif
    m_binary_brightness_img = binary_brightness_img;
    m_binary_color_img = binary_color_img;
}

std::vector<Light> Detector::findLights(cv::Mat &img)
{
    using namespace std;
    using namespace cv;
    vector<Light> lights;
    Point2f offset_roi_point;
    vector<vector<Point>> contours_light;
    vector<vector<Point>> contours_brightness;
    findContours(m_binary_color_img, contours_light, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    findContours(m_binary_brightness_img, contours_brightness, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    //#pragma omp for
    for(size_t i = 0; i < contours_brightness.size(); i++)
    {
        double area = contourArea(contours_brightness[i]);
        if (area < 20.0 || 1e5 < area) continue;
        for(size_t ii = 0; ii < contours_light.size(); ii++)
        {
            if(pointPolygonTest(contours_light[ii], contours_brightness[i][0], false) >= 0.0 )
            {
                double length = arcLength(contours_brightness[i], true); // 灯条周长
                if (length > 15 && length <4000)
                {   // 使用拟合椭圆的方法要比拟合最小矩形提取出来的角度更精确
                    RotatedRect RRect = fitEllipse(contours_brightness[i]);
#ifdef SHOW_LIGHT_CONTOURS
                    // 旋转矩形提取四个点
                    Point2f rect_point[4];
                    RRect.points(rect_point);
                    for (int i = 0; i < 4 ; i++)
                    {
                        line(img, rect_point[i]+m_offset_roi_point, rect_point[(i+1)%4]+m_offset_roi_point, Scalar(255,0,255),1);
                    }

#endif
                    // 角度换算，将拟合椭圆0~360 -> -180~180
                    if(RRect.angle>90.0f)
                        RRect.angle =  RRect.angle - 180.0f;
#ifdef SHOW_LIGHT_PUT_TEXT
                    putText(img, to_string(RRect.angle), RRect.center + Point2f(2,2) + offset_roi_point, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
#endif
                    if (fabs(RRect.angle) <= 30)  // 超过一定角度的灯条不要
                    {
                        Light r(RRect);
                        auto rect = r.boundingRect();
                        if(0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= img.cols && 0 <= rect.y &&
                           0 <= rect.height && rect.y + rect.height <= img.rows)
                        {
                            int sum_r = 0, sum_b = 0;
                            auto roi = img(rect);
                            for(int i = 0; i < roi.rows; i++)
                            {
                                for(int j = 0; j < roi.cols; j++)
                                {
                                    if(cv::pointPolygonTest(contours_light[ii], cv::Point2f(j + rect.x, i + rect.y), false) >= 0)
                                    {
                                        sum_r += roi.at<cv::Vec3b>(i, j)[0];
                                        sum_b += roi.at<cv::Vec3b>(i, j)[2];
                                    }
                                }
                            }
                            r.color = sum_r < sum_b ? RED : BLUE;
//                            cout << "灯条颜色：" << r.color << endl;
                        }
                        lights.push_back(r);
                    }
                }
                break;
            }
        }
    }
    return lights;
}


std::vector<Armor> Detector::matchLights(const std::vector<Light> & lights)
{
    std::vector<Armor> armors;

  // Loop all the pairing of lights
    for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
        for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
            if (light_1->color != enemy_color || light_2->color != enemy_color) continue;

            if (containLight(*light_1, *light_2, lights)) {
                continue;
            }
            auto armor = Armor(*light_1, *light_2);
            if (isArmor(armor)) {
                armors.emplace_back(armor);
            }
        }
    }

    return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
// 两灯条之间是否夹着第三根
bool Detector::containLight(
    const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
{
    auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
    auto bounding_rect = cv::boundingRect(points);

    for (const auto & test_light : lights) {
        if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

        if (
        bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
        bounding_rect.contains(test_light.center)) {
        return true;
        }
    }

    return false;
}

bool Detector::isArmor(Armor & armor)
{
    Light light_1 = armor.left_light;
    Light light_2 = armor.right_light;
    // Ratio of the length of 2 lights (short side / long side)
    float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                               : light_2.length / light_1.length;
    bool light_ratio_ok = light_length_ratio > a.min_light_ratio && light_length_ratio < a.max_light_ratio;

    // Distance between the center of 2 lights (unit : light length)
    float avg_light_length = (light_1.length + light_2.length) / 2;
    float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
    bool center_distance_ok = (a.min_small_center_distance < center_distance &&
                               center_distance < a.max_small_center_distance) ||
                              (a.min_large_center_distance < center_distance &&
                               center_distance < a.max_large_center_distance);


    bool armor_width_ok;
    float armor_width = std::fabs(light_1.center.x - light_2.center.x);

    armor_width_ok = armor_width > light_1.size.width && armor_width > light_2.size.width
                     && armor_width > (light_1.size.width + light_2.size.width) * 3;

    // Angle of light center connection
    cv::Point2f diff = light_1.center - light_2.center;
    float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
    bool angle_ok = angle < a.max_angle;

    bool is_armor = light_ratio_ok && armor_width_ok && angle_ok && center_distance_ok;
    armor.armor_type = center_distance > a.min_large_center_distance ? LARGE : SMALL;

#ifdef SHOW_ARMOR_PARAM
    cout << "灯条长度比例：" << light_length_ratio << endl;
    cout << "中心距离：" << center_distance << endl;
    cout << "装甲板宽度：" << armor_width << endl;
    cout << "灯条中点误差：" << angle << endl;
    cout << "装甲板形状：" << armor.armor_type << endl;
#endif

    return is_armor;
}

cv::Rect Detector::GetRoi(const cv::Mat &img)
{
    cv::Size img_size = img.size();
    cv::Rect rect_tmp = last_target_;
    cv::Rect rect_roi;
    if(rect_tmp.x == 0 || rect_tmp.y == 0
            || rect_tmp.width == 0 || rect_tmp.height == 0
            || lost_cnt_ >= 15 || detect_cnt_%100 == 0
            )
    {
        last_target_ = cv::Rect(0,0,img_size.width, img_size.height);
        rect_roi = cv::Rect(0,0,img_size.width, img_size.height);
        return rect_roi;
    }
    else
    {
        float scale = 2;
        if (lost_cnt_ < 30)
            scale = 3;
        else if(lost_cnt_ <= 60)
            scale = 4;
        else if(lost_cnt_ <= 120)
            scale = 5;

        int w = int(rect_tmp.width * scale);
        int h = int(rect_tmp.height * scale);
        int x = int(rect_tmp.x - (w - rect_tmp.width)*0.5f);
        int y = int(rect_tmp.y - (h - rect_tmp.height)*0.5f);

        rect_roi = cv::Rect(x, y, w, h);

        if(makeRectSafe(rect_roi, img_size)== false)
        {
            rect_roi = cv::Rect(0,0,img_size.width, img_size.height);
        }
    }
    return rect_roi;
}
int Detector::ArmorDetectorTask(cv::Mat &img)
{
    cv::Rect Roi = GetRoi(img);
    if(DetectArmor(img, Roi))
    {
        cv::Point3f position_in_camera, position_in_gun;
        GimbalPose cur_pose;//当前云台位姿

        solve_angle_.solvePnP(target, position_in_camera, m_offset_roi_point);
        cout << "position_in_camera: " << position_in_camera << endl;
        position_in_gun = solve_angle_.changeCoor(position_in_camera);
        cout << "position_in_gun: " << position_in_gun << endl;

        // 云台系坐标转地面系
        cv::Point3f armor_abs_point = solve_angle_.cam2abs(position_in_gun, cur_pose);
        cout << "armor_abs_point: " << armor_abs_point << endl;

        //预测坐标
        cv::Point3f predict_point;
        armor_abs_point.x *= 1000;
        armor_abs_point.y *= 1000;
        armor_abs_point.z *= 1000;
        predict_point = predictor.prediction(armor_abs_point, now(), 15, INFANTRY);
        cout << "predict_point: " << predict_point << endl;

    }
}
bool Detector::DetectArmor(cv::Mat &img, cv::Rect roi_rect)
{
    setParam();
    cv::Mat roi_image = img(roi_rect);
    cv::Point2f offset_roi_point(roi_rect.x, roi_rect.y);
    m_offset_roi_point = offset_roi_point;
    preprocessImage(roi_image);
    vector<Light> lights = findLights(roi_image);
    vector<Armor> armors = matchLights(lights);
//    cv::imshow("roi", roi_image);

    float dist = 1e8;
    bool found_flag = 0;
    cv::Point2f roi_center(roi_rect.width / 2, roi_rect.height / 2);
    float dx, dy;
    for(int i = 0; i < armors.size(); i++)
    {
        dx = pow((armors.at(i).center.x - roi_center.x), 2.0f);
        dy = pow((armors.at(i).center.y - roi_center.y), 2.0f);
        if( dx + dy < dist){
            target = armors.at(i);
            dist = dx + dy;
        }
        found_flag = true;
    }
    //计算装甲板四个点顶点，用于pnp姿态结算
//    cv::RotatedRect target_rect;
    if(found_flag)
    {
        cv::circle(img, target.center + offset_roi_point, 7, cv::Scalar(255, 0, 255), -1);
        cv::Point2f point_tmp[4];
        cv::Point2f point_2d[4];
        cv::RotatedRect R, L;
        R = target.right_light;
        L = target.left_light;
        //装载图像二维坐标
        L.points(point_tmp);
        point_2d[0] = point_tmp[1];
        point_2d[3] = point_tmp[0];
        R.points(point_tmp);
        point_2d[1] = point_tmp[2];
        point_2d[2] = point_tmp[3];
        points_2d_.clear();
        vector<cv::Point2f> points_roi_tmp;
        for(int i=0;i<4;i++)
        {
            points_roi_tmp.push_back(point_2d[i] + offset_roi_point);
            points_2d_.push_back(point_2d[i] + offset_roi_point);
            putText(img, to_string(i), points_2d_.at(i), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1);
            circle(img, points_2d_.at(i), 5, Scalar(255, 255, 255), -1);
            circle(img, points_2d_.at(i), 3, Scalar(i*50, i*50, 255), -1);
        }
        //计算ROI的相关参数
        last_target_ = boundingRect(points_roi_tmp);
        rectangle(img, last_target_,Scalar(255,255,255), 1);
        lost_cnt_ = 0;
    }else{
        //计算ROI相关参数
        lost_cnt_ ++;
    }
    detect_cnt_++;


    return found_flag;
}

}  // namespace rm_auto_aim

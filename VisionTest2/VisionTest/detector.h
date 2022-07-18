#ifndef DETECTOR_H
#define DETECTOR_H
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "armor.h"
#include "solveangle.h"
#include "predict.h"
namespace rm_auto_aim
{
class Detector
{
public:
    struct LightParams
    {
        // width / height
        double min_ratio;
        double max_ratio;
        // vertical angle
        double max_angle;
    };
    struct ArmorParams
    {
        double min_light_ratio;
        double max_light_ratio;
        double min_small_center_distance;
        double max_small_center_distance;
        double min_large_center_distance;
        double max_large_center_distance;
        // horizontal angle
        double max_angle;
    };

    //色彩识别阈值
    int color_th_;
    int gray_th_;

    int enemy_color; //敌方颜色
    LightParams l;
    ArmorParams a;

    Armor target;

    void setParam();

    void preprocessImage(const cv::Mat & rbg_img);

    std::vector<Light> findLights(cv::Mat &img);

    std::vector<Armor> matchLights(const std::vector<Light> & lights);

    cv::Rect GetRoi(const cv::Mat &img);

    int ArmorDetectorTask(cv::Mat &img);

    bool DetectArmor(cv::Mat &img, cv::Rect roi_rect);

    void draw_rect(cv::Mat& img, cv::Point2f roi_offset_point, Armor armor) const;

private:
    bool isLight(const Light & light);

    bool containLight(
        const Light & light_1, const Light & light_2, const std::vector<Light> & lights);

    bool isArmor(Armor & armor);

    // ｒｏｉ参数
    cv::Rect last_target_;
    int lost_cnt_ = 0;
    int detect_cnt_ = 0;
    bool makeRectSafe(cv::Rect & rect, cv::Size size){
        if (rect.x < 0)
            rect.x = 0;
        if (rect.x + rect.width > size.width)
            rect.width = size.width - rect.x;
        if (rect.y < 0)
            rect.y = 0;
        if (rect.y + rect.height > size.height)
            rect.height = size.height - rect.y;
        if (rect.width <= 0 || rect.height <= 0)
            return false;
        return true;
    }
    //引用的类
    PnPSolver solve_angle_;
    Prediction predictor;

    //图像
    cv::Mat m_binary_brightness_img, m_binary_color_img;
    cv::Mat m_roi_img;

    //点
    cv::Point2f m_offset_roi_point;
    std::vector<cv::Point2f> points_2d_;//二维点
};

}  // namespace rm_auto_aim

#endif // DETECTOR_H

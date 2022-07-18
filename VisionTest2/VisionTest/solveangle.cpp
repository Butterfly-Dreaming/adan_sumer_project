#include "solveangle.h"

#include <opencv2/calib3d.hpp>
#include <vector>
namespace rm_auto_aim
{
PnPSolver::PnPSolver()
{
    double small_half_x = kSmallArmorWidth / 2.0;
    double small_half_y = kSmallArmorHeight / 2.0;
    double large_half_x = kLargeArmorWidth / 2.0;
    double large_half_y = kLargeArmorHeight / 2.0;

    // Start from bottom left in clockwise order 顺时针
    small_armor_points_.emplace_back(cv::Point3f(-small_half_x, small_half_y, 0));
    small_armor_points_.emplace_back(cv::Point3f(-small_half_x, -small_half_y, 0));
    small_armor_points_.emplace_back(cv::Point3f(small_half_x, -small_half_y, 0));
    small_armor_points_.emplace_back(cv::Point3f(small_half_x, small_half_y, 0));

    large_armor_points_.emplace_back(cv::Point3f(-large_half_x, large_half_y, 0));
    large_armor_points_.emplace_back(cv::Point3f(-large_half_x, -large_half_y, 0));
    large_armor_points_.emplace_back(cv::Point3f(large_half_x, -large_half_y, 0));
    large_armor_points_.emplace_back(cv::Point3f(large_half_x, large_half_y, 0));
    //相机参数
    camera_matrix_ = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
    camera_matrix_.ptr<double>(0)[0] = 1293.5303221625442802;
    camera_matrix_.ptr<double>(0)[1] = 0.3651215140945823;
    camera_matrix_.ptr<double>(0)[2] = 355.9091806402759630;
    camera_matrix_.ptr<double>(1)[1] = 1293.9256252855957428;
    camera_matrix_.ptr<double>(1)[2] = 259.1868664367483461;
    camera_matrix_.ptr<double>(2)[2] = 1.0000000000000000;
    dist_coeffs_ = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
    dist_coeffs_.ptr<double>(0)[0] = -0.2126367859619807;
    dist_coeffs_.ptr<double>(1)[0] = 0.2282910064864265;
    dist_coeffs_.ptr<double>(2)[0] = 0.0020583387355406;
    dist_coeffs_.ptr<double>(3)[0] = 0.0006136511397638;
    dist_coeffs_.ptr<double>(4)[0] = -0.7559987171745171;

//    std::cout << "camera_matrix_: " << camera_matrix_ << std::endl;
//    std::cout << "dist_coeffs_:   " << dist_coeffs_ << std:: endl;
}

bool PnPSolver::solvePnP(const Armor & armor, cv::Point3f & point, cv::Point2f offset_roi_point)
{
    std::vector<cv::Point2f> image_armor_points;

    // Fill in image points
    image_armor_points.emplace_back(armor.left_light.bottom + offset_roi_point);
    image_armor_points.emplace_back(armor.left_light.top + offset_roi_point);
    image_armor_points.emplace_back(armor.right_light.top + offset_roi_point);
    image_armor_points.emplace_back(armor.right_light.bottom + offset_roi_point);

    // Solve pnp
    cv::Mat rvec, tvec;
    auto object_points = armor.armor_type == SMALL ? small_armor_points_ : large_armor_points_;
    bool success = cv::solvePnP(
      object_points, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
      cv::SOLVEPNP_IPPE);

    if (success) {
        // Convert to geometry_msgs::msg::Point
        point.x = tvec.at<double>(0) * 0.001;
        point.y = tvec.at<double>(1) * 0.001;
        point.z = tvec.at<double>(2) * 0.001;
        return true;
    } else {
        return false;
    }
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
    float cx = camera_matrix_.at<double>(0, 2);
    float cy = camera_matrix_.at<double>(1, 2);
    return cv::norm(image_point - cv::Point2f(cx, cy));
}
cv::Point3f PnPSolver::changeCoor(cv::Point3f point)
{
    cv::Mat trans_mat = GetTransMat(m_x_off, m_y_off, m_z_off);
    cv::Mat target    = (cv::Mat_<float>(4, 1) << point.x, point.y, point.z, 1);
    cv::Mat_<float> target_in_base = trans_mat * target;
    return cv::Point3f(target_in_base.at<float>(0), target_in_base.at<float>(1), target_in_base.at<float>(2));
}

cv::Point3f PnPSolver::cam2abs(cv::Point3f position, GimbalPose cur_pose)
{
    cv::Mat rot_mat    = GetRotMatXYZ(0.0, cur_pose.pitch, cur_pose.yaw);
    cv::Mat trans_mat  = GetTransMat(m_x_off, m_y_off, m_z_off);
    cv::Mat tf_mat     = rot_mat * trans_mat;
    cv::Mat target     = (cv::Mat_<float>(4, 1) << position.x, position.y, position.z, 1);
    cv::Mat abs_target = tf_mat * target;

    return cv::Point3f(abs_target.at<float>(0), abs_target.at<float>(1), abs_target.at<float>(2)); // X Y Z
}
cv::Point3f PnPSolver::abs2cam(cv::Point3f position, GimbalPose cur_pose)
{
    cv::Mat rot_mat    = GetRotMatXYZ(0.0, cur_pose.pitch, cur_pose.yaw);
    cv::Mat trans_mat  = GetTransMat(m_x_off, m_y_off, m_z_off);
    cv::Mat tf_mat     = rot_mat * trans_mat;
    cv::Mat target     = (cv::Mat_<float>(4, 1) << position.x, position.y, position.z, 1);
    cv::Mat gun_target = tf_mat.inv() * target;

    return cv::Point3f(gun_target.at<float>(0), gun_target.at<float>(1), gun_target.at<float>(2)); // X Y Z
}
cv::Mat GetRotMatX(float x)
{
    return cv::Mat_<float>(4, 4) <<
           1,  0,      0,          0,
           0,  cos(x), -sin(x),    0,
           0,  sin(x), cos(x),     0,
           0,  0,      0,          1;
}

cv::Mat GetRotMatY(float y)
{
    return cv::Mat_<float>(4, 4) <<
           cos(y),     0,  sin(y), 0,
           0,          1,  0,      0,
           -sin(y),    0,  cos(y), 0,
           0,          0,  0,      1;
}

cv::Mat GetRotMatZ(float z)
{
    return cv::Mat_<float>(4, 4) <<
           cos(z), -sin(z),    0,  0,
           sin(z), cos(z),     0,  0,
           0,      0,          1,  0,
           0,      0,          0,  1;
}

cv::Mat GetTransMat(float x, float y, float z)
{
    return cv::Mat_<float>(4, 4) <<
           1,  0,  0,  x,
           0,  1,  0,  y,
           0,  0,  1,  z,
           0,  0,  0,  1;
}

cv::Mat GetRotMatXYZ(float x, float y, float z)
{
    return GetRotMatZ(z) * GetRotMatY(y) * GetRotMatX(x);
}

}  // namespace rm_auto_aim

#ifndef SOLVEANGLE_H
#define SOLVEANGLE_H
#include "common.h"

#include <array>
#include <vector>

#include "armor.h"

namespace rm_auto_aim
{

//构造矩阵
cv::Mat GetRotMatX(float x);
cv::Mat GetRotMatY(float y);
cv::Mat GetRotMatZ(float z);
cv::Mat GetRotMatXYZ(float x, float y, float z);
cv::Mat GetTransMat(float x, float y, float z);
class PnPSolver
{
public:
    PnPSolver();

    // Get 3d position
    bool solvePnP(const Armor & armor, cv::Point3f & point, cv::Point2f offset_roi_point);

    // Calculate the distance between armor center and image center
    float calculateDistanceToCenter(const cv::Point2f & image_point);

    //坐标系转换：相机系 -> 枪管系
    cv::Point3f changeCoor(cv::Point3f point);

    //将装甲板坐标从相对坐标转化为绝对坐标
    cv::Point3f cam2abs(cv::Point3f position, GimbalPose cur_pose);
    //将装甲板坐标从绝对坐标转化为相对坐标
    cv::Point3f abs2cam(cv::Point3f position, GimbalPose cur_pose);

private:
    float m_x_off = 0.005;
    float m_y_off = -0.05;
    float m_z_off = 0.0405;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    const float kSmallArmorWidth = 135;
    const float kSmallArmorHeight = 55;
    const float kLargeArmorWidth = 230;
    const float kLargeArmorHeight = 55;

    // Four vertices of armor in 3d
    std::vector<cv::Point3f> small_armor_points_;
    std::vector<cv::Point3f> large_armor_points_;
};

}  // namespace rm_auto_aim


#endif // SOLVEANGLE_H

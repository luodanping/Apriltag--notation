// programmed by deep 20181004
// 提供坐标转换的相关函数


#ifndef _DEEP_MATH_H
#define _DEEP_MATH_H

#include <string>
#include <sstream>
#include <vector>
#include <map>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

Eigen::Matrix4d getRelativeTransform(
    std::vector<cv::Point3d > objectPoints,
    std::vector<cv::Point2d > imagePoints, double c[3]);  //第三个参数是相机的内参

#endif

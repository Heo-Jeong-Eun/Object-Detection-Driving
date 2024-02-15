/**
 * @file GetTrafficLightColor.hpp
 * @author Jeongmin Kim (jeongmin981@gmail.com)
 * @author JeongEun Heo 
 * @brief GetTrafficLightColor Class header file
 * @date 2023-07-04
 */

#ifndef TRAFFIC_LIGHT_COLOR_DETECTOR_H
#define TRAFFIC_LIGHT_COLOR_DETECTOR_H

#include <cmath>
#include "opencv2/opencv.hpp"

namespace Xycar{
template <typename PREC>

class GetTrafficLightColor
{
public:

    using Ptr = std::unique_ptr<GetTrafficLightColor>;

    GetTrafficLightColor() = default;
    ~GetTrafficLightColor() = default;

    void processImage(const cv::Mat& image);
    int16_t detectTrafficLightColor(const cv::Mat& croppedImage);
    int16_t detectTrafficLightColor2(const cv::Mat& croppedImage);

    cv::Mat Frame;
};

}

#endif  // TRAFFIC_LIGHT_COLOR_DETECTOR_H

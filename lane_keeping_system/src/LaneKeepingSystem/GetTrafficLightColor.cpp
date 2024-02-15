/**
 * @file GetTrafficLightColor.cpp
 * @author JeongMin Kim
 * @author JeongEun Heo
 * @brief traffic light color discrimination file 
 * @version 1.1
 * @date 2023-07-04
 */

#include "LaneKeepingSystem/GetTrafficLightColor.hpp"

namespace Xycar {


template <typename PREC>
int16_t GetTrafficLightColor<PREC>::detectTrafficLightColor(const cv::Mat& cropped_image)
{
    // result values: off = 0, red = 1, yellow = 2, green = 3
    int16_t result = 0;
    // blur processing for accurate circle detection (using median blur)
    cv::Mat resized_image;
    cv::Mat blur_image;
    
    cv::medianBlur(cropped_image, blur_image, 5);

    // BGR -> HSV
    cv::Mat hsv;
    cv::cvtColor(blur_image, hsv, cv::COLOR_BGR2HSV);

    // split H, S, V channels (to use V)
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    // detect circles
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(channels[2], circles, cv::HOUGH_GRADIENT, 1, 20, 25, 25, 30, 40);

    // define lower and upper boundaries for red, green, and yellow colors
    cv::Scalar red_lower(0, 100, 100);
    cv::Scalar red_upper(10, 255, 255);
    cv::Scalar green_lower(45, 90, 90);
    cv::Scalar green_upper(100, 255, 255);
    cv::Scalar yellow_lower(20, 100, 120);
    cv::Scalar yellow_upper(35, 255, 255);

    for (const auto& circle : circles)
    {
        // circle coordinates and radius
        int32_t x = std::min(std::max(10, cvRound(circle[0])), cropped_image.cols - 10);
        int32_t y = std::min(std::max(10, cvRound(circle[1])), cropped_image.rows - 10);
        int32_t radius = cvRound(circle[2]);
        
        // crop region around the circle
        cv::Mat cr_image_h = channels[0](cv::Range(y - 10, y + 10), cv::Range(x - 10, x + 10));
        cv::Mat cr_image_s = channels[1](cv::Range(y - 10, y + 10), cv::Range(x - 10, x + 10));
        cv::Mat cr_image_v = channels[2](cv::Range(y - 10, y + 10), cv::Range(x - 10, x + 10));
        
        cv::Mat cr_image;
        cv::merge(std::vector<cv::Mat>{cr_image_h, cr_image_s, cr_image_v}, cr_image);

        // draw the circle
        cv::circle(cropped_image, cv::Point(x, y), radius, cv::Scalar(0, 255, 0), 2);

        // resize images to the same size
        cv::resize(cropped_image, resized_image, cr_image.size());

        // use mask images to detect colors within the HSV image
        // red version
        cv::Mat red_mask, red_result;
        cv::inRange(cr_image, red_lower, red_upper, red_mask);
        cv::bitwise_and(resized_image, resized_image, red_result, red_mask);

        // green version
        cv::Mat green_mask, green_result;
        cv::inRange(cr_image, green_lower, green_upper, green_mask);
        cv::bitwise_and(resized_image, resized_image, green_result, green_mask);

        // yellow version
        cv::Mat yellow_mask, yellow_result;
        cv::inRange(cr_image, yellow_lower, yellow_upper, yellow_mask);
        cv::bitwise_and(resized_image, resized_image, yellow_result, yellow_mask);

        // if the pixel mean value is nonzero, a color has been detected
        if (cv::mean(cr_image).val[0] >= 90)
        {
            if (cv::mean(red_result).val[0] >= 2) result = 1;
            if (cv::mean(yellow_result).val[0] >= 2) result = 2;
            if (cv::mean(green_result).val[0] >= 2) result = 3;
        }

        // if the cropped image mean value is less than or equal to 60, traffic light is off (result = 0)
        if (cv::mean(cropped_image).val[0] <= 60)
        {
            result = 0;
        }
        std::cout << cv::mean(cropped_image) << std::endl;
        std::cout << result << std::endl;

        cv::imshow("image", cropped_image);
        cv::waitKey(1000);
    }
    return result;
}

template <typename PREC>
int16_t GetTrafficLightColor<PREC>::detectTrafficLightColor2(const cv::Mat& cropped_image)
{
    // result values: off = 0, red = 1, yellow = 2, green = 3
    int16_t result = 0;
    std::vector<cv::Mat> channels;
    cv::split(cropped_image, channels);

    // 각각의 색상 채널의 평균을 계산합니다.
    cv::Scalar blueMean = cv::sum(channels[0]);
    cv::Scalar greenMean = cv::sum(channels[1]);
    cv::Scalar redMean = cv::sum(channels[2]);

    // 각각의 색상 채널의 평균값을 출력합니다.
    // std::cout << "Blue Mean: " << blueMean[0] << std::endl;
    // std::cout << "Green Mean: " << greenMean[0] << std::endl;
    // std::cout << "Red Mean: " << redMean[0] << std::endl;

    PREC redGreenRatio = redMean[0] / greenMean[0];

    if (redGreenRatio <= 0.945) {
        result = 3;
    } else {
        result = 1;
    }
    std::cout << "detection: " << result << std::endl;
    std::cout << "ratio: " << redGreenRatio << std::endl;

    return result;
}


template class GetTrafficLightColor<float>;
template class GetTrafficLightColor<double>;
} // xycar space


#ifndef KINECTFUSION_UTIL_H
#define KINECTFUSION_UTIL_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Weffc++"
#include <opencv2/imgproc.hpp>
#pragma GCC diagnostic pop

cv::Mat color_normal(const cv::Mat& normal_map)
{
    cv::Mat output(normal_map.size(), CV_8UC3);
    for (int y = 0; y < normal_map.rows; y++) {
        for (int x = 0; x < normal_map.cols; x++) {
            auto& col = output.at<cv::Vec3b>(y, x);
            col[0] = col[1] = col[2] = 255;
            const auto& normal = normal_map.at<const cv::Vec3f>(y, x);
            if (normal[2] != 0) {
                col[0] = static_cast<uchar>((normal[0] + 1) / 2 * 255);
                col[1] = static_cast<uchar>((normal[1] + 1) / 2 * 255);
                col[2] = static_cast<uchar>((normal[2] + 1) / 2 * 255);
            }
        }
    }
    return output;
}

cv::Mat color_depth(const cv::Mat& depth_map)
{
    cv::Mat output = depth_map.clone();
    double min, max;
    cv::minMaxIdx(depth_map, &min, &max);
    output -= min;
    cv::convertScaleAbs(output, output, 255 / (max - min));
    cv::applyColorMap(output, output, cv::COLORMAP_JET);
    return output;
}

#endif //KINECTFUSION_UTIL_H

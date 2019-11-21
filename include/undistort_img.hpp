#ifndef UNDISTORT_IMG_H
#define UNDISTORT_IMG_H

#include <cstring>
#include <opencv2/core/mat.hpp>

void undistort_img(const cv::Mat& img_in, cv::Mat& img_out, 
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder);
#endif

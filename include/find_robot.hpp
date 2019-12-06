#ifndef FIND_ROBOT
#define FIND_ROBOT

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include "utils.hpp"

#define PI 3.14159265

const int MAX_WIDTH = 800;
const int MAX_HEIGHT = 600;

bool find_Robot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder);


#endif

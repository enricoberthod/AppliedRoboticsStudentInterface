#ifndef PROCESS_MAP
#define PROCESS_MAP

#include <cstring>
#include <opencv2/core/mat.hpp>
#include "utils.hpp"

const double MIN_AREA_SIZE = 500; 

/////RED_MASK_LOW
const int RLlr = 0, RLlg = 50, RLlb = 50; 
const int RLhr = 10, RLhg = 255, RLhb = 255;

/////RED_MASK_HIGH
const int RHlr = 160, RHlg = 50, RHlb = 50;
const int RHhr = 179, RHhg = 255, RHhb = 255;

/////GREEN_MASK
const int Glr = 45, Glg = 100, Glb = 40;
const int Ghr = 75, Ghg = 255, Ghb = 255;

//std::string template_folder = "/home/lar2019/Desktop/AppliedRoboticsStudentInterface/new_template/";
std::string template_folder = "/home/robotics/workspace/group_3/new_template/";


float f_linea(cv::Mat src, cv::Mat dst, cv::Point& C);
void findVictim(cv::Mat hsv_img, cv::Mat img_in, cv::Mat kernel, const double scale, std::vector<std::pair<int,Polygon>>& victim_list);
void findGate(cv::Mat hsv_img, cv::Mat img_in, cv::Mat kernel, const double scale, Polygon& gate);
void redRegions(cv::Mat hsv_img, cv::Mat img_in, cv::Mat kernel, const double scale, std::vector<Polygon>& obstacle_list);
bool process_Map(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder);
#endif

#ifndef PlanPath_Interface_H
#define PlanPath_Interface_H

#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include "utils.hpp"
//using namespace std;


void plan_Path123(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, 
				const Polygon& gate, const float x, const float y, const float theta, Path& path, const std::string& config_folder);

#endif


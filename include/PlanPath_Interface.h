#ifndef PlanPath_Interface_H
#define PlanPath_Interface_H

#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include "utils.hpp"
#include <vector> 
#include <string>
#include <iostream>
#include <cmath>
#include <unordered_map>
#include "Voronoi.h"
#include "PathFinder.h"
#include "Dubins.h"
#include "clipper.hpp"

bool plan_Path_2(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, 
				const Polygon& gate, const float x, const float y, const float theta, Path& path, const std::string& config_folder);

bool sample(C, Path&, int&);
std::vector<float> IDP(float, std::vector<VoronoiPoint> &, Path &);
void function_L_doppio(int, float, std::vector<VoronoiPoint> &, std::vector<float> &, Path &, bool);
bool collision_detection(double, double);

#endif


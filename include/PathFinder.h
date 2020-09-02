//PathFinder.h

#ifndef PathFinder_H
#define PathFinder_H
#include "DataStructure.h"
#include "utils.hpp"
#include "Voronoi.h"
#include <opencv2/imgproc/imgproc.hpp>

void PathFinder(VoronoiPoint, bool, VoronoiPoint, bool, VoronoiResults*, std::vector<VoronoiPoint> *, int, const std::vector<std::vector<cv::Point>>&, const std::vector<std::pair<int,Polygon>> *, const std::string&);
void sampleSegment2(VoronoiPoint, VoronoiPoint, std::vector<VoronoiPoint>&);
bool edgeOnObstacle(VoronoiPoint, VoronoiPoint, const std::vector<std::vector<cv::Point>>&);

#endif

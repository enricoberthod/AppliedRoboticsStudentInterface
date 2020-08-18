//PathFinder.h

#ifndef PathFinder_H
#define PathFinder_H
#include "DataStructure.h"
#include "Voronoi.h"
#include <opencv2/imgproc/imgproc.hpp>

void PathFinder(VoronoiPoint, bool, VoronoiPoint, bool, VoronoiResults*, std::vector<VoronoiPoint> *, int, const std::vector<std::vector<cv::Point>>&);

#endif

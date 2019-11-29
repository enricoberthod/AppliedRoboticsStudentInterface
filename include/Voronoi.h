//Voronoi.h

#ifndef Voronoi_H
#define Voronoi_H
#include "DataStructure.h"

struct VoronoiPoint {
	float a;
	float b;
  	VoronoiPoint(float x, float y) : a(x), b(y) {}
};

struct Segment {
  	VoronoiPoint p0;
 	VoronoiPoint p1;
  	Segment(float x1, float y1, float x2, float y2) : p0(x1, y1), p1(x2, y2) {}
};

struct GraphEdge{
  	VoronoiPoint p0;
 	VoronoiPoint p1;
	float length;
	int idFirstNode,idSecondNode;
  	GraphEdge(float x1, float y1, float x2, float y2) : p0(x1, y1), p1(x2, y2), length(0), idFirstNode(0), idSecondNode(0) {}
	GraphEdge(float x1, float y1, float x2, float y2, float len) : p0(x1, y1), p1(x2, y2), length(len), idFirstNode(0), idSecondNode(0) {}
	GraphEdge(float x1, float y1, float x2, float y2, float len, float id1, float id2) : p0(x1, y1), p1(x2, y2), length(len), idFirstNode(id1), idSecondNode(id2) {}
};


struct VoronoiResults {
	std::vector<int> ids;
	std::vector<GraphEdge> resultEdges;
	std::vector<VoronoiPoint> resultPoints;
};

void Voronoi(std::vector<VoronoiPoint>,std::vector<Segment>, VoronoiResults*);

#endif

//Voronoi.h

#ifndef Voronoi_H
#define Voronoi_H
#include "DataStructure.h"
#include <unordered_map>

struct VoronoiPoint {
	int a;
	int b;
  	
	int getA(){return a;};
	int getB(){return b;};
	VoronoiPoint(int x, int y) : a(x), b(y) {};
	VoronoiPoint() : a(0), b(0) {};
};

struct Segment {
  	VoronoiPoint p0;
 	VoronoiPoint p1;
  	Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
};

struct GraphEdge{
  	VoronoiPoint p0;
 	VoronoiPoint p1;
	double length;
	int idFirstNode,idSecondNode;
  	GraphEdge(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2), length(0), idFirstNode(0), idSecondNode(0) {}
	GraphEdge(int x1, int y1, int x2, int y2, double len) : p0(x1, y1), p1(x2, y2), length(len), idFirstNode(0), idSecondNode(0) {}
	GraphEdge(int x1, int y1, int x2, int y2, double len, int id1, int id2) : p0(x1, y1), p1(x2, y2), length(len), idFirstNode(id1), idSecondNode(id2) {}
};


struct VoronoiResults {
	std::vector<int> ids;
	std::vector<GraphEdge> resultEdges;
	std::vector<VoronoiPoint> resultPoints;
};

//void Voronoi(std::vector<VoronoiPoint>,std::vector<Segment>,std::unordered_map<int,VoronoiPoint>, VoronoiResults*);
void Voronoi(std::vector<VoronoiPoint>,std::vector<Segment>,std::unordered_map<int,VoronoiPoint>, const std::vector<std::vector<cv::Point>>&, VoronoiResults*);
int encoder(int, int);
void decoder(int, int &, int &);

#endif

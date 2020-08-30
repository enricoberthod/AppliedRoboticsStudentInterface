// Boost.Polygon library voronoi_basic_tutorial.cpp file

//          Copyright Andrii Sydorchuk 2010-2012.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

// See http://www.boost.org for updates, documentation, and revision history.

#include "Voronoi.h"

#include <cstdio>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <cmath>
#include <unordered_map>

#include <boost/polygon/voronoi.hpp>
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;
using boost::polygon::low;
using boost::polygon::high;

//bool isObstaclePoint(double, double, std::unordered_map<int,VoronoiPoint>);
bool isObstaclePoint(int, int, const std::vector<std::vector<cv::Point>>& contours);
void removeObstacles(std::vector<VoronoiPoint>, VoronoiResults*);
void remove(std::vector<VoronoiPoint>, VoronoiResults*);
int visited(std::vector<int>, int);
//int encoder(int, int);
//void decoder(int, int &, int &);

//#include "voronoi_visual_utils.hpp"

namespace boost {
namespace polygon {

template <>
struct geometry_concept<VoronoiPoint> {
  typedef point_concept type;
};

template <>
struct point_traits<VoronoiPoint> {
  typedef int coordinate_type;

  static inline coordinate_type get(
      const VoronoiPoint& point, orientation_2d orient) {
    return (orient == HORIZONTAL) ? point.a : point.b;
  }
};

template <>
struct geometry_concept<Segment> {
  typedef segment_concept type;
};

template <>
struct segment_traits<Segment> {
  typedef int coordinate_type;
  typedef VoronoiPoint point_type;

  static inline point_type get(const Segment& segment, direction_1d dir) {
    return dir.to_int() ? segment.p1 : segment.p0;
  }
};

}  // polygon
}  // boost

// Traversing Voronoi edges using edge iterator.
int iterate_primary_edges1(const voronoi_diagram<double>& vd) {
  int result = 0;
  for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin();
       it != vd.edges().end(); ++it) {
    if (it->is_primary())
      ++result;
  }
  return result;
}

// Traversing Voronoi edges using cell iterator.
int iterate_primary_edges2(const voronoi_diagram<double> &vd) {
  int result = 0;
  for (voronoi_diagram<double>::const_cell_iterator it = vd.cells().begin();
       it != vd.cells().end(); ++it) {
    const voronoi_diagram<double>::cell_type& cell = *it;
    const voronoi_diagram<double>::edge_type* edge = cell.incident_edge();
    // This is convenient way to iterate edges around Voronoi cell.
    do {
      if (edge->is_primary())
        ++result;
      edge = edge->next();
    } while (edge != cell.incident_edge());
  }
  return result;
}

// Traversing Voronoi edges using vertex iterator.
// As opposite to the above two functions this one will not iterate through
// edges without finite endpoints and will iterate only once through edges
// with single finite endpoint.
//int iterate_primary_edges3(const voronoi_diagram<double> &vd, std::unordered_map<int,VoronoiPoint>*points_map, VoronoiResults *results) {
int iterate_primary_edges3(const voronoi_diagram<double> &vd, std::unordered_map<int,VoronoiPoint>*points_map, const std::vector<std::vector<cv::Point>>& contours, VoronoiResults *results) {
  	int result = 0;
	const voronoi_diagram<double>::vertex_type* startVertex;
	const voronoi_diagram<double>::vertex_type* endVertex;
	int xa,ya,xb,yb=0;
	double prev_xa=-1;
	double prev_ya=-1;
	int id1,id2,longId=0;
	std::vector<int> visitedIds;
	int max_X = 1560;
	int max_Y = 1060;
	
    	const voronoi_diagram<double>::edge_type* edge;
	
  	for (voronoi_diagram<double>::const_vertex_iterator it = vd.vertices().begin(); it != vd.vertices().end(); ++it) 
	{
    		const voronoi_diagram<double>::vertex_type& vertex = *it;
    		edge = vertex.incident_edge();
		double x = vertex.x();
		double y = vertex.y();
		double len;
		
		//VoronoiPoint point(x,y);
		//results->resultPoints.push_back(point);
    // This is convenient way to iterate edges around Voronoi vertex.
		
		
    		do {
      			if (edge->is_primary())
			{
        			++result;
				startVertex=edge->vertex0();
				endVertex=edge->vertex1();
				if(startVertex!=NULL && endVertex!=NULL)
				{
					xa = (int)startVertex->x();
					ya = (int)startVertex->y();
					//if(!isObstaclePoint(xa,ya,*points_map))
					if(!isObstaclePoint(xa,ya,contours))
					{
						xb = (int)endVertex->x();  
						yb = (int)endVertex->y();
						len=sqrt(pow((xa-xb),2)+pow((ya-yb),2));
						if(xa!=prev_xa || ya!=prev_ya)
						{
						/*
							if(ya<1000)
								longId=(xa*1000)+ya;
							else
								longId=(xa*10000)+ya;*/
							longId=encoder(xa,ya);
							id1=visited(visitedIds,longId);
							if(id1==-1)
							{
								visitedIds.push_back(longId);
								id1=(visitedIds.size()-1);
							}
						}
						/*
						if(yb<1000)
							longId=(xb*1000)+yb;
						else
							longId=(xb*10000)+yb;*/
						longId=encoder(xb,yb);
						id2=visited(visitedIds,longId);
						if(id2==-1)
						{
							visitedIds.push_back(longId);
							id2=(visitedIds.size()-1);
						}
						if(xa>=0 && xa<=max_X && ya>=0 && ya<=max_Y && xb>=0 && xb<=max_X && yb>=0 && yb<=max_Y)
						{
							GraphEdge e(xa,ya,xb,yb,len,id1,id2);
							results->resultEdges.push_back(e);
							VoronoiPoint p(xa,ya);
							results->resultPoints.push_back(p); 
						}
					}
					prev_xa=xa;
					prev_ya=ya;
					//printf("vertex (%i, %i): (%i,%i),(%i,%i)  \n", id1,id2,xa,ya,xb,yb);
				}
			}
      			edge = edge->rot_next();
		} while (edge != vertex.incident_edge());
  	}
	//if(!isObstaclePoint(xb,yb,*points_map))
	//Add the last node to the result points array
	if(!isObstaclePoint(xb,yb,contours))
	{
		if(xb>=0 && xb<=max_X && yb>=0 && yb<=max_Y)
		{
			VoronoiPoint p(xb,yb);
			results->resultPoints.push_back(p);
		}
	}
	results->ids=visitedIds;	

  	return result;
}

int visited(std::vector<int> visitedIds, int num)
{
	for(int i=0; i<visitedIds.size(); i++)
		if(visitedIds[i]==num)
			return i;
	return -1;
}

//void Voronoi(std::vector<VoronoiPoint>points, std::vector<Segment> segments, std::unordered_map<int,VoronoiPoint>points_map, VoronoiResults *results)
void Voronoi(std::vector<VoronoiPoint>points, std::vector<Segment> segments, std::unordered_map<int,VoronoiPoint>points_map, const std::vector<std::vector<cv::Point>>& contours, VoronoiResults *results)
// si può aggiungere un flag isObstaclePoint alla struttura di GraphEdge così da marchiare subito i vertici da togliere
{
  
	//printf("pppp %i\n",p.size());
  // Construction of the Voronoi Diagram.
  voronoi_diagram<double> vd;
  construct_voronoi(points.begin(), points.end(), segments.begin(), segments.end(), &vd); 
	printf("pippo c'è \n");
  // Traversing Voronoi Graph.
  {
    printf("Traversing Voronoi graph.\n");
    //printf("Number of visited primary edges using edge iterator: %d\n", iterate_primary_edges1(vd));
		printf("pippo c'è \n");
    //printf("Number of visited primary edges using cell iterator: %d\n", iterate_primary_edges2(vd));
	//printf("Number of visited primary edges using vertex iterator: %d\n", iterate_primary_edges3(vd, &points_map, results));
    printf("Number of visited primary edges using vertex iterator: %d\n", iterate_primary_edges3(vd, &points_map, contours, results));
    printf("before points size: %i \n",results->resultPoints.size());
    printf("before edge size: %i \n",results->resultEdges.size());

	//removeObstacles(points,results);

    //printf("after edge size: %i \n",results->resultEdges.size());
  }
	
	
}
/*
bool isObstaclePoint(double x, double y, std::unordered_map<int,VoronoiPoint>points_map)
{
	*//*
	for(int i=0; i<points.size();i++)
	{
		if(x==points[i].a && y==points[i].b)
			return true;
	} *//*
	//return false;
	VoronoiPoint p;
	int longId;
	if(y<1000)
		longId=(x*1000)+y;
	else
		longId=(x*10000)+y;
	if(points_map.find(longId)==points_map.end())
		return false;
	else
		return true;
	//return points_map.contains(longId);
	
}
*/

int encoder(int x, int y)
{
	return (x*10000)+y;
}

void decoder(int encoded, int &x, int &y)    
{
	x=encoded/10000;
	y=encoded-(x*10000);
}

bool isObstaclePoint(int x, int y, const std::vector<std::vector<cv::Point>>& contours) {
	bool r = false;
	double res;
	for (int i = 0; i < contours.size() && !r; i++) {
		res = cv::pointPolygonTest(contours[i] , cv::Point2f(x,y) , true);
		if(res >= 0 || x == 0 || x == 1560 || y == 0 || y == 1060) {
			r = true;
			//std::cout << "INSIDE!!! " << res << " -> " << " punto <" << x << ", " << y << ">" << " -> " << r << std::endl;
		}
		//else {
			//std::cout << "OUTSIDE " << res << " punto <" << x << ", " << y << ">" << " -> " << r << std::endl;
		//}
	}
	return r;
}

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

bool isObstaclePoint(int, int, const std::vector<std::vector<cv::Point>>& contours);
void removeObstacles(std::vector<VoronoiPoint>, VoronoiResults*);
void remove(std::vector<VoronoiPoint>, VoronoiResults*);
int visited(std::vector<int>, int);

int victimGain;
int max_X;
int max_Y;

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

//traversing Voronoi edges using edge iterator.
int iterate_primary_edges1(const voronoi_diagram<double>& vd) {
  int result = 0;
  for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin();
       it != vd.edges().end(); ++it) {
    if (it->is_primary())
      ++result;
  }
  return result;
}

//traversing Voronoi edges using cell iterator.
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
int iterate_primary_edges3(const voronoi_diagram<double> &vd, std::unordered_map<int,VoronoiPoint>*points_map, const std::vector<std::vector<cv::Point>>& contours, VoronoiResults *results) 
{
  	int result = 0;
	const voronoi_diagram<double>::vertex_type* startVertex;
	const voronoi_diagram<double>::vertex_type* endVertex;
	int xa,ya,xb,yb=0;
	double prev_xa=-1;
	double prev_ya=-1;
	int id1,id2,longId=0;
	std::vector<int> visitedIds;
	
    	const voronoi_diagram<double>::edge_type* edge;
	
  	for (voronoi_diagram<double>::const_vertex_iterator it = vd.vertices().begin(); it != vd.vertices().end(); ++it) 
	{
    		const voronoi_diagram<double>::vertex_type& vertex = *it;
    		edge = vertex.incident_edge();
		double x = vertex.x();
		double y = vertex.y();
		double len;
		
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
					
					//if the point is not an obstacle 
					if(!isObstaclePoint(xa,ya,contours))
					{
						xb = (int)endVertex->x();  
						yb = (int)endVertex->y();
						//compute the distance between A e B
						len=sqrt(pow((xa-xb),2)+pow((ya-yb),2));
						//add the space gain that the robot ears if it saves a victim 
						len=len+victimGain;
						
						if(xa!=prev_xa || ya!=prev_ya)
						{
							longId=encoder(xa,ya);
							id1=visited(visitedIds,longId);
							//if not already visited, mark as visited node
							if(id1==-1)
							{
								visitedIds.push_back(longId);
								id1=(visitedIds.size()-1);
							}
						}
					
						longId=encoder(xb,yb);
						id2=visited(visitedIds,longId);
						//if not already visited, mark as visited node
						if(id2==-1)
						{
							visitedIds.push_back(longId);
							id2=(visitedIds.size()-1);
						}
						//if is inside the arena
						if(xa>margine && xa<max_X && ya>margine && ya<max_Y && xb>margine && xb<max_X && yb>margine && yb<max_Y)	
						{
							//add at the result edges the edge between A e B and viceversa
							GraphEdge e(xa,ya,xb,yb,len,id1,id2);
							GraphEdge e1(xb,yb,xa,ya,len,id2,id1);
							results->resultEdges.push_back(e);
							results->resultEdges.push_back(e1);
							VoronoiPoint p(xa,ya);
							results->resultPoints.push_back(p); 
						}
					}
					prev_xa=xa;
					prev_ya=ya;
				}
			}
      			edge = edge->rot_next();
		} while (edge != vertex.incident_edge());
  	}
	
	//add the last node to the result points array
	//if the point is not an obstacle 
	if(!isObstaclePoint(xb,yb,contours))
	{
		//if is inside the arena
		if(xb>margine && xb<max_X && yb>margine && yb<max_Y)		
		{
			VoronoiPoint p(xb,yb);
			results->resultPoints.push_back(p);
		}
	}
	results->ids=visitedIds;	

  	return result;
}


/* function visited: function which returns if a node is new or not
   -parameters:
   	visitedIds: array of visited nodes
	num: node number to check 
    -return: the position in the visited array or -1 if not found 
*/
int visited(std::vector<int> visitedIds, int num)
{
	for(int i=0; i<visitedIds.size(); i++)
		if(visitedIds[i]==num)
			return i;
	return -1;
}

/* function Voronoi: function which returns if a node is new or not 
   -parameters:
   	points: obstacles points
	segments: obstacles segments
	points_map: obstacles verteces
	contours: obstacle contours
	b_x_max: maximum x to stay inside the border without collisions
	b_y_max: maximum y to stay inside the border without collisions
	results: the structure to store the road map
	config_folder: path to reach the param.xml config file
    -return: none
*/
void Voronoi(std::vector<VoronoiPoint>points, std::vector<Segment> segments, std::unordered_map<int,VoronoiPoint>points_map, const std::vector<std::vector<cv::Point>>& contours, float b_x_max, float b_y_max, VoronoiResults *results, const std::string& config_folder)
{
  //read parameter from param.xml	
  std::string file = config_folder+"/param.xml";
  cv::FileStorage param(file, cv::FileStorage::READ);

  //victimGain indicates the gain in space the robot earns if it saves a victim (m2 porpuses) 
  victimGain = (int)param["victimGain"]; //TODO add * spead_robot	

  max_X = (int)b_x_max;
  max_Y = (int)b_y_max;

  //construction of the Voronoi Diagram.
  voronoi_diagram<double> vd;
  construct_voronoi(points.begin(), points.end(), segments.begin(), segments.end(), &vd); 
	
  //traversing Voronoi Graph.
  iterate_primary_edges3(vd, &points_map, contours, results);	
}

/* function Voronoi: function which computes the encoded sequence x y 
   -parameters:
   	x: x coordinate
	y: y coordinate
    -return: the encoded sequence x y 
*/
int encoder(int x, int y)
{
	return (x*10000)+y;
}


/* function Voronoi: function which computes x, y from the encoded sequence x y 
   -parameters:
   	encoded: the encoded sequence x y 
   	x: solution x coordinate
	y: solution y coordinate
    -return: none
*/
void decoder(int encoded, int &x, int &y)    
{
	x=encoded/10000;
	y=encoded-(x*10000);
}


/* function isObstaclePoint: function which computes if a point is on the obstacle
   -parameters:
   	x: x coordinate
	y: y coordinate
	contours: vectors of obstacles 
    -return: true if the point is on the obstacle, false if not
*/
bool isObstaclePoint(int x, int y, const std::vector<std::vector<cv::Point>>& contours) {
	bool r = false;
	double res;
	//for each obstacle
	for (int i = 0; i < contours.size() && !r; i++) {
		//check if the point is inside the obstacle
		res = cv::pointPolygonTest(contours[i] , cv::Point2f(x,y) , true);
		if(res >= 0 || x <= margine || x >= max_X || y <= margine || y >= max_Y) {		
			r = true;		
		}
	}
	return r;
}

// Boost.Polygon library voronoi_basic_tutorial.cpp file

//          Copyright Andrii Sydorchuk 2010-2012.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

// See http://www.boost.org for updates, documentation, and revision history.

#include "Voronoi.h"

#include <cstdio>
#include <vector>
#include <cmath>

#include <boost/polygon/voronoi.hpp>
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;
using boost::polygon::low;
using boost::polygon::high;

bool isObstaclePoint(float, float, std::vector<VoronoiPoint>);
void removeObstacles(std::vector<VoronoiPoint>, VoronoiResults*);
void remove(std::vector<VoronoiPoint>, VoronoiResults*);
int visited(std::vector<int>, int);

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
int iterate_primary_edges3(const voronoi_diagram<double> &vd, std::vector<VoronoiPoint>* points, VoronoiResults *results) {
  	int result = 0;
	const voronoi_diagram<double>::vertex_type* startVertex;
	const voronoi_diagram<double>::vertex_type* endVertex;
	float xa,ya,xb,yb=0;
	double prev_xa=-1;
	double prev_ya=-1;
	int id1,id2,longId=0;
	std::vector<int> visitedIds;
	
	
    	const voronoi_diagram<double>::edge_type* edge;
	
  	for (voronoi_diagram<double>::const_vertex_iterator it = vd.vertices().begin(); it != vd.vertices().end(); ++it) 
	{
    		const voronoi_diagram<double>::vertex_type& vertex = *it;
    		edge = vertex.incident_edge();
		float x = vertex.x();
		float y = vertex.y();
		float len;
		int mostId,lessId;
		
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
					xa = (float)startVertex->x();
					ya = (float)startVertex->y();
					xb = (float)endVertex->x();  
					yb = (float)endVertex->y();
					len=sqrt(pow((xa-xb),2)+pow((ya-yb),2));
					if(xa!=prev_xa || ya!=prev_ya)
					{
						mostId=(int)(xa*1000);
						lessId=(int)(ya*1000);
						if(ya<1)
						{
											
							longId=(mostId*1000)+lessId;
						}
						else
						{
							
							longId=(mostId*10000)+lessId;

						}
						id1=visited(visitedIds,longId);
						if(id1==-1)
						{
							visitedIds.push_back(longId);
							id1=(visitedIds.size()-1);
						}
					}
					mostId=(int)(xb*1000);
					lessId=(int)(yb*1000);
					if(yb<1)
						longId=(mostId*1000)+lessId; 
					else
						longId=(mostId*10000)+lessId; 
					id2=visited(visitedIds,longId);
					if(id2==-1)
					{
						visitedIds.push_back(longId);
						id2=(visitedIds.size()-1);
					}
					GraphEdge e(xa,ya,xb,yb,len,id1,id2);
					results->resultEdges.push_back(e);
					prev_xa=xa;
					prev_ya=ya;
					printf("vertex (%i, %i): (%f,%f),(%f,%f)  \n", id1,id2,xa,ya,xb,yb);
				}
			}
      			edge = edge->rot_next();
		} while (edge != vertex.incident_edge());
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

void Voronoi(std::vector<VoronoiPoint>points, std::vector<Segment> segments, VoronoiResults *results)  // si può aggiungere un flag isObstaclePoint alla struttura di GraphEdge così da marchiare subito i vertici da togliere
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
    printf("Number of visited primary edges using vertex iterator: %d\n", iterate_primary_edges3(vd, &points, results));
    printf("before edge size: %i \n",results->resultEdges.size());
//////////////////////////// check why the process dies in remove
	removeObstacles(points,results);

    printf("after edge size: %i \n",results->resultEdges.size());
  }
	
	
}

bool isObstaclePoint(float x, float y, std::vector<VoronoiPoint>points)
{
	for(int i=0; i<points.size();i++)
	{
		if(x==points[i].a && y==points[i].b)
			return true;
	}
	return false;
}

void removeObstacles(std::vector<VoronoiPoint>obstPoints, VoronoiResults *results)
{
	std::vector<VoronoiPoint>obstacleVertices;
	bool found=false;
	obstacleVertices.clear();
	 printf("removeObstacle %i\n",results->resultEdges.size());
	for(int i=0;i<results->resultEdges.size() && !found;i++)
	{
		printf("i %i \n",i);
		if(isObstaclePoint(results->resultEdges[i].p0.a,results->resultEdges[i].p0.b,obstPoints))
		{
			found=true;
			printf("true\n");
			obstacleVertices.push_back(VoronoiPoint(results->resultEdges[i].p1.a,results->resultEdges[i].p1.b));
			printf("push\n");
		}
		else
		{
			printf("false\n");
			results->resultPoints.push_back(results->resultEdges[i].p0);
			printf("push\n");
		}
	}
	
	if(found==true)
	{
		remove(obstPoints, results);
		printf("enrico dice che è qui\n");
		removeObstacles(obstacleVertices, results);
	}
}

void remove(std::vector<VoronoiPoint>points, VoronoiResults *results)
{
	std::vector<GraphEdge>::iterator it;
	printf("remove %i\n",points.size());
	for(int i=0; i<points.size();i++)
	{
		printf("i2 %i\n",i);
		for(it=results->resultEdges.begin(); it!=results->resultEdges.end(); it++)
		{
			if((it->p0.a==points[i].a && it->p0.b==points[i].b) || (it->p1.a==points[i].a && it->p1.b==points[i].b))
			{
				//printf("erase \n");
				results->resultEdges.erase(it);
				//printf("erase 2 \n");
			}
		} 
	}
}


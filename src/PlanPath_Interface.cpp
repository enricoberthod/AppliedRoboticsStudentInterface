#include "PlanPath_Interface.h"

#include <vector> 
#include <string>
#include <iostream>
#include "Voronoi.h"
#include "PathFinder.h"
#include "Dubins.h"

//using namespace std;

void plan_Path123(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, 
				const Polygon& gate, const float x, const float y, const float theta, Path& path, const std::string& config_folder)
{
	path.points.emplace_back(1,2,3,1,2);

	//Robot center
	VoronoiPoint start = VoronoiPoint(x,y);
	float x_tot=0,y_tot=0;
	
	for(int i=0;i<gate.size();i++)
	{
		x_tot+=gate[i].x;
		y_tot+=gate[i].y;
	}
	
	VoronoiPoint end = VoronoiPoint(x_tot/4,y_tot/4); 
	
	//data structure for results
	VoronoiResults voronoiPaths;
	

	//ADD CLIPPER TO ENLARGE THE OBSTACLES //////////////////////////////////////////////////

	std::vector<VoronoiPoint> inputPoints;
	std::vector<Segment> obstacles_edges;

	//create a vector of points and obstacles edges
	int vertexNumber;
	float xa,xb,ya,yb;
	for(int i=0;i<obstacle_list.size();i++)
	{
		vertexNumber=obstacle_list[i].size();
		for(int j=0;j<vertexNumber-1;j++)
		{
			printf("obsvertx: %f,%f\n",xa,ya);
			xa=obstacle_list[i][j].x;
			ya=obstacle_list[i][j].y;
			xb=obstacle_list[i][j+1].x;
			yb=obstacle_list[i][j+1].y;
			inputPoints.push_back(VoronoiPoint(xa,ya));
			obstacles_edges.push_back(Segment(xa, ya, xb, yb));
		}
		//close the polygon with the last edge
		xa=obstacle_list[i][vertexNumber-1].x;
		ya=obstacle_list[i][vertexNumber-1].y;
		xb=obstacle_list[i][0].x;
		yb=obstacle_list[i][0].y;
		inputPoints.push_back(VoronoiPoint(xa,ya));
		obstacles_edges.push_back(Segment(xa, ya, xb, yb));
	}

	
	
	//add borders
	for(int i=0;i<3;i++)
	{
		xa=borders[i].x;
		ya=borders[i].y;
		xb=borders[i+1].x;
		yb=borders[i+1].y;
		printf("bordivertx: %f,%f\n",xa,ya);
		inputPoints.push_back(VoronoiPoint(xa,ya));
		obstacles_edges.push_back(Segment(xa, ya, xb, yb));
	}
	//close the borders
	xa=borders[3].x;
	ya=borders[3].y;
	xb=borders[0].x;
	yb=borders[0].y;
	printf("bordivertx: %f,%f\n",xa,ya);
	inputPoints.push_back(VoronoiPoint(xa,ya));
	obstacles_edges.push_back(Segment(xa, ya, xb, yb));

	printf("size: %i", inputPoints.size());
	printf("size2: %i",obstacles_edges.size());	
	

	//Creates the Voronoi map
	Voronoi(inputPoints, obstacles_edges, &voronoiPaths);
	


	//data structure for results
	std::vector<VoronoiPoint> rightPath;

	printf("vectorvoronoiPath: %i\n", voronoiPaths.resultPoints.size());
	
	for(int j=0;j<voronoiPaths.resultPoints.size();j++)
	{
		printf("x: %f",voronoiPaths.resultPoints[j].a);
		printf(", %f\n",voronoiPaths.resultPoints[j].b);
	}

	throw std::logic_error( "STOP" );
	
	//Direct path from robot center initial position to gate center
	PathFinder(start, end, &voronoiPaths, &rightPath); 
	
	throw std::logic_error( "STOP" );

	///////////////////////////////ADD SAMPLING POINTS OF THE RIGHTPATH

	//Calculate dubins curves on the path initial_robot - gate	
	int pidx;
	D curve;
	float s,pt_x,pt_y,pt_theta,kappa;
	
	for(int i=0; i<rightPath.size()-1; i++)
	{	
		//Calculate Dubins curve from one node to the next one
		Dubins(rightPath[i].a,rightPath[i].b,rightPath[i+1].a,rightPath[i+1].b, &curve, &pidx); //add initial theta in and theta out.
		
		/*//Create a structure representing an arc of a Dubins curve (straight or circular)
		struct C{ double x0,y0,th0,k,L,xf,yf,thf; };
		
		// Create a structure representing a Dubins curve (composed by three arcs)
		struct D{ C a1,a2,a3; double L = a1.L+a2.L+a3.L; };*/
		
		pt_x = curve.a1.xf;
		pt_y = curve.a1.yf;
		pt_theta = curve.a1.thf;
		kappa = curve.a1.k;
		s = curve.a1.L;
		path.points.emplace_back(s,pt_x,pt_y,pt_theta,kappa);
		
		pt_x = curve.a2.xf;
		pt_y = curve.a2.yf;
		pt_theta = curve.a2.thf;
		kappa = curve.a2.k;
		s = curve.a2.L;
		path.points.emplace_back(s,pt_x,pt_y,pt_theta,kappa);
		
		pt_x = curve.a3.xf;
		pt_y = curve.a3.yf;
		pt_theta = curve.a3.thf;
		kappa = curve.a3.k;
		s = curve.a3.L;
		path.points.emplace_back(s,pt_x,pt_y,pt_theta,kappa);
	} 
}
  

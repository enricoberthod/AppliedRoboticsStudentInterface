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
	const int floatToInt=1000;
	//path.points.emplace_back(1,2,3,1,2);

	//Robot center
	VoronoiPoint start = VoronoiPoint((int)(x*floatToInt),(int)(y*floatToInt));
	//VoronoiPoint start = VoronoiPoint((int)(0.2*floatToInt),(int)(0.1*floatToInt));
	float x_tot=0,y_tot=0;
	
	for(int i=0;i<gate.size();i++)
	{
		x_tot+=gate[i].x;
		y_tot+=gate[i].y;
	}
	
	VoronoiPoint end = VoronoiPoint((int)((x_tot/4)*floatToInt),(int)((y_tot/4)*floatToInt)); 
	
	//data structure for results
	VoronoiResults voronoiPaths;
	

	//ADD CLIPPER TO ENLARGE THE OBSTACLES //////////////////////////////////////////////////

	std::vector<VoronoiPoint> inputPoints;
	std::vector<Segment> obstacles_edges;

	//create a vector of points and obstacles edges
	int vertexNumber;
	
	int xa,xb,ya,yb;
	for(int i=0;i<obstacle_list.size();i++)
	{
		vertexNumber=obstacle_list[i].size();
		for(int j=0;j<vertexNumber-1;j++)
		{
			printf("obsvertxf: %f,%f\n",obstacle_list[i][j].x,obstacle_list[i][j].y);
			xa=(int)(obstacle_list[i][j].x*floatToInt);
			ya=(int)(obstacle_list[i][j].y*floatToInt);
			printf("obsvertx: %i,%i\n",xa,ya);
			xb=(int)(obstacle_list[i][j+1].x*floatToInt);
			yb=(int)(obstacle_list[i][j+1].y*floatToInt);
			inputPoints.push_back(VoronoiPoint(xa,ya));
			obstacles_edges.push_back(Segment(xa, ya, xb, yb));
		}
		//close the polygon with the last edge
		xa=(int)(obstacle_list[i][vertexNumber-1].x*floatToInt);
		ya=(int)(obstacle_list[i][vertexNumber-1].y*floatToInt);
		xb=(int)(obstacle_list[i][0].x*floatToInt);
		yb=(int)(obstacle_list[i][0].y*floatToInt);
		inputPoints.push_back(VoronoiPoint(xa,ya));
		obstacles_edges.push_back(Segment(xa, ya, xb, yb));
	}

	
	
	//add borders
	for(int i=0;i<3;i++)
	{
		xa=(int)((borders[i].x*floatToInt)>0?(borders[i].x*floatToInt):0);
		ya=(int)((borders[i].y*floatToInt)>0?(borders[i].y*floatToInt):0);
		xb=(int)((borders[i+1].x*floatToInt)>0?(borders[i+1].x*floatToInt):0);
		yb=(int)((borders[i+1].y*floatToInt)>0?(borders[i+1].y*floatToInt):0);
		
		printf("bordivertxf: %f,%f\n",borders[i].x,borders[i].y);
		printf("bordivertx: %i,%i\n",xa,ya);
		inputPoints.push_back(VoronoiPoint(xa,ya));
		obstacles_edges.push_back(Segment(xa, ya, xb, yb));
	}
	//close the borders
	xa=(int)((borders[3].x*floatToInt)>0?(borders[3].x*floatToInt):0);
	ya=(int)((borders[3].y*floatToInt)>0?(borders[3].y*floatToInt):0);
	xb=(int)((borders[0].x*floatToInt)>0?(borders[0].x*floatToInt):0);
	yb=(int)((borders[0].y*floatToInt)>0?(borders[0].y*floatToInt):0);	
	
	printf("bordivertxf: %f,%f\n",borders[3].x,borders[3].y);
	printf("bordivertx: %i,%i\n",xa,ya);
	inputPoints.push_back(VoronoiPoint(xa,ya));
	obstacles_edges.push_back(Segment(xa, ya, xb, yb));

	printf("size: %i", inputPoints.size());
	printf("size2: %i",obstacles_edges.size());	


	//throw std::logic_error( "STOP" );

	//Creates the Voronoi map
	Voronoi(inputPoints, obstacles_edges, &voronoiPaths);
		


	//data structure for results
	std::vector<VoronoiPoint> rightPath;

	printf("edgesAfterVoronoi: %i\n", voronoiPaths.resultEdges.size());
	printf("pointsAfterVoronoi: %i\n", voronoiPaths.resultPoints.size());
	
	for(int j=0;j<voronoiPaths.resultPoints.size();j++)
	{
		printf("x: %i",voronoiPaths.resultPoints[j].a);
		printf(", %i\n",voronoiPaths.resultPoints[j].b);
	}

	//throw std::logic_error( "STOP" );
	
	//Direct path from robot center initial position to gate center
	PathFinder(start, end, &voronoiPaths, &rightPath); 
	
	throw std::logic_error( "STOP" );


/////////////////////////////////////////////////////////////////////////////////////////////////////////Tutto ok fino a qui //////////////////////////////////////////

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
  

#include "PlanPath_Interface.h"

#include <vector> 
#include <string>
#include <iostream>
#include <cmath>
#include <unordered_map>
#include "Voronoi.h"
#include "PathFinder.h"
#include "Dubins.h"
#include "clipper.hpp"

void angle_calculator(VoronoiPoint, VoronoiPoint, VoronoiPoint, double &,  double &);
void sample(C, Path&);

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


//	CLIPPER
	ClipperLib::Paths subj(obstacle_list.size());
	ClipperLib::Paths solution(obstacle_list.size());

	for(int i=0;i<obstacle_list.size();i++)
	{
		for(int j=0;j<obstacle_list[i].size();j++)//each element in obstacle list
		{
			subj[i] << ClipperLib::IntPoint(obstacle_list[i][j].x*floatToInt,obstacle_list[i][j].y*floatToInt);
		}
	}

	printf("clipper1: %i\n", subj.size());
	printf("clipper2: %i\n", obstacle_list.size());
	
	//throw std::logic_error( "STOP" );

	const int offset = 100;
	
	ClipperLib::ClipperOffset co;
	co.AddPaths(subj, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
	co.Execute(solution, offset);	 //QUESTA NON VA quindi c'è un problema da qualche parte qui!!!!!!!!!!!!!!!!!!!!!!!

	printf("clipper3: %i\n", solution.size());
	printf("clipper4: %i\n", obstacle_list.size());

	std::vector<VoronoiPoint> inputPoints;
	std::vector<Segment> obstacles_edges;

	//create an unordered_map of the obstacles' vertexes

	int xa,xb,ya,yb,longId,vertexNumber;
	VoronoiPoint p;
	std::unordered_map<int,VoronoiPoint> obstaclesVertexes;

	for(int i=0;i<solution.size();i++)
	{
		vertexNumber=solution[i].size();
		for(int j=0;j<vertexNumber-1;j++)
		{
			xa=(int)(solution[i][j].X<0?0:solution[i][j].X); //ADD CONTROL IF X>BORDER MAX X
			ya=(int)(solution[i][j].Y<0?0:solution[i][j].Y); //ADD CONTROL IF Y>BORDER MAX Y
			printf("obsvertx: %i,%i\n",xa,ya);
			xb=(int)(solution[i][j+1].X<0?0:solution[i][j+1].X);
			yb=(int)(solution[i][j+1].Y<0?0:solution[i][j+1].Y);
			if(ya<floatToInt)
				longId=(xa*floatToInt)+ya;
			else
				longId=(xa*floatToInt*10)+ya;
			p=VoronoiPoint(xa,ya);
			obstaclesVertexes[longId]=p;
			inputPoints.push_back(VoronoiPoint(xa,ya));
			obstacles_edges.push_back(Segment(xa, ya, xb, yb));
		}
		//close the polygon with the last edge
		xa=(int)(solution[i][vertexNumber-1].X<0?0:solution[i][vertexNumber-1].X);
		ya=(int)(solution[i][vertexNumber-1].Y<0?0:solution[i][vertexNumber-1].Y);
		if(ya<floatToInt)
			longId=(xa*floatToInt)+ya;
		else
			longId=(xa*floatToInt*10)+ya;
		p=VoronoiPoint(xa,ya);
		obstaclesVertexes[longId]=p;
		xb=(int)(solution[i][0].X<0?0:solution[i][0].X);
		yb=(int)(solution[i][0].Y<0?0:solution[i][0].Y);
		inputPoints.push_back(VoronoiPoint(xa,ya));
		obstacles_edges.push_back(Segment(xa, ya, xb, yb));
	}
	
	//throw std::logic_error( "STOP" );
/*
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
			if(ya<floatToInt)
				longId=(xa*floatToInt)+ya;
			else
				longId=(xa*floatToInt*10)+ya;
			p=VoronoiPoint(xa,ya);
			obstaclesVertexes[longId]=p;
			inputPoints.push_back(VoronoiPoint(xa,ya));
			obstacles_edges.push_back(Segment(xa, ya, xb, yb));
		}
		//close the polygon with the last edge
		xa=(int)(obstacle_list[i][vertexNumber-1].x*floatToInt);
		ya=(int)(obstacle_list[i][vertexNumber-1].y*floatToInt);
		if(ya<floatToInt)
			longId=(xa*floatToInt)+ya;
		else
			longId=(xa*floatToInt*10)+ya;
		p=VoronoiPoint(xa,ya);
		obstaclesVertexes[longId]=p;
		xb=(int)(obstacle_list[i][0].x*floatToInt);
		yb=(int)(obstacle_list[i][0].y*floatToInt);
		inputPoints.push_back(VoronoiPoint(xa,ya));
		obstacles_edges.push_back(Segment(xa, ya, xb, yb));
	}
*/
	printf("hash %i\n",obstaclesVertexes.size());


	//throw std::logic_error( "STOP" );
	
	
	//add borders
	for(int i=0;i<3;i++)
	{
		xa=(int)((borders[i].x*floatToInt)>0?(borders[i].x*floatToInt):0);
		ya=(int)((borders[i].y*floatToInt)>0?(borders[i].y*floatToInt):0);
		if(ya<floatToInt)
			longId=(xa*floatToInt)+ya;
		else
			longId=(xa*floatToInt*10)+ya;
		p=VoronoiPoint(xa,ya);
		obstaclesVertexes[longId]=p;
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
	if(ya<floatToInt)
		longId=(xa*floatToInt)+ya;
	else
		longId=(xa*floatToInt*10)+ya;
	p=VoronoiPoint(xa,ya);
	obstaclesVertexes[longId]=p;
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
	Voronoi(inputPoints, obstacles_edges, obstaclesVertexes, &voronoiPaths);
		


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
	
	//throw std::logic_error( "STOP" );


/////////////////////////////////////////////////////////////////////////////////////////////////////////Tutto ok fino a qui //////////////////////////////////////////

	std::vector <double> angles;
	double theta_in=0, theta_out=0; //the first theta_out (to assign in place of 0) is the robot angle at the starting point.	
	angles.push_back(theta_out); 
	for(int i=1;i<rightPath.size()-1;i++)
	{
		angle_calculator(rightPath[i-1], rightPath[i], rightPath[i+1], theta_in, theta_out);
		angles.push_back(theta_in);
		angles.push_back(theta_out);
		printf("angle in %i: %f\n",i,theta_in);
		printf("angle out %i: %f\n",i,theta_out);
	}	
	angles.push_back(0); //this is the last angle for entering the gate in the right way. only a theta_in

	//throw std::logic_error( "STOP" );

	for(int i=0;i<angles.size();i++)
	{
		if(angles[i]>M_PI)
			angles[i]=-((2*M_PI)-angles[i]);
		printf("angles for dubins: %f\n", angles[i]);
	}

	//Calculate dubins curves on the path initial_robot - gate	
	int pidx;
	D curve;
	float s,pt_x,pt_y,pt_theta,kappa;
	int j=0;
	
	for(int i=0; i<rightPath.size()-1; i++)
	//int i=0;
	{
		printf("in loop dubins %i: \n",i);
		printf(" (%i, %i) - (%i, %i)\n",(rightPath[i].a),(rightPath[i].b),(rightPath[i+1].a),(rightPath[i+1].b));
		Dubins(rightPath[i].a,rightPath[i].b,rightPath[i+1].a,rightPath[i+1].b, angles[j], angles[j+1], &curve, &pidx); //add initial theta in and theta out.
		
		sample(curve.a1, path);
		sample(curve.a2, path);
		sample(curve.a3, path);

		j+=2;
	}
	
	//throw std::logic_error( "STOP" );
}

void angle_calculator(VoronoiPoint prev_p, VoronoiPoint point, VoronoiPoint next_p, double &theta_in,  double &theta_out)
{
	//compute some segments' lenght
	double prev_next = sqrt(pow((prev_p.a - next_p.a),2) + pow((prev_p.b - next_p.b),2));
	double prev_point = sqrt(pow((prev_p.a - point.a),2) + pow((prev_p.b - point.b),2));
	double point_next = sqrt(pow((point.a - next_p.a),2) + pow((point.b - next_p.b),2));
	double point_abscissa = sqrt(pow((point.a - (point.a+1)),2) + pow((point.b - point.b),2));
	double next_abscissa = sqrt(pow((next_p.a - (point.a+1)),2) + pow((next_p.b - point.b),2));
	//alpha is the angle between the segment of (prev_p, point) and (point, next_p)
	double alpha = acos((pow(prev_point,2) + pow(point_next,2) - pow(prev_next,2)) / (2 * prev_point * point_next));
	//beta is the complementary of alpha
	double beta = M_PI-alpha;
	//gamma is the angle between the segment (point, next_p) and abscissa
	double gamma = acos((pow(point_abscissa,2) + pow(point_next,2) - pow(next_abscissa,2)) / (2 * point_abscissa * point_next));
	//delta is the normalized angle wrt abscissa
	double delta=0;
	
	//if is above the pararrel of the abscissa with y=y_of_the_point 
	if(((point.a+1)-point.a)*(next_p.b-point.b)-(point.b-point.b)*(next_p.a-point.a))
	{
		if(gamma<=(M_PI/2) && gamma>=0)
			delta = gamma;
		else //gamma>(M_PI/2) && gamma<=M_PI
			delta = gamma - beta;
		theta_in=M_PI+gamma;
	}
	else //if is under the pararrel of the abscissa with y=y_of_the_point 
	{	
		if(gamma<=(M_PI/2) && gamma>=0) //check if acos is clockwise or not!!!!!!!
			delta = (M_PI*2)-gamma;
		else //gamma>(M_PI/2) && gamma<=M_PI
			delta = (3/2)*beta + alpha;
		theta_in=gamma+alpha;	
	}	

	//I assume as theta_in i take the prev_theta_out to have more uniformity in the movements 

	theta_out=delta;	
}

void sample(C arc, Path& path)
{
	double s,pt_x,pt_y,pt_theta,kappa, x, y, th;
	
	pt_x = arc.x0;
	pt_y = arc.y0;
	pt_theta = arc.th0;
	kappa = arc.k;

	for(int i=0;i<9;i++)	
	{
		s = (arc.L/8)*i;
		circline(s, pt_x, pt_y, pt_theta, kappa, &x, &y, &th);
		path.points.emplace_back(s/1000.0,x/1000.0,y/1000.0,th,kappa);
	}	

	//path.points.emplace_back(s/1000.0,ptf_x/1000.0,ptf_y/1000.0,ptf_theta,kappa);
}















  

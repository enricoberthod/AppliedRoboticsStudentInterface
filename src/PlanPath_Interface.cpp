#include "PlanPath_Interface.h"

#include <vector> 
#include <string>
#include <iostream>
#include <cmath>
#include <unordered_map>
#include "Voronoi.h"
#include "PathFinder.h"
#include "Dubins.h"

void angle_calculator(VoronoiPoint, VoronoiPoint, VoronoiPoint, double &,  double &);

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

/*
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
*/
	//create an unordered_map of the obstacles' vertexes

	int xa,xb,ya,yb,longId,vertexNumber;
	VoronoiPoint p;
	std::unordered_map<int,VoronoiPoint> obstaclesVertexes;

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
		
		pt_x = curve.a1.xf;
		pt_y = curve.a1.yf;
		pt_theta = curve.a1.thf;
		kappa = curve.a1.k;
		s = curve.a1.L;
		printf(" (%f, %f) - %f, %f, %f\n",pt_x/1000.0,pt_y/1000.0,pt_theta,kappa,s/1000.0);
		path.points.emplace_back(s/1000.0,pt_x/1000.0,pt_y/1000.0,pt_theta,kappa);
		
		pt_x = curve.a2.xf;
		pt_y = curve.a2.yf;
		pt_theta = curve.a2.thf;
		kappa = curve.a2.k;
		s = curve.a2.L;
		printf(" (%f, %f) - %f, %f, %f\n",pt_x/1000.0,pt_y/1000.0,pt_theta,kappa,s/1000.0);
		path.points.emplace_back(s/1000.0,pt_x/1000.0,pt_y/1000.0,pt_theta,kappa);
		
		pt_x = curve.a3.xf;
		pt_y = curve.a3.yf;
		pt_theta = curve.a3.thf;
		kappa = curve.a3.k;
		s = curve.a3.L;
		printf(" (%f, %f) - %f, %f, %f\n",pt_x/1000.0,pt_y/1000.0,pt_theta,kappa,s/1000.0);
		path.points.emplace_back(s/1000.0,pt_x/1000.0,pt_y/1000.0,pt_theta,kappa); 

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

void sample(D curve, int pidx, Path& path)
{
	float s,pt0_x,pt0_y,pt0_theta,ptf_x,ptf_y,ptf_theta,kappa;
	
	pt0_x = curve.a1.x0;
	pt0_y = curve.a1.y0;
	pt0_theta = curve.a1.th0;
	ptf_x = curve.a1.xf;
	ptf_y = curve.a1.yf;
	ptf_theta = curve.a1.thf;
	kappa = curve.a1.k;
	s = curve.a1.L;

	path.points.emplace_back(s/1000.0,ptf_x/1000.0,ptf_y/1000.0,ptf_theta,kappa);
}















  

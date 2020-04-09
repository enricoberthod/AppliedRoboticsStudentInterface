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
std::vector<float> IDP(float, std::vector<VoronoiPoint> &, Path &);
void function_L(int, float, std::vector<VoronoiPoint> &, std::vector<float> &, Path &);

//using namespace std;

void plan_Path123(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, 
				const Polygon& gate, const float x, const float y, const float theta, Path& path, const std::string& config_folder)
{
	
	//TEST PER DUBLINS
	/*
	int pidxxx;   
	D curvexx;
  	double startxx[3] = {100.0, 100.0, 0.0};
  	double endxx[3] = {300.0, 600.0, 0.0};
  	Dubins(startxx[0], startxx[1], endxx[0], endxx[1], startxx[2], endxx[2], &curvexx, &pidxxx);
	if(curvexx.a1.L!=0)
		sample(curvexx.a1, path);
	if(curvexx.a2.L!=0)
		sample(curvexx.a2, path);
	if(curvexx.a3.L!=0)
		sample(curvexx.a3, path);

	for(int i = 0; i < path.size(); i++) {
		std::cout << path.points.at(i).s << " " << path.points.at(i).x << " " << path.points.at(i).y << " " << path.points.at(i).theta << " " << path.points.at(i).kappa << std::endl;
	}
  //circline(0, pt_x, pt_y, pt_theta, kappa, &x, &y, &th);
  //path.points.emplace_back(0, start[0]/1000.0, start[1]/1000.0, start[2], 0);
  //path.points.emplace_back(0, end[0]/1000.0, end[1]/1000.0, end[2], 0);
	//path
  return;
*/
	
	//--lettura file parametri
	
	std::string file = config_folder+"/param.xml";
	cv::FileStorage param(file, cv::FileStorage::READ);
	
	int mission_type = (int)param["mission_type"];
	std::cout << "Mission -> " << mission_type << std::endl;
	//aggiungi tutti i dati che ti servono
	
	//--
	
	
	const int floatToInt=1000;
	//path.points.emplace_back(1,2,3,1,2);

	
	
	
	
	
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
	std::vector<VoronoiPoint> piecePath;

	printf("edgesAfterVoronoi: %i\n", voronoiPaths.resultEdges.size());
	printf("pointsAfterVoronoi: %i\n", voronoiPaths.resultPoints.size());
	
	for(int j=0;j<voronoiPaths.resultPoints.size();j++)
	{
		printf("x: %i",voronoiPaths.resultPoints[j].a);
		printf(", %i\n",voronoiPaths.resultPoints[j].b);
	}

	//throw std::logic_error( "STOP" );
	
	//VoronoiPoint start = VoronoiPoint((int)(0.2*floatToInt),(int)(0.1*floatToInt));
	float x_tot=0,y_tot=0;
	
	for(int i=0;i<gate.size();i++)
	{
		x_tot+=gate[i].x;
		y_tot+=gate[i].y;
	}

	//Robot center
	VoronoiPoint robot_pos = VoronoiPoint((int)(x*floatToInt),(int)(y*floatToInt));
	VoronoiPoint gate_pos = VoronoiPoint((int)((x_tot/4)*floatToInt),(int)((y_tot/4)*floatToInt)); 
	VoronoiPoint start = robot_pos;
	VoronoiPoint end = gate_pos;
	bool find = false;

	switch(mission_type) //0 for robot-gate //1 for robot-victims_in_right_order-gate //2 for robot-victims-gate in min time
	{
	case 0: //Direct path from robot initial position to gate position 		
		PathFinder(start, end, &voronoiPaths, &rightPath, false); 
	break;
	case 1: //Path from robot to victim 1, victim 1 to victim 2, victim n to gate position (Mission 1)
		
		//find shortest path from robot to 1, 1 to 2, 2 to n, n to gate  
		for(int i=1; i<=victim_list.size(); i++)
		{
			//search for the next victim 
			for(int j=0; j<victim_list.size() && !find; j++)
			{
				if(victim_list[j].first==i)
				{
					find=true;
					//set the end path to the next victim
					end=VoronoiPoint((int)(victim_list[j].second[0].x*floatToInt),(int)(victim_list[j].second[0].y*floatToInt));
					//find the shortest path
					PathFinder(start, end, &voronoiPaths, &piecePath, false); 
					//connect the piece of the path (victim to next victim) to the total one
					rightPath.insert(rightPath.end(),piecePath.begin(),rightPath.end());
					//the victim became the next starting point
					start=end;
				}
			}
			find=false;
		}
		//last call for connect the last victims with the gate
		PathFinder(start, gate_pos, &voronoiPaths, &piecePath, false);
		rightPath.insert(rightPath.end(),piecePath.begin(),rightPath.end()); 
	break;
	case 2: //Path which minimize the time for exit from the arena (Mission 2)
		PathFinder(start, end, &voronoiPaths, &rightPath, true);
	break;
	default: throw std::logic_error("STOP_mission_type different from [0-2]"); break;
	}
	
	//throw std::logic_error( "STOP" );
	

////////////////////////////////////////////////////////////////////Tutto ok fino a qui //////////////////////////////////////////////////

	std::vector<VoronoiPoint> rightPathNew;
	VoronoiPoint p1 = VoronoiPoint(rightPath[0].a,rightPath[0].b);
	rightPathNew.emplace_back(p1);
	int x1,y1,x2,y2;
	for(int i=0;i<rightPath.size()-1;i++)
	{
		x1=rightPath[i].a;
		y1=rightPath[i].b;
		x2=rightPath[i+1].a;
		y2=rightPath[i+1].b;		
	
		if(sqrt(pow((x1-x2),2)+pow((y1-y2),2))>100)
		{
			VoronoiPoint p = VoronoiPoint(x2,y2);
			rightPathNew.emplace_back(p);
		}
	}
	printf("-------Path %i => %i: ",rightPath.size(),rightPathNew.size());



	std::vector<float> prova;
	prova=IDP(theta, rightPathNew, path); //function to add the sample points to the simulator path

	//printf("Angoli!! %i\n", prova.size());
	//printf("path!! %i\n", path.points.size());
	
	for(int i=0; i<prova.size(); i++)
		printf(" %f: ",prova[i]);
	
	//throw std::logic_error( "STOP" ); 
}

float theta[8] = {0, M_PI/4, M_PI/2, (3*M_PI)/4, M_PI, -(M_PI/4), -(M_PI/2), -((3*M_PI)/4)};// (5*M_PI)/4, (3*M_PI)/2, (7*M_PI)/4};

std::vector<float> IDP(float angolo_start_robot, std::vector<VoronoiPoint> &rightPath, Path &path){
	
	std::vector<float> angoli;
	angoli.resize(rightPath.size());
	printf("Angoli!! %i\n",angoli.size());
	angoli[0]=angolo_start_robot;
	function_L(0,angoli.at(0), rightPath, angoli, path);
	
	return angoli;
}

int pidx;   
D curve;
float angle;
float residual_s=0;


void function_L(int j, float theta_j, std::vector<VoronoiPoint> &rightPath, std::vector<float> &angoli, Path &path){
	float min_length = 999999999.0;	
	int best_pidx;
	D best_curve;	
	printf("--------angolo: %f\n",theta_j);
	for(int i=0; i<8; i++) {
		//Dubins function (the given matlab code)  
		Dubins(rightPath[j].a,rightPath[j].b,rightPath[j+1].a,rightPath[j+1].b, theta_j, theta[i], &curve, &pidx); 
		if(curve.L < min_length) {
			min_length = curve.L;
			angoli[j+1] = theta[i];
		}
	}
	Dubins(rightPath[j].a,rightPath[j].b,rightPath[j+1].a,rightPath[j+1].b, theta_j, angoli[j+1], &best_curve, &best_pidx); 
	printf("kappa: %f, %f, %f \n", best_curve.a1.k,best_curve.a2.k,best_curve.a3.k);
	if(best_curve.a1.L!=0)//if the first arc is not 0 length
		sample(best_curve.a1, path);
	if(best_curve.a2.L!=0)//if the second arc is not 0 length
		sample(best_curve.a2, path);
	if(best_curve.a3.L!=0)//if the third arc is not 0 length
		sample(best_curve.a3, path);
	
	if(j < (rightPath.size()-2)){			//controllare estremi j +/- 1
		//angle=(angoli.at(j+1)+M_PI)<(2*M_PI)?(angoli.at(j+1)+M_PI):(angoli.at(j+1)-M_PI);
		function_L(j+1, angoli.at(j+1), rightPath, angoli, path);
	}
	else //not our problem now 
	{
		//add the gate point
		
		//path.points.emplace_back(s_tot+(arc.L/1000.0),x/1000.0,y/1000.0,th,kappa);
	}
}

void sample(C arc, Path& path)
{
	double pt_x,pt_y,pt_theta,kappa, x, y, th, s_tot=0;
	int step;
	//printf("arckappa: %f, %f, %f \n", arc.k, arc.L, arc.th0);
	pt_x = arc.x0;
	pt_y = arc.y0;
	pt_theta = arc.th0;
	//printf("th0 %f\n",arc.th0);
	//printf("thf %f\n",arc.thf);
	kappa = arc.k;
	if(path.points.size()==0)//the first point is the robot position 
		path.points.emplace_back(0, pt_x/1000.0, pt_y/1000.0, pt_theta, (kappa*1000));
	else
	{	
		//?? residual_s non lo usiamo ??
		
		step=10; //auxiliar variable for sampling step by step each 0.01
		//if(kappa==10 && arc.L<s)
		//	path.points.emplace_back(s_tot+arc.L,pt_x/1000.0,pt_y/1000.0,pt_theta,kappa);//add to the simulator path a new point
		//else
		
			s_tot= path.points.back().s; //(residual_s/1000.0); //start from the last point in simulator path + the new step
			while(step<arc.L)
			{		
					
				circline(step, pt_x, pt_y, pt_theta, kappa, &x, &y, &th); //use the function defined in Dubins (the given matlab code) to compute x,y,theta
				//printf("arckappa11: %f \n", kappa);
				//printf("th in %f\n",th);
				//s_tot=s_tot+(step/1000.0);
				path.points.emplace_back(s_tot+(step/1000.0),x/1000.0,y/1000.0,th,(kappa*1000));//add to the simulator path a new point
				step=step+10;
			}	
			circline(arc.L, pt_x, pt_y, pt_theta, kappa, &x, &y, &th); //use the function defined in Dubins (the given matlab code) to compute x,y,theta
			path.points.emplace_back(s_tot+((arc.L)/1000.0),x/1000.0,y/1000.0,th,(kappa*1000));//add to the simulator path a new point
			//printf("th out %f\n",th);
			
	}	

	//residual_s=((arc.L)-(s-10)); //calcolo il residual tra s e il nodo successivo (useles if we use s_tot=...+0.01)

	//printf("S %i", s);
	//printf(" arc.L %f", arc.L);
	//printf(" residual %f\n", residual_s);
		
}















  

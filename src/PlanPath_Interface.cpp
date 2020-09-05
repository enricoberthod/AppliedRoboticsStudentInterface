#include "PlanPath_Interface.h"

float theta[8] = {0, M_PI/4, M_PI/2, (3*M_PI)/4, M_PI, -(M_PI/4), -(M_PI/2), -((3*M_PI)/4)};		// possible angles for Dubins
int i_gate;																							// entry angle for the gate
std::vector<std::vector<cv::Point>> contours;														// list of the vertices of all obstacles after clipper
std::vector<cv::Point> bordi;																		// list of the verteces of the edge of the map
std::vector<cv::Point> arrivo;																		// list of the verteces of the gate
const int floatToInt=1000;																			// factor to convert m in mm
const int offset = 100;																				// offset of obstacle's expansion with clipper 
int pidx_1;																							// TODO
int pidx_2;  																						// 
D curve_1;																							// line drawn by Dubins
D curve_2;																							// line drawn by Dubins
std::vector<std::vector<float>> angoli_scartati;													// discarded angles
std::vector<int> punti_inseriti;																	// keep track of the number of insertions in path
bool result;																						// variable become false if doesn't exist a path

/* function plan_Path_2: call pathFinder() and sample the path checking collisions
   -parameters:
   	borders: struct with the points of the edges of the map
	obstacle_list: vector that contains all obstacles
	victim_list: vector which contains the victims
	gate: struct with the points of the gate
	x: x coordinate of the robot at the beginning
	y: y coordinate of the robot at the beginning
	theta: angle of the robot at the beginning
	path: struct that will contain the path to follow
	config_folder: path to reach the param.xml config file
    -return: true if there is a path from the starting point and the gate, false otherwise 
*/
bool plan_Path_2(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, 
				const Polygon& gate, const float x, const float y, const float theta, Path& path, const std::string& config_folder) {	
	
	// read parameter from param.xml	
	std::string file = config_folder+"/param.xml";
	cv::FileStorage param(file, cv::FileStorage::READ);
	
	int mission_type = (int)param["mission_type"];						// number of the mission
	std::cout << "Mission -> " << mission_type << std::endl;		
		
	VoronoiResults voronoiPaths;										// data structure for results


	float b_x_max = 0.0;												// biggest x value of borders
	float b_y_max = 0.0;												// biggest y value of borders
	for(int i = 0; i<borders.size(); i++) {
		b_x_max = std::max(b_x_max, borders[i].x);
		b_y_max = std::max(b_y_max, borders[i].y);
	}
	b_x_max = (b_x_max * floatToInt) - margine;
	b_y_max = (b_y_max * floatToInt) - margine;

	// CLIPPER library to expand obstacles			TODO (clippper non completa)
	ClipperLib::Paths subj(obstacle_list.size());
	ClipperLib::Paths solution(obstacle_list.size());

	for(int i=0;i<obstacle_list.size();i++)
	{
		for(int j=0;j<obstacle_list[i].size();j++)						// fill subj variable
		{
			subj[i] << ClipperLib::IntPoint(obstacle_list[i][j].x*floatToInt,obstacle_list[i][j].y*floatToInt);
		}
	}	
	
	ClipperLib::ClipperOffset co;
	co.AddPaths(subj, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
	co.Execute(solution, offset);						 				// calculate the final expansion of obstacles

	std::vector<VoronoiPoint> inputPoints;								// TODO
	std::vector<Segment> obstacles_edges;								// 

	int xa,xb,ya,yb,longId,vertexNumber;
	VoronoiPoint p;
	std::unordered_map<int,VoronoiPoint> obstaclesVertexes;				// create an unordered_map of the obstacles' vertexes
	// fill structurs with vertices and edge of obstacles after the expansion
	for(int i=0;i<solution.size();i++)
	{
		vertexNumber=solution[i].size();
		for(int j=0;j<vertexNumber-1;j++)
		{
			xa = (int)(solution[i][j].X);
			ya = (int)(solution[i][j].Y);
			xb = (int)(solution[i][j].X);
			yb = (int)(solution[i][j].Y);
			longId = encoder(xa, ya);
			p=VoronoiPoint(xa,ya);
			obstaclesVertexes[longId]=p;
			inputPoints.push_back(VoronoiPoint(xa,ya));
			obstacles_edges.push_back(Segment(xa, ya, xb, yb));
		}
		// close the polygon with the last edge
		xa = (int)(solution[i][vertexNumber-1].X);
		ya = (int)(solution[i][vertexNumber-1].Y);
		longId = encoder(xa, ya);
		p=VoronoiPoint(xa,ya);
		obstaclesVertexes[longId]=p;
		xb = (int)(solution[i][0].X);
		yb = (int)(solution[i][0].Y);
		inputPoints.push_back(VoronoiPoint(xa,ya));
		obstacles_edges.push_back(Segment(xa, ya, xb, yb));
	}

	// fill the structure of the list of the vertices of all obstacles after clipper
	for (int i = 0; i < solution.size(); i++) {
		std::vector<cv::Point> cc;
		for (int j = 0; j < solution[i].size(); j++) {
			xa = (int)(solution[i][j].X);
			ya = (int)(solution[i][j].Y);
			cc.push_back(cv::Point(xa,ya));
		}
		contours.push_back(cc);
	}	
	
	// add borders to structures
	for(int i=0;i<3;i++)
	{
		xa=(int)((borders[i].x*floatToInt)>0?(borders[i].x*floatToInt):0);
		ya=(int)((borders[i].y*floatToInt)>0?(borders[i].y*floatToInt):0);
		if(xa < 500) {
			xa = xa + margine;								// use margine in order to expand borders to avoid that robot is too near
		}
		else {
			xa = xa - margine;
		}
		if(ya < 500) {
			ya = ya + margine;
		}
		else {
			ya = ya - margine;
		}
		longId=encoder(xa,ya);
		p=VoronoiPoint(xa,ya);
		obstaclesVertexes[longId]=p;
		bordi.push_back(cv::Point(xa, ya));
		xb=(int)((borders[i+1].x*floatToInt)>0?(borders[i+1].x*floatToInt):0);
		yb=(int)((borders[i+1].y*floatToInt)>0?(borders[i+1].y*floatToInt):0);
		if(xb < 500) {
			xb = xb + margine;
		}
		else {
			xb = xb - margine;
		}
		if(yb < 500) {
			yb = yb + margine;
		}
		else {
			yb = yb - margine;
		}
		inputPoints.push_back(VoronoiPoint(xa,ya));
		obstacles_edges.push_back(Segment(xa, ya, xb, yb));
	}
	// close the borders
	xa=(int)((borders[3].x*floatToInt)>0?(borders[3].x*floatToInt):0);
	ya=(int)((borders[3].y*floatToInt)>0?(borders[3].y*floatToInt):0);
	if(xa < 500) {
		xa = xa + margine;
	}
	else {
		xa = xa - margine;
	}
	if(ya < 500) {
		ya = ya + margine;
	}
	else {
		ya = ya - margine;
	}
	longId=encoder(xa,ya);
	p=VoronoiPoint(xa,ya);
	obstaclesVertexes[longId]=p;
	bordi.push_back(cv::Point(xa, ya));
	xb=(int)((borders[0].x*floatToInt)>0?(borders[0].x*floatToInt):0);
	yb=(int)((borders[0].y*floatToInt)>0?(borders[0].y*floatToInt):0);	
	if(xb < 500) {
		xb = xb + margine;
	}
	else {
		xb = xb - margine;
	}
	if(yb < 500) {
		yb = yb + margine;
	}
	else {
		yb = yb - margine;
	}
	inputPoints.push_back(VoronoiPoint(xa,ya));
	obstacles_edges.push_back(Segment(xa, ya, xb, yb));	

	// Creates the Voronoi map
	Voronoi(inputPoints, obstacles_edges, obstaclesVertexes, contours, b_x_max, b_y_max, &voronoiPaths, config_folder);

	// data structure for results
	std::vector<VoronoiPoint> rightPath;
	std::vector<VoronoiPoint> piecePath;
	
	float x_tot=0,y_tot=0;
	for(int i=0;i<gate.size();i++)
	{
		arrivo.push_back(cv::Point((int)(gate[i].x * floatToInt), (int)(gate[i].y * floatToInt)));			// fill vector arrivo with gate's points
		x_tot+=gate[i].x;
		y_tot+=gate[i].y;
	}

	//Robot center
	VoronoiPoint robot_pos = VoronoiPoint((int)(x*floatToInt),(int)(y*floatToInt));							// starting point of the robot
	VoronoiPoint gate_pos = VoronoiPoint((int)((x_tot/4)*floatToInt),(int)((y_tot/4)*floatToInt)); 			// central point of the gate
	VoronoiPoint start = robot_pos;																			// start point to calculate the path
	VoronoiPoint end = gate_pos;																			// end point to calculate the path
	bool find = false;
	bool firstTime = true;

	switch(mission_type) // 0 for robot-gate		1 for robot-victims_in_right_order-gate			2 for robot-victims-gate in min time
	{
	case 0: 
		result = PathFinder(start, true, end, true, &voronoiPaths, &rightPath, 0, contours, NULL, config_folder);	// calculate the path
	break;
	case 1:
		//find shortest path from robot to 1, 1 to 2, 2 to n, n to gate  (Mission 1)
		for(int i=1; i<=(victim_list.size()+5); i++)
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
					piecePath.clear();
					result = PathFinder(start, firstTime, end, true, &voronoiPaths, &piecePath, 10, contours, NULL, config_folder); 
					//connect the piece of the path (victim to next victim) to the total one
					rightPath.insert(rightPath.end(),piecePath.begin(),piecePath.end());
					//the victim became the next starting point
					start=end;
					firstTime=false;				// avoid to insert two times the same point
				}
			}
			find=false;
		}
		//last call for connect the last victims with the gate
		piecePath.clear();
		if(victim_list.size()>0)
			result = PathFinder(start, false, gate_pos, true, &voronoiPaths, &piecePath, 0, contours, NULL, config_folder);
		else
			result = PathFinder(start, true, gate_pos, true, &voronoiPaths, &piecePath, 0, contours, NULL, config_folder);	// if there isn't victims
		rightPath.insert(rightPath.end(),piecePath.begin(),piecePath.end()); 										// complete the path
	break;
	case 2: //Path which minimize the time for exit from the arena (Mission 2)
		result = PathFinder(start, true, end, true, &voronoiPaths, &rightPath, 10, contours, &victim_list, config_folder);
	break;
	default: throw std::logic_error("STOP_mission_type different from [0-2]"); break;
	}	

	// if PathFinder didn't find a path
	if(!result) {
		return result;
	}

	std::vector<VoronoiPoint> rightPathNew;									// optimized path
	VoronoiPoint p1 = VoronoiPoint(rightPath[0].a,rightPath[0].b);
	rightPathNew.emplace_back(p1);
	int x1,y1,x2,y2;
	// delete some near points from the path in order do semplify it
	for(int i=0;i<rightPath.size()-2;i++)
	{
		x1=rightPathNew[rightPathNew.size()-1].a;
		y1=rightPathNew[rightPathNew.size()-1].b;
		x2=rightPath[i+1].a;
		y2=rightPath[i+1].b;	

		bool is_victim = false;	
	
		for(int j=0; j<victim_list.size(); j++) {
			if(x2 == (int)(victim_list[j].second[0].x*floatToInt)) {
				if(y2 == (int)(victim_list[j].second[0].y*floatToInt)) {
					is_victim = true;
				}
			}
		}
		if((sqrt(pow((x1-x2),2)+pow((y1-y2),2))>250) || is_victim)
		{
			VoronoiPoint p = VoronoiPoint(x2,y2);
			rightPathNew.emplace_back(p);
		}
	}
	VoronoiPoint p_end = VoronoiPoint(rightPath[rightPath.size()-1].a,rightPath[rightPath.size()-1].b);
	rightPathNew.insert(rightPathNew.end(), p_end);

	// add a point on the edge of the gate and decide the entry angle in the gate
	VoronoiPoint border_gate_pos;
	int dy = std::fabs(gate_pos.b-(int)(gate[0].y*floatToInt));
	int dx = std::fabs(gate_pos.a-(int)(gate[0].x*floatToInt));
	if(dx > dy){
		if(gate_pos.b < 500) {
			border_gate_pos = VoronoiPoint((int)(gate_pos.a),(int)(gate_pos.b+dy));
			i_gate = 6;
		}
		else {
			border_gate_pos = VoronoiPoint((int)(gate_pos.a),(int)(gate_pos.b-dy));
			i_gate = 2;
		}
	}
	else {
		if(gate_pos.a < 500) {
			border_gate_pos = VoronoiPoint((int)(gate_pos.a+dx),(int)(gate_pos.b));
			i_gate = 4;
		}
		else {
			border_gate_pos = VoronoiPoint((int)(gate_pos.a-dx),(int)(gate_pos.b));
			i_gate = 0;
		}
	}
	rightPathNew.insert(rightPathNew.end()-1, border_gate_pos);			// add the point to the path

	result = IDP(theta, rightPathNew, path); 									// function to add the sample points to the simulator path
	std::cout << "--- result --- " << result << std::endl;
	std::cout << "--- START PATH --- " << std::endl;
	for(int i=0; i<rightPathNew.size(); i++)
		std::cout << "--> " << (i+1) << ": (" << rightPathNew[i].a << ", " << rightPathNew[i].b << ")" << std::endl;
	std::cout << "--- END PATH --- " << std::endl;
	return result;	
}

/* function IDP: wrapper function for the first call of function_L_doppio() 
   -parameters:
   	angolo_start_robot: angle of the robot at the beginning
	rightPath: vector of path's points that the robot will reach
	path: struct that will contain the path to follow
    -return: true if a path is possible, false otherwise
*/
bool IDP(float angolo_start_robot, std::vector<VoronoiPoint> &rightPath, Path &path){
	std::vector<float> angoli;
	angoli.resize(rightPath.size());
	angoli[0]=angolo_start_robot;
	bool r = function_L_doppio(0,angoli.at(0), rightPath, angoli, path, false);			// first call
	return r;
}

/* function function_L_doppio: function that canclulate the entry angles for all points of the rightPath, call Dubins() and sample()  
							   In case of collision change angle and can backtrack to change previous angles
   -parameters:
   	j: number of the point of rightpath that is considered
	theta_j: angle of the point j
	rightPath: vector of path's points that the robot will reach
	angoli: vector with all angles
	path: struct that will contain the path to follow
	finito: true if the program already compute the angle for this point, false otherwise
    -return: true if a path is possible without collisions, false otherwise
*/
bool function_L_doppio(int j, float theta_j, std::vector<VoronoiPoint> &rightPath, std::vector<float> &angoli, Path &path, bool finito){
	float min_length = 999999999.0;	
	int best_pidx;
	D best_curve;	
	bool collision;
	bool salta = false;
	bool fase_1_finita = false;
	bool init = false;
	bool r;

	// when backtrack, discard the previous angle and remove the inserted points in path
	if(finito) {
		angoli_scartati[j].emplace_back(angoli[j+1]);
		for (int e = 0; e < punti_inseriti[j]; e++) {
			path.points.pop_back();
		}
		punti_inseriti[j] = 0;
	}
	// repeat this part in case of collisions
	do {
		if(j==rightPath.size()-3) {				// last two points have a pre-decided angle
			angoli[j+1] = theta[i_gate];
			angoli[j+2] = theta[i_gate];
		}
		else {
			// search the best angle for the point j+1 to minimize the space in the part j -> j+1 -> j+2
			for(int i=0; i<8; i++) {
				salta = false;
				if(init || finito) {			// discard angles that aren't adapt
					for(int h = 0; h < angoli_scartati[j].size(); h++) {
						if(theta[i] == angoli_scartati[j][h]) {
							salta = true;
						}
					}
				}
				if(!salta) {
					// Dubins function to calculate the route that robot will follow
					Dubins(rightPath[j].a,rightPath[j].b,rightPath[j+1].a,rightPath[j+1].b, theta_j, theta[i], &curve_1, &pidx_1);					// first part j -> j+1
					if(j==rightPath.size()-4) {		// if the second point is the gate's edge
						Dubins(rightPath[j+1].a,rightPath[j+1].b,rightPath[j+2].a,rightPath[j+2].b, theta[i], theta[i_gate], &curve_2, &pidx_2);	// second part j+1 -> j+2
						if((curve_1.L + curve_2.L) < min_length) {
							min_length = curve_1.L + curve_2.L;
							angoli[j+1] = theta[i];
						}
					}
					else {
						for(int w=0; w<8; w++) {
							Dubins(rightPath[j+1].a,rightPath[j+1].b,rightPath[j+2].a,rightPath[j+2].b, theta[i], theta[w], &curve_2, &pidx_2);		// second part j+1 -> j+2
							if((curve_1.L + curve_2.L) < min_length) {
								min_length = curve_1.L + curve_2.L;
								angoli[j+1] = theta[i];
							}
						}
					}
				}
			}	
		}
		// call Dubins for j -> j+1 with the best angle 
		Dubins(rightPath[j].a,rightPath[j].b,rightPath[j+1].a,rightPath[j+1].b, theta_j, angoli[j+1], &best_curve, &best_pidx); 
		// insert the first data in the posiotion j for punti_inseriti and angoli_scartati
		if(finito == false && init == false) {
			punti_inseriti.emplace_back(0);
			std::vector<float> tmp;
			tmp.emplace_back(0.0);
			angoli_scartati.emplace_back(tmp);
		}
		if(best_curve.a1.L!=0)												// if the first arc is not 0 length
			collision = sample(best_curve.a1, path, punti_inseriti[j]);		// check collisions
		if(best_curve.a2.L!=0 && !collision)								// if the second arc is not 0 length
			collision = sample(best_curve.a2, path, punti_inseriti[j]);		// check collisions
		if(best_curve.a3.L!=0 && !collision)								// if the third arc is not 0 length
			collision = sample(best_curve.a3, path, punti_inseriti[j]);		// check collisions
		if(collision) {
			// remove points in the path in case of collision
			for (int e = 0; e < punti_inseriti[j]; e++) {
				path.points.pop_back();
			}
			punti_inseriti[j] = 0;
			// insert the first angle to discard
			if(!finito && !init) {
				std::vector<float> tmp;
				tmp.emplace_back(angoli[j+1]);
				angoli_scartati.at(j) = tmp; 
				init = true;
			}
			else {
				angoli_scartati[j].emplace_back(angoli[j+1]);				// insert an angle to discard
			}
			// if all angles are discarded and it's not the bininning -> backtrack step
			if (angoli_scartati[j].size() >= 8 && j>0) {
				std::cout << std::endl;
				fase_1_finita = true;
				r = function_L_doppio(j-1, angoli.at(j-1), rightPath, angoli, path, fase_1_finita);			// backtrack step
				if (!r) {
					return r;
				}
				theta_j = angoli.at(j);																		// new value for theta_j
				fase_1_finita = false;
				angoli_scartati[j].clear();
			}
			// if all angles are discarded and it's the bininning -> no path
			else if (angoli_scartati[j].size() >= 8 && j==0) {
				return false;
			}
			min_length = 999999999.0;	
		}
	} while(collision);
	
	if(j < (rightPath.size()-2) && !finito) {		
		return function_L_doppio(j+1, angoli.at(j+1), rightPath, angoli, path, fase_1_finita);				// call for the next point
	}
	else if(finito) {		// backtrack case, return the control to the right point
		return true;
	}
	else {
		return true;
	}
}

/* function sample: calculate little sample along the route checking the collision and fill the path variable
   -parameters:
   	arc: piece of the route calculated by Dubins
	path: struct that will contain the path to follow
	punti_inseriti: count the number of point added to the path
    -return: true if there is a collision, false if all is ok
*/
bool sample(C arc, Path& path, int &punti_inseriti) {
	double x, y, th, s_tot=0;
	int step;
	bool collision = false;
	double pt_x = arc.x0;
	double pt_y = arc.y0;
	double pt_theta = arc.th0;
	double kappa = arc.k;
	int inserts = 0;

	if(path.points.size()==0) { 		// the first point is the robot position 
		path.points.emplace_back(0, pt_x/1000.0, pt_y/1000.0, pt_theta, (kappa*1000));
		inserts++;
	}
	step=10; 							// auxiliar variable for sampling step by step each 0.01 m
	s_tot= path.points.back().s; 		// start from the last point in simulator path + the new step
	while(step<arc.L && !collision) {		
		circline(step, pt_x, pt_y, pt_theta, kappa, &x, &y, &th); 							// use the function defined in Dubins to compute x,y,theta
		collision = collision_detection(x, y);									// check collisions
		path.points.emplace_back(s_tot+(step/1000.0),x/1000.0,y/1000.0,th,(kappa*1000));	// add to the simulator path a new point
		inserts++;
		step=step+10;
	}	
	if(!collision) {
		circline(arc.L, pt_x, pt_y, pt_theta, kappa, &x, &y, &th); 							// use the function defined in Dubins to compute x,y,theta
		collision = collision_detection(x, y);												// check collisions
		path.points.emplace_back(s_tot+((arc.L)/1000.0),x/1000.0,y/1000.0,th,(kappa*1000));	// add to the simulator path a new point
		inserts++;
	}
	if(collision) {
		// remove added elements in case of collision
		for (int i = 0; i < inserts; i++) {
			path.points.pop_back();
		}
	}
	else {
		punti_inseriti = punti_inseriti + inserts;				// keep track of the number of insertions in path
	}
	return collision;
}

/* function collision_detection: detect collision with obstacles and with borders (if the path is not inside the gate)
   -parameters:
   	x: x coordinate of the point to check
	y: y coordinate of the point to check
    -return: true if there is a collision, false if all is ok
*/
bool collision_detection(double x, double y) {
	bool r = false;
	double res = cv::pointPolygonTest(bordi , cv::Point2f(x,y) , true);				// check where is the point respect borders
	double in_gate = cv::pointPolygonTest(arrivo , cv::Point2f(x,y) , true);		// check where is the point respect the gate
	// if robot is outside the border and is not in the gate
	if(res <= 0 && in_gate <= 0) {
		r = true;
	}
	// check if the point is outside every obstacle
	for (int i = 0; i < contours.size() && !r; i++) {
		res = cv::pointPolygonTest(contours[i] , cv::Point2f(x,y) , true);
		if(res > 0) {
			r = true;
		}
	}
	return r;
}

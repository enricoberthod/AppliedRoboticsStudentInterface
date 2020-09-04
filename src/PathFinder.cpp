#include "PathFinder.h"
#include "Dijkstra.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <stdlib.h> 
#include <limits.h> 
#include <stdio.h> 

void connect(int, VoronoiPoint, VoronoiResults*, bool, double, std::vector<GraphEdge>*, int, const std::vector<std::vector<cv::Point>>&);
bool shortestPath(VoronoiPoint, bool, int, VoronoiPoint, bool, int, VoronoiResults *, std::vector<VoronoiPoint> *);
bool edgeOnObstacle(VoronoiPoint, VoronoiPoint, const std::vector<std::vector<cv::Point>>&);
bool findCollision(double, double, const std::vector<std::vector<cv::Point>>&); 
void connector_netpoints(VoronoiPoint, int, int, int, int, double, VoronoiResults *, int, const std::vector<std::vector<cv::Point>>&);
void connector_singlepoint(VoronoiPoint, int, int, int, int, double, VoronoiResults *, const std::vector<std::vector<cv::Point>>&);

/* function PathFinder: function which computes the best path from the start point to the end point given the road map and the following parameters
   -parameters:
   	start: starting point of the path 
	startNew: boolean which indicates if the start point is already in the road map (false) or not (true)
	end: ending point of the path
	endNew: boolean which indicates if the end point is already in the road map (false) or not (true)
	voronoiPaths: the road map
	rightPath: the solution vector which contains the points to follow from start to end
	netRadius: indicates the distance from the end point to the cloud of points generated around it (m1 porpouses)
	obsContours: vector which contains the expanded obstacles points 
	victim_list: vector which contains the victims
	config_folder: path to reach the param.xml config file
    -return: true if a path is found, false otherwise
*/
bool PathFinder(VoronoiPoint start, bool startNew, VoronoiPoint end, bool endNew, VoronoiResults *voronoiPaths, std::vector<VoronoiPoint> *rightPath, int netRadius, const std::vector<std::vector<cv::Point>>& obsContours, const std::vector<std::pair<int,Polygon>> *victim_list, const std::string& config_folder)
{
	int offset; //offset indicates the initial offset in which a point (start or end or victim) can connect itself to the road map points
	int offset_2; //offset_2 indicates the initial offset in which a point (start or end or victim) can connect itself to the road map points (m1 porpouses)
	int max_offset; //max_offset indicates the max distance in which a point (start or end) can connect itself to the road map points
	int max_offset_vic; //max_offset_vic indicates the max distance in which a point (victim) can connect itself to the road map points
	int step; //step indicate sthe amount to add to offset each time until to reach the max_offset or max_offset_vic
	int nConnections; //nConnections indicate sthe max number of road map points that can be connected to the point (estart or end or victim)
	int victimGain; //victimGain indicates the gain in space (mm) the robot earns if it saves a victim (m2 porpuses) 
	float robotSpeed; //robotSpeed indicates the robot velocity
	int const netPointsNumber = 4; //netPointsNumber indicates the number of points which compose the cloud of points around a point (victim) (m1 porpouses)
	std::vector<VoronoiPoint> netVertex; //vector which contains the cloud of points around a point (victim) and point itself (m1 porpouses)
	bool result = true; //variable become false if doesn't exist a path

	//read parameter from param.xml	
	std::string file = config_folder+"/param.xml";
	cv::FileStorage param(file, cv::FileStorage::READ);
	
	//robotSpeed indicates the speds of the robot in m/s
	robotSpeed = (float)param["robotSpeed"];
	//convert m/s in mm/s
	robotSpeed*1000;

	//victimGain indicates the gain in space the robot earns if it saves a victim (m2 porpuses) 
	victimGain = (int)param["victimGain"];
	//computes the gain in space (mm) given the gain in seconds and the robot velocity
	victimGain = (int)(victimGain*robotSpeed)/2;
	
	//offset indicates the initial offset in which a point (start or end or victim) can connect itself to the road map points
	offset = (int)param["offset"];
	
	//step indicate sthe amount to add to offset each time until to reach the max_offset or max_offset_vic
	step = (int)param["step"];
	
	//max_offset indicates the max distance in which a point (start or end) can connect itself to the road map points
	max_offset = (int)param["max_offset"];

	//max_offset_vic indicates the max distance in which a point (victim) can connect itself to the road map points
	max_offset_vic = (int)param["max_offset_vic"];

	//nConnections indicate sthe max number of road map points that can be connected to the point (start or end or victim)
	nConnections = (int)param["nConnections"];

	//if start point is not alredy in the road map (true)	
	if(startNew)
	{
		//connect the start point to the road map 
		connector_singlepoint(start, offset, step, max_offset, nConnections, victimGain, voronoiPaths, obsContours);
	}
	
	//if end point is not alredy in the road map (true)
	if(endNew)
	{
		if(netRadius>0)
		{	
			//if no victim_list (mission 1)
			if(victim_list == NULL)	
				//connect the end point (gate or victim) to the road map
				connector_netpoints(end, offset, step, max_offset, nConnections, victimGain, voronoiPaths, netRadius, obsContours);
			//if victim list (mission 2)
			else
			{
				//connect the gate to the road map
				connector_singlepoint(end, offset, step, max_offset, nConnections, victimGain, voronoiPaths, obsContours);
				VoronoiPoint p;
				//connect each victim in victim list to the road map
				for(int j=0; j<victim_list->size(); j++)
				{
					p=VoronoiPoint((int)(victim_list->at(j).second[0].x*1000),(int)(victim_list->at(j).second[0].y*1000));
					connector_singlepoint(p, offset, step, max_offset_vic, nConnections, 0, voronoiPaths, obsContours);
				}			
			}
		}
		//if net radius == 0 
		else
		{
			//connect the gate to the road map
			connector_singlepoint(end, offset, step, max_offset, nConnections, victimGain, voronoiPaths, obsContours);
		}
		
	}

	//check if there is an obstacle between start and end, check if victim list is not null (mission 2) 
	if(edgeOnObstacle(start,end,obsContours) || victim_list != NULL)
	{
		if(netRadius==0)
		{
			//call the shortestPath function (mission 0), -2 and -1 because the start is the second-last element and the gate is the last (but arrays are from 0 to size-1) 
			result = shortestPath(start, startNew, -2, end, endNew, -1, voronoiPaths, rightPath);		
		}
		else
		{
			int posStart,posEnd;
			if(victim_list == NULL)
			{
				//the same reasoning as before but here there is the cloud of points around the victim point to consider
				posStart=(netPointsNumber+2)*(-1);
				posEnd=-1;
			}
			// victim_list not null (mission 2)
			else
			{
				//the same reasoning as before but here there are the victmins after the robot and the gate in the vector
				posStart=(2+victim_list->size())*(-1);
				posEnd=(1+victim_list->size())*(-1);
			}
			//call the shortestPath function
			result = shortestPath(start, startNew, posStart, end, endNew, posEnd, voronoiPaths, rightPath);
		}
	}
	//if not mission 2 or there are no obstacles between start point and end point
	else
	{
		//if the start point is not in the road map it is also not in the best path
		if(startNew)
		{
			//add directly the start point to the best path without call shortestPath function (the best path is a straight line)
			rightPath->push_back(start);
		}
		//if the end point is not in the road map it is also not in the best path
		if(endNew)
		{
			//add directly the end point to the best path without call shortestPath function (the best path is a straight line)
			rightPath->push_back(end);
		}
	}
	return result;
}

/* function shortestPath: function which practically computes the best path from the start point to the end point 
   -parameters:
   	start: starting point of the path 
	startNew: boolean which indicates if the start point is already in the road map (false) or not (true)
	idStart: the position of the start point in the ids array 
	end: ending point of the path
	endNew: boolean which indicates if the end point is already in the road map (false) or not (true)
	idEnd: the position of the end point in the ids array 
	voronoiPaths: the road map
	rightPath: the solution vector which contains the points to follow from start to end
    -return: true if a path is found, false otherwise
*/
bool shortestPath(VoronoiPoint start, bool startNew, int idStart, VoronoiPoint end, bool endNew, int idEnd, VoronoiResults *voronoiPaths, std::vector<VoronoiPoint> *rightPath)
{
	//V is the number of vertices of the graph
	int V = voronoiPaths->ids.size(); 
	int k=0;
	int idNode1,idNode2=0;
	//array of the solutions of Dijkstra
	int path[V];
	//array to map the encoded coordinates and the ids of the nodes
	int mapPosition[V];
	int encodedCoord,x,y;
	//create the graph using the function createGraph() (see Dijkstra.cpp)
	struct Graph* graph = createGraph(V); 
	
	//for each edge of the road map
	for(int i=0;i<voronoiPaths->resultEdges.size();i++)
	{		
		//save the coordinates of the first node (in edge A-B, take A)
		x=voronoiPaths->resultEdges[i].p0.a;
		y=voronoiPaths->resultEdges[i].p0.b;
		//encode the coordinates 
		encodedCoord=encoder(x,y);	
		
		//save the coordinates in the array, each encoded coordinates in position using the id of the first node (edge A-B, take A id) 
		mapPosition[voronoiPaths->resultEdges[i].idFirstNode]=encodedCoord;
		
		//create the edge in the graph used by Dijkstra algorithm using for the edge A-B the id of A, the id of B and the lenght of the segment A-B (see Dijkstra.cpp)
		addEdge(graph, voronoiPaths->resultEdges[i].idFirstNode, voronoiPaths->resultEdges[i].idSecondNode, voronoiPaths->resultEdges[i].length); 		
	}
	
	//call the Dijkstra algorithm using the graph generated before and the ids of the start point and end point (see Dijkstra.cpp)
	dijkstra(graph,voronoiPaths->ids.size()+idStart,voronoiPaths->ids.size()+idEnd,path); 
	
	//create the array for the solution of Dijkstra (start to end) (here there are only the ids)
	std::vector<int> startEndPath;
	//add the start point id to the solution
	startEndPath.push_back(voronoiPaths->ids.size()+idStart);
	//add the best path found using Dijkstra using the storePath function (see Dijkstra.cpp)
	storePath(path,voronoiPaths->ids.size()+idEnd,&startEndPath);

	//retirn false if there is no path from start point to end point
	if(startEndPath.size()<2){
		return false;
		// TODO throw std::logic_error("STOP_possible_path_not_found");
	}
	
	//if the start point is not in existing best path (chek due to mission 1)
	if(startNew) {	
		rightPath->push_back(start);
	}
	
	int x1,y1;
	//for each id in the path found using Dijkstra, except the start point and the end point
	for(int i=1;i<startEndPath.size()-1;i++)
	{
		//retrive x and y coordinates of the id in the solution path
		decoder(mapPosition[startEndPath[i]],x1,y1);		
		VoronoiPoint vertex = VoronoiPoint(x1,y1);
		rightPath->push_back(vertex);
	}
	if(endNew) {
		//if the end point is not in existing best path (chek due to mission 1)
		rightPath->push_back(end);
	}
	return true;
}



/* function connect: function which connect a point to the road map 
   -parameters:
   	offset: initial max distance where to find points of the road map (from this point to each road map point)
	pointP: the point to connect
	pointOK: boolean to check if we have to connect the point
	vGain: the gain in space that the robot earns if saves a victim
	pointEdges: the solution vector which contains the candidates edges from the point and the road map
	IDpos: the position of the point in the ids vector
	obsContours: vector which contains the expanded obstacles points 
    -return: no retun 
*/
void connect(int offset, VoronoiPoint pointP, VoronoiResults *voronoiPaths, bool pointOK, double vGain, std::vector<GraphEdge> *pointEdges, int IDpos, const std::vector<std::vector<cv::Point>>& obsContours)
{
	int x,y,id1;
	int prev_x,prev_y=-1;
	double len;
	int longId;
	int pointLongId;
	int pointId; 
	bool find;
	
	pointLongId=encoder(pointP.a, pointP.b);
	pointId=voronoiPaths->ids.size()+IDpos; 

	//for each edge in the road map 
	for(int i=0;i<voronoiPaths->resultEdges.size();i++)
	{
		//save the coordinates of the first node (in edge A-B, take A)
		x=voronoiPaths->resultEdges[i].p0.a;
		y=voronoiPaths->resultEdges[i].p0.b;
		
		if(pointOK)
		{
			//compute the length between the road map point and the point we want to connect 
			len=sqrt(pow((x-pointP.a),2)+pow((y-pointP.b),2));
			//add the gain in space that the robot earns if saves a victim
			len=len+vGain;
			
			//if the length is smaller than a threshold
			if(len<=((offset*1.0)+vGain))
			{
				//if the road map point is not the previous one
				if(x!=prev_x || y!=prev_y)
				{
					//if there is no obstacles between the road map point and the point we want to connect
					if(!edgeOnObstacle(pointP,VoronoiPoint(x,y),obsContours))
					{
						longId=encoder(x,y);
						id1=-1;
						//look for theposition of the id in the ids array 
						for(int j=0;j<voronoiPaths->ids.size() && id1==-1;j++)
							if(voronoiPaths->ids[j]==longId)
								id1=j;
						
						find=false;
						//check that the node is not already taken in this step
						for(int k=0;k<pointEdges->size() && !find;k++)
						{
							if(pointEdges->at(k).idFirstNode==id1)
								find=true;
						}
						//check that the node is not already taken in previous calls
						for(int j=(voronoiPaths->resultEdges.size()-1);j>=0 && !find ;j--)
						{
							if(voronoiPaths->resultEdges[j].idFirstNode==pointId && voronoiPaths->resultEdges[j].idSecondNode==id1)
								find=true;
						}
						//if not already taken 
						if(!find)
						{
							find=false;
							//add the edge from the road map point and the point we want to connect and viceversa
							GraphEdge e1(x,y,pointP.a,pointP.b,len,id1,pointId);
							pointEdges->push_back(e1);
							GraphEdge e2(pointP.a,pointP.b,x,y,len,pointId,id1);
							pointEdges->push_back(e2);
						}
					}
					prev_x=x;
					prev_y=y;
				}
			}
		}
	}
}	

/* function edgeOnObstacle: function which check if there is an obstacle between point A and B
   -parameters:
   	a: first point
	b: second point
	obsContours: vector which contains the expanded obstacles points 
    -return: true if there is an obstacle between a and b, false if there is not
*/
bool edgeOnObstacle(VoronoiPoint a, VoronoiPoint b, const std::vector<std::vector<cv::Point>>& obsContours)
{
	//vector of sample points (on the segment from A to B)
	std::vector<VoronoiPoint> samples;
	bool isCollision=false;
	
	//call the function to compute the samples
	sampleSegment2(a, b, samples);
	
	//for each sample between A and B look for a collision with an obstacle
	for(int i=0; i<samples.size() && isCollision==false;i++)
	{
		isCollision=findCollision(samples[i].a, samples[i].b, obsContours);
	}
	
	return isCollision;
}
	


/* function sampleSegment2: function which computes sample points on the segment AB
   -parameters:
   	a: first point
	b: second point
	samples: vector containing the solution, the points on the segment AB
    -return: none
*/
void sampleSegment2(VoronoiPoint a, VoronoiPoint b, std::vector<VoronoiPoint>& samples) {
	
	samples.push_back(a);
	float d = 5;
	float d_tot2 = (((a.a - b.a) * (a.a - b.a)) + ((a.b - b.b) * (a.b - b.b)));
	float d_tot = (std::sqrt(d_tot2));
	int x, x1, x2, y, y1, y2;
	
	//if the x coordinate of A is different from x coordinate of B
	if (a.a != b.a) {
		float m = ((float)(a.b - b.b) / (float)(a.a - b.a));
		float q = (a.b - (m * a.a));
		float par_a = ((m * m) + 1);
		float par_b = ((2 * m * q) - (2 * a.a) - (2 * a.b * m));
		float par_c;
		while (d < d_tot) { 
			par_c = (a.a * a.a) + (a.b * a.b) + (q * q) - (2 * a.b * q) - (d * d);
			x1 = (int)(- par_b + std::sqrt((par_b * par_b) - (4 * par_a * par_c))) / (2 * par_a);
			x2 = (int)(- par_b - std::sqrt((par_b * par_b) - (4 * par_a * par_c))) / (2 * par_a);
			y1 = (int)(m * x1) + q; 
			y2 = (int)(m * x2) + q;
			if((x1 > a.a && x1 < b.a) || (x1 > b.a && x1 < a.a)) {
				x = x1;
				y = y1;
			}
			else {
				x = x2;
				y = y2;
			}
			VoronoiPoint p = VoronoiPoint(x,y);
			samples.push_back(p);
			d = d + 5;
		}
	}
	//if the x coordinate of A is equal to x coordinate of B
	else {
		while (d < d_tot) {
			x = a.a;
			if(a.b < b.b) {
				y = a.b + d;
			}
			else {
				y = b.b + d;
			}
			VoronoiPoint p = VoronoiPoint(x,y);
			samples.push_back(p);
			d = d + 5;
		}	
	}
	samples.push_back(b);
}



/* function findCollision: function which computes if a point is inside an obstacle
   -parameters:
   	x: x coordinate of the point
	y: y coordinate of the point
	obsContours: vector which contains the expanded obstacles points 
    -return: true if the point is inside the obstacle, false if it is not
*/
bool findCollision(double x, double y, const std::vector<std::vector<cv::Point>>& obsContours) {
	bool r = false;
	double res;
	//for each obstacle check if the point is inside 
	for (int i = 0; i < obsContours.size() && !r; i++) {
		res = cv::pointPolygonTest(obsContours[i] , cv::Point2f(x,y) , true);
		if(res > 0)
			r = true;
	}
	return r;
}



/* function connector_singlepoint: function which computes if the right amount of road map points to connect to a new point 
   -parameters:
   	point: point to connect to the road map 
	offset: indicates the initial offset in which a point (start or end or victim) can connect itself to the road map points
	step: indicate sthe amount to add to offset each time until to reach the max_offset
	max_offset: indicates the max distance in which a point can connect itself to the road map points
	nConnections: indicate sthe max number of road map points that can be connected to the point (start or end or victim)
	victimGain: indicates the gain in space the robot earns if it saves a victim (m2 porpuses) 
	voronoiPaths: the road map
	obsContours: vector which contains the expanded obstacles points 
    -return: none
*/
void connector_singlepoint(VoronoiPoint point, int offset, int step, int max_offset, int nConnections, double victimGain, VoronoiResults *voronoiPaths, const std::vector<std::vector<cv::Point>>& obsContours)
{
	std::vector<GraphEdge> connections;
	int longId;
	int connectionsSize = 1000;
	longId=encoder(point.a, point.b);
	
	//connect with the standard offset
	voronoiPaths->ids.push_back(longId);
	connect(offset, point, voronoiPaths, true, victimGain, &connections, -1, obsContours);
	connectionsSize=connections.size();

	//if the number of points using the standard offset is less than threshold, increase the offset and try again 
	while(connectionsSize<=(nConnections)*2)
	{
		offset=offset+step;
		connections.clear();
		connect(offset, point, voronoiPaths, connectionsSize<=(nConnections*2), victimGain, &connections, -1, obsContours);
		connectionsSize=connections.size();	
		if(offset>max_offset)
			connectionsSize=1000;		
	}
	
	//save the result edges from point to the road map points and viceversa
	for(int i=0;i<connections.size();i++)
		voronoiPaths->resultEdges.push_back(connections[i]);
}



/* function connector_netpoints: function which computes if the right amount of road map points to connect to a new point and to the cloud oints around it
   -parameters:
   	point: point to connect to the road map 
	offset: indicates the initial offset in which a point (start or end or victim) can connect itself to the road map points
	step: indicate sthe amount to add to offset each time until to reach the max_offset
	max_offset: indicates the max distance in which a point can connect itself to the road map points
	nConnections: indicate sthe max number of road map points that can be connected to the point (start or end or victim)
	victimGain: indicates the gain in space the robot earns if it saves a victim (m2 porpuses) 
	netRadius: indicates the distance from the end point to the cloud of points generated around it
	voronoiPaths: the road map
	obsContours: vector which contains the expanded obstacles points 
    -return: none
*/
void connector_netpoints(VoronoiPoint point, int offset, int step, int max_offset, int nConnections, double victimGain, VoronoiResults *voronoiPaths, int netRadius, const std::vector<std::vector<cv::Point>>& obsContours)
{
	std::vector<GraphEdge> connections;
	std::vector<VoronoiPoint> netVertex;
	int connectionsSize = 1000;
	int offset_2;
	int longId;
	int idPos;
	int x = point.a;
	int y = point.b;
		
	//insert the points of the cloud
	netVertex.push_back(VoronoiPoint(x, y+netRadius));
	netVertex.push_back(VoronoiPoint(x, y-netRadius));
	netVertex.push_back(VoronoiPoint(x+netRadius, y));
	netVertex.push_back(VoronoiPoint(x-netRadius, y));
	
	//insert the point into the cloud
	netVertex.push_back(VoronoiPoint(x, y));
	
	//for each element of the cloud
	for(int i=0;i<netVertex.size();i++)
	{
		x=netVertex[i].a;
		y=netVertex[i].b;
		longId=encoder(x,y);
		
		//connect with the standard offset
		voronoiPaths->ids.push_back(longId);
		connections.clear();
		connect(offset, netVertex[i], voronoiPaths, true, victimGain, &connections, -1, obsContours);
		connectionsSize=connections.size();
		
		offset_2=offset;
		//if the number of points using the standard offset is less than threshold, increase the offset and try again 
		while(connectionsSize<=((nConnections+i)*2))
		{
			offset_2=offset_2+step;
			connections.clear();
			connect(offset_2, netVertex[i], voronoiPaths, (connectionsSize<=((nConnections+i))*2), victimGain, &connections, -1, obsContours);
			connectionsSize=connections.size();
			if(offset_2>max_offset)
				connectionsSize=1000;
		}
		
		//save the result edges from point to the road map points and viceversa
		for(int j=0;j<connections.size();j++) {
			voronoiPaths->resultEdges.push_back(connections[j]);
		}
	}
}


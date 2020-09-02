#include "PathFinder.h"
#include "Dijkstra.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <stdlib.h> 
#include <limits.h> 
#include <stdio.h> 

//Quando faccio il path finder tra il punto iniziale e il punto intermedio, il punto intermedio non esiste nel voronoipath pervhè non è un punto della roadmap (come invece lo sono start e end), quindi
// è probabile che possa inserire un nuovo e a caso successivo rispetto alla ids size

//void connect(int, VoronoiPoint, VoronoiPoint, VoronoiResults*, bool, bool, std::vector<GraphEdge>*, std::vector<GraphEdge>*);
void connect(int, VoronoiPoint, VoronoiResults*, bool, double, std::vector<GraphEdge>*, int, const std::vector<std::vector<cv::Point>>&);
void shortestPath(VoronoiPoint, bool, int, VoronoiPoint, bool, int, VoronoiResults *, std::vector<VoronoiPoint> *);
//void sampleSegment(VoronoiPoint, VoronoiPoint, std::vector<VoronoiPoint>&);
VoronoiPoint midpoint(VoronoiPoint, VoronoiPoint);
bool edgeOnObstacle(VoronoiPoint, VoronoiPoint, const std::vector<std::vector<cv::Point>>&);
bool findCollision(double, double, const std::vector<std::vector<cv::Point>>&); 
void connector_netpoints(VoronoiPoint, int, int, int, int, double, VoronoiResults *, int, const std::vector<std::vector<cv::Point>>&);
void connector_singlepoint(VoronoiPoint, int, int, int, int, double, VoronoiResults *, const std::vector<std::vector<cv::Point>>&);
//int encoder(int, int);
//void decoder(int, int &, int &);

void PathFinder(VoronoiPoint start, bool startNew, VoronoiPoint end, bool endNew, VoronoiResults *voronoiPaths, std::vector<VoronoiPoint> *rightPath, int netRadius, const std::vector<std::vector<cv::Point>>& obsContours, const std::vector<std::pair<int,Polygon>> *victim_list, const std::string& config_folder)
{
	int offset;
	int offset_2;
	int max_offset;
	int max_offset_vic;
	int offset_3;
	int step;
	int nConnections;
	int idPos;
	int victimGain;
	int const netPointsNumber = 4;
	std::vector<GraphEdge> start_connection, end_connection;
	std::vector<VoronoiPoint> netVertex;

	//--lettura file parametri
	
	std::string file = config_folder+"/param.xml";
	cv::FileStorage param(file, cv::FileStorage::READ);

	victimGain = (int)param["victimGain"];
	std::cout << "victimGain -> " << victimGain << std::endl;
	
	max_offset = (int)param["max_offset"];
	std::cout << "max_offset -> " << max_offset << std::endl;

	max_offset_vic = (int)param["max_offset_vic"];
	std::cout << "max_offset -> " << max_offset << std::endl;

	offset = (int)param["offset"];
	std::cout << "offset -> " << offset << std::endl;

	nConnections = (int)param["nConnections"];
	std::cout << "nConnections -> " << nConnections << std::endl;

	step = (int)param["step"];
	std::cout << "step -> " << step << std::endl;
	
	//--

	//std::vector<GraphEdge> point_connections
	
	//connect(offset, start, end, voronoiPaths, true, true, &start_connection, &end_connection);
	
	if(startNew)
	{
		//if(victim_list == NULL)	
			connector_singlepoint(start, offset, step, max_offset, nConnections, victimGain, voronoiPaths, obsContours);
		//else
		//	connector_singlepoint(start, offset, step, max_offset, nConnections, 0, voronoiPaths, obsContours);
		/*startLongId=encoder(start.a, start.b);	
		voronoiPaths->ids.push_back(startLongId);
		std::cout << "start id " <<  startLongId << "    size : " << voronoiPaths->ids.size() << std::endl;

		connect(offset, start, voronoiPaths, true, &start_connection, -1, obsContours); //-1 indicate sthe right id is the last in ids vector
		startSize=start_connection.size();
		std::cout << "start Size ingress " <<  startSize << std::endl; */	
	}
	
	if(endNew)
	{
		if(netRadius>0)
		{	
			
			if(victim_list == NULL)				
				connector_netpoints(end, offset, step, max_offset, nConnections, victimGain, voronoiPaths, netRadius, obsContours);
			else
			{
				connector_singlepoint(end, offset, step, max_offset, nConnections, victimGain, voronoiPaths, obsContours);
				VoronoiPoint p;
				for(int j=0; j<victim_list->size(); j++)
				{
					p=VoronoiPoint((int)(victim_list->at(j).second[0].x*1000),(int)(victim_list->at(j).second[0].y*1000));
					//connector_netpoints(p, offset, step, max_offset, nConnections, 0, voronoiPaths, netRadius, obsContours);
					connector_singlepoint(p, offset, step, max_offset_vic, nConnections, 0, voronoiPaths, obsContours);
				}			
			}
			/*
			int x = end.a;
			int y = end.b;
		
			netVertex.push_back(VoronoiPoint(x, y+netRadius));
			netVertex.push_back(VoronoiPoint(x, y-netRadius));
			netVertex.push_back(VoronoiPoint(x+netRadius, y));
			netVertex.push_back(VoronoiPoint(x-netRadius, y));
			//netVertex.push_back(VoronoiPoint(x+(netRadius/2), y+(netRadius/2)));
			//netVertex.push_back(VoronoiPoint(x-(netRadius/2), y+(netRadius/2)));
			//netVertex.push_back(VoronoiPoint(x+(netRadius/2), y-(netRadius/2)));
			//netVertex.push_back(VoronoiPoint(x-(netRadius/2), y-(netRadius/2)));
			netVertex.push_back(VoronoiPoint(x, y));
		
			for(int i=0;i<netVertex.size();i++)
			{
				x=netVertex[i].a;
				y=netVertex[i].b;
				std::cout << "netVertex " << x << "," << y << " " << netVertex.size() <<std::endl;
				endLongId=encoder(x,y);
				voronoiPaths->ids.push_back(endLongId);
				end_connection.clear();
				connect(offset, netVertex[i], voronoiPaths, true, &end_connection, -1, obsContours);
				endSize=end_connection.size();
				std::cout << "end Size ingress r>0 " <<  endSize << std::endl;
				offset_2=offset;
				while(endSize<=(nConnection+i))
				{
					std::cout << i << " qui 1" << std::endl;
					offset_2=offset_2+step;
					end_connection.clear();
					connect(offset_2, netVertex[i], voronoiPaths, (endSize<=(nConnection+i)), &end_connection, -1, obsContours);
					endSize=end_connection.size();
					if(offset_2>max_offset)
						endSize=100;
				}
				std::cout << "end Size exit r>0" <<  end_connection.size() << std::endl;
				for(int j=0;j<end_connection.size();j++) {
					std::cout << i << " qui 2" << std::endl;
					voronoiPaths->resultEdges.push_back(end_connection[j]);
				}
			}
			std::cout << "end id " <<  endLongId << "    size : " << voronoiPaths->ids.size() << std::endl;
			*/
		}
		else
		{
			connector_singlepoint(end, offset, step, max_offset, nConnections, victimGain, voronoiPaths, obsContours);
			/*endLongId=encoder(end.a,end.b);
			voronoiPaths->ids.push_back(endLongId);
			std::cout << "end id " <<  endLongId << "    size : " << voronoiPaths->ids.size() << std::endl;
			connect(offset, end, voronoiPaths, true, &end_connection, -1, obsContours);
			endSize=end_connection.size();
			std::cout << "end Size ingress" <<  endSize << std::endl;*/
		}
		
	}

	//std::cout << "Step " << offset << std::endl;
	
	
	//// END /////////////////////////////////

	/*while(startSize<=nConnection || endSize<=nConnection)
	{
		offset=offset+step;
		std::cout << "Step " << offset << "size: " << startSize << " " << endSize <<std::endl;

		//connect(offset, start, end, voronoiPaths, startSize<=nConnection, endSize<=nConnection, &start_connection, &end_connection);
		if(startNew && startSize<=nConnection)
		{
			start_connection.clear();
			if(netRadius==0)
				connect(offset, start, voronoiPaths, startSize<=nConnection, &start_connection, -2, obsContours);
			else
				connect(offset, start, voronoiPaths, startSize<=nConnection, &start_connection, -6, obsContours);
			startSize=start_connection.size();
		}
		if(endNew && netRadius==0 && endSize<=nConnection)
		{
			end_connection.clear();
			connect(offset, end, voronoiPaths, endSize<=nConnection, &end_connection, -1, obsContours);
			endSize=end_connection.size();
		}
		
		if(offset>max_offset)
		{
			startSize=100;
			endSize=100;
		}
			
	}
	
	if(startNew) //maybe this section must be inside the prev while
	{
		std::cout << "start Size exit" <<  start_connection.size() << std::endl;
		for(int i=0;i<start_connection.size();i++)
			voronoiPaths->resultEdges.push_back(start_connection[i]);
	}
	if(endNew && netRadius==0)
	{
		std::cout << "end Size exit" <<  end_connection.size() << std::endl;
		for(int i=0;i<end_connection.size();i++)
			voronoiPaths->resultEdges.push_back(end_connection[i]);
	}
	*/
	std::cout << std::endl;
	std::cout << std::endl;
	//Check if there is an obstacle between start and end, if not connect directly the two points without using the roadMap, if so call the shirtestPath function
	if(edgeOnObstacle(start,end,obsContours) || victim_list != NULL)
	{
		if(netRadius==0)
		{
			std::cout << "NO SKIP - id: " << (voronoiPaths->ids.size()-2) << " -> " << start.a << "," << start.b << " " << startNew <<" - id:" << (voronoiPaths->ids.size()-1) << " -> " << end.a << "," << end.b << " " << endNew << std::endl;
			shortestPath(start, startNew, -2, end, endNew, -1, voronoiPaths, rightPath); //!!! se mission 0 ok -2,-1 		
		}
		else
		{
			int posStart,posEnd;
			if(victim_list == NULL)
			{
				posStart=(netPointsNumber+2)*(-1);
				posEnd=-1;
			}
			else
			{
				posStart=(2+victim_list->size())*(-1);	//posStart=(((1+victim_list->size())*(netPointsNumber)))*(-1);
				posEnd=(1+victim_list->size())*(-1);
			}
			std::cout << "NO SKIP - id: " << (voronoiPaths->ids.size()+posStart) << " -> " << start.a << "," << start.b << " " << startNew <<" - id:" << (voronoiPaths->ids.size()+posEnd) << " -> " << end.a << "," << end.b << " " << endNew << std::endl;
			shortestPath(start, startNew, posStart, end, endNew, posEnd, voronoiPaths, rightPath); //!!! se aggiungo punti, l id si sposta a -netvertex posizioni
		}
	}
	else
	{
		std::cout << "SKIP ";
		if(startNew)
		{
			std::cout << "START NEW - ";
			rightPath->push_back(start);
		}
		if(endNew)
		{
			std::cout << "END NEW";
			rightPath->push_back(end);
		}
		std::cout << std::endl;
	}
}


void shortestPath(VoronoiPoint start, bool startNew, int idStart, VoronoiPoint end, bool endNew, int idEnd, VoronoiResults *voronoiPaths, std::vector<VoronoiPoint> *rightPath)
{
	int V = voronoiPaths->ids.size(); 
	//std::cout << "V " << V << std::endl;
	std::cout << "V " << V << "  start (" << start.a << ", " << start.b << ") " << idStart << " - end (" << end.a << ", " << end.b << ") " << idEnd << std::endl;
	int k=0;
	int idNode1,idNode2=0;
	int path[V];
	int mapPosition[V];
	int encodedCoord,x,y;
	//int encoder;
	struct Graph* graph = createGraph(V); 
	for(int i=0;i<voronoiPaths->resultEdges.size();i++)
	{
		//std::cout << voronoiPaths->resultEdges[i].idFirstNode << " ; " << voronoiPaths->resultEdges[i].idSecondNode << "  ("<< voronoiPaths->resultEdges[i].p0.a << "," << voronoiPaths->resultEdges[i].p0.b << ")" << "  ("<< voronoiPaths->resultEdges[i].p1.a << "," << voronoiPaths->resultEdges[i].p1.b << ")  => " << voronoiPaths->resultEdges[i].length << std::endl;
			
		//Save the coordinates of the first node (in edge A-B, take A)
		x=voronoiPaths->resultEdges[i].p0.a;
		y=voronoiPaths->resultEdges[i].p0.b;
		std::cout << "(pathfinder) x = " << x << std::endl;
		std::cout << "(pathfinder) y = " << y << std::endl;
		//Encode coordinates x and y to an integer (xy one after the other)
		/*if(y<1000)
			//encodedCoord=(x*1000)+y;
			encodedCoord=( (x!=0?x:1) *1000)+y;
		else
			//encodedCoord=(x*10000)+y;
			encodedCoord=( (x!=0?x:1) *10000)+y;*/
		encodedCoord=encoder(x,y);	
		std::cout << voronoiPaths->resultEdges[i].idFirstNode << " (pathfinder) encodedCoord = " << encodedCoord << " - edgelen: " << voronoiPaths->resultEdges[i].length << " to " << voronoiPaths->resultEdges[i].idSecondNode << std::endl;
		//Save the coordinates in the array, each encoded coordinates in position using the id of the first node (edge A-B, take A id) 
		mapPosition[voronoiPaths->resultEdges[i].idFirstNode]=encodedCoord;
		//std::cout << "id " << voronoiPaths->resultEdges[i].idFirstNode << " coo " << encodedCoord << " coo2 " << mapPosition[voronoiPaths->resultEdges[i].idFirstNode] <<  std::endl; 
		//Create the edge in the graph used by Dijkstra algorithm using for the edge A-B the id of A, the id of B and the lenght of the segment A-B
		addEdge(graph, voronoiPaths->resultEdges[i].idFirstNode, voronoiPaths->resultEdges[i].idSecondNode, voronoiPaths->resultEdges[i].length); 		
	}
	
	std::cout << "AFTER SKIP/NO_SKIP - id: " << (voronoiPaths->ids.size()+idStart) << " -> " << mapPosition[voronoiPaths->ids.size()+idStart] <<  " " << startNew <<" - id:" << (voronoiPaths->ids.size()+idEnd) << " -> " << mapPosition[voronoiPaths->ids.size()+idEnd] << " " << endNew << std::endl;

	std::cout << std::endl;
	std::cout << std::endl;
	
	std::cout << "Dijkstra " << voronoiPaths->ids.size()+idStart << " " << voronoiPaths->ids.size()+idEnd << " - " << voronoiPaths->ids[voronoiPaths->ids.size()+idStart] << " " << voronoiPaths->ids[voronoiPaths->ids.size()+idEnd] << std::endl;

	//Call the Dijkstra algorithm using the graph generated before and the ids of the start and destination nodes
	dijkstra(graph,voronoiPaths->ids.size()+idStart,voronoiPaths->ids.size()+idEnd,path); 
	std::cout << "Dijkstra " << voronoiPaths->ids.size()+idStart << " " << voronoiPaths->ids.size()+idEnd << " - " << voronoiPaths->ids[voronoiPaths->ids.size()+idStart] << " " << voronoiPaths->ids[voronoiPaths->ids.size()+idEnd] << std::endl;
	std::cout << std::endl;
	std::cout << std::endl;
	std::cout << std::endl;

	std::cout << std::endl << "right path -1 ";
	for(int j=0;j<rightPath->size();j++) {
		std::cout << " " << rightPath->at(j).a << "," << rightPath->at(j).b << std::endl;
	}
	
	std::vector<int> startEndPath;
	startEndPath.push_back(voronoiPaths->ids.size()+idStart);
	std::cout << "startEndPath[0] : " << startEndPath[0] << std::endl;
	std::cout << "(pathfinder) mapposition : " << mapPosition[startEndPath[0]] << std::endl;
	std::cout << "voronoiPaths->ids.size()+idStart : " << voronoiPaths->ids.size()+idStart << std::endl;
	storePath(path,voronoiPaths->ids.size()+idEnd,&startEndPath);
	
	
	std::cout << std::endl << "right path ";
	for(int j=0;j<startEndPath.size();j++) {
		std::cout << " " << startEndPath[j];
		std::cout << "(pathfinder) mapposition : " << mapPosition[startEndPath[j]] << std::endl;
	}

	if(startEndPath.size()<2)
		throw std::logic_error("STOP_possible_path_not_found");
	std::cout << std::endl;
	std::cout << std::endl;
	//std::cout << "map " << mapPosition[111] << std::endl;
	
	if(startNew) {
		std::cout << "startNew inside" << std::endl;	
		rightPath->push_back(start);
	}
	std::cout << "node start: " << startEndPath[0] << " " << mapPosition[startEndPath[0]] << std::endl;
	int x1,y1;
	for(int i=1;i<startEndPath.size()-1;i++)
	{
		std::cout << "(pathfider) startEndPath[" << i << "] : " << startEndPath[i] << std::endl;
		//if(mapPosition[voronoiPaths->resultEdges[startEndPath[i]].idFirstNode]<10000)
		/*if(mapPosition[startEndPath[i]]<10000)
			encoder=100;
		else
			encoder=1000;
		x1 = mapPosition[startEndPath[i]]/encoder;
		y1 = mapPosition[startEndPath[i]]-(x1*encoder); */
		decoder(mapPosition[startEndPath[i]],x1,y1);
		
		VoronoiPoint vertex = VoronoiPoint(x1,y1);
		rightPath->push_back(vertex);
	
		std::cout << "node: " << startEndPath[i] << " " << mapPosition[startEndPath[i]] << " x,y (" << x1 << "," << y1 << ")" << std::endl;
	}
	if(endNew) {
		std::cout << "endNew inside" << std::endl;
		rightPath->push_back(end);
	}
	std::cout << "node end: " << startEndPath[startEndPath.size()-1] << " " << mapPosition[startEndPath[startEndPath.size()-1]] << std::endl;
	
	std::cout << std::endl << "right path 2 ";
	for(int j=0;j<rightPath->size();j++) {
		std::cout << " " << rightPath->at(j).a << "," << rightPath->at(j).b << std::endl;
	}

}

/*
int encoder(int x, int y)
{
	return (x*10000)+y;
}

void decoder(int encoded, int &x, int &y)    
{
	x=encoded/10000;
	y=encoded-(x*10000);
}
*/





//////////////////////////////////////
//void connect(int offset, VoronoiPoint startP, VoronoiPoint endP, VoronoiResults *voronoiPaths, bool startOk, bool endOk, std::vector<GraphEdge> *start, std::vector<GraphEdge> *end)
void connect(int offset, VoronoiPoint pointP, VoronoiResults *voronoiPaths, bool pointOK, double vGain, std::vector<GraphEdge> *pointEdges, int IDpos, const std::vector<std::vector<cv::Point>>& obsContours)
{
	int a,b,c,d,o,x,y,endId,startId,id1;
	int prev_x,prev_y=-1;
	double len;
	int longId,startLongId,endLongId;

	int pointLongId;
	int pointId; 
	bool find;
	a=pointP.a-offset;
	b=pointP.b-offset;
	o=offset*2;
	
	pointLongId=encoder(pointP.a, pointP.b);
	
	pointId=voronoiPaths->ids.size()+IDpos; 

	std::cout << std::endl;
	std::cout << std::endl;
	std::cout << std::endl;
	std::cout << "at the beginning of connect, offset " << offset << std::endl;

	for(int i=0;i<voronoiPaths->resultEdges.size();i++)
	{
		x=voronoiPaths->resultEdges[i].p0.a;
		y=voronoiPaths->resultEdges[i].p0.b;
		
		if(pointOK)// && (a<x) && (x<(a+o)) && ((b<y) && (y<(b+o))))
		{
			len=sqrt(pow((x-pointP.a),2)+pow((y-pointP.b),2));
			len=len+vGain;
			if(len<=((offset*1.0)+vGain))
			{
				std::cout << "connect 1 if" << std::endl;
				if(x!=prev_x || y!=prev_y)
				{
					std::cout << "connect 2 if" << std::endl;
					if(!edgeOnObstacle(pointP,VoronoiPoint(x,y),obsContours))
					{
						longId=encoder(x,y);
						
						id1=-1;
						for(int j=0;j<voronoiPaths->ids.size() && id1==-1;j++)
							if(voronoiPaths->ids[j]==longId)
								id1=j;
						
						std::cout << "connect 3 if" << std::endl;
						find=false;
					////////////
						for(int k=0;k<pointEdges->size() && !find;k++)
						{
							if(pointEdges->at(k).idFirstNode==id1)
								find=true;
						}
					///////////
						for(int j=(voronoiPaths->resultEdges.size()-1);j>=0 && !find ;j--)
						{
							if(voronoiPaths->resultEdges[j].idFirstNode==pointId && voronoiPaths->resultEdges[j].idSecondNode==id1)
								find=true;
						}
						
						if(!find)
						{
							std::cout << std::endl;
							std::cout << "offset: " << offset << " - edge_vect size: " << (voronoiPaths->resultEdges.size()-1) << " - " << std::endl;
							std::cout << "connect points (" << x << "," << y << ") and (" << pointP.a << "," << pointP.b << ") - len: " << len;
							std::cout << " - id " << id1 << "  -  longId " << longId << "  -  pointLongId " << pointLongId << "  -  PointId " << pointId << std::endl;
							std::cout << std::endl;
							find=false;
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
	std::cout << "at the end of connect, nconnections " << pointEdges->size() << std::endl;
	
}	

bool edgeOnObstacle(VoronoiPoint a, VoronoiPoint b, const std::vector<std::vector<cv::Point>>& obsContours)
{
	std::vector<VoronoiPoint> samples;
	bool isCollision=false;
	sampleSegment2(a, b, samples);
	//sampleSegment(a, b, samples);
	//std::cout << "------ n samples " << samples.size() << std::endl; 
	for(int i=0; i<samples.size() && isCollision==false;i++)
	{
		//std::cout << "------ sample " << i << " coord(" << samples[i].a << "," << samples[i].b << ")" << std::endl; 
		isCollision=findCollision(samples[i].a, samples[i].b, obsContours);
	}
	if(isCollision)
		std::cout << "BOOOOOOOOOOM " << std::endl; 
	return isCollision;
	//return true;
}
	

VoronoiPoint midpoint(VoronoiPoint a, VoronoiPoint b)
{	
	int x = (int)(a.a+b.a)/2;
	int y = (int)(a.b+b.b)/2;
	return VoronoiPoint(x,y);	
}

/*
void sampleSegment(VoronoiPoint a, VoronoiPoint b, std::vector<VoronoiPoint>& samples)
{
	// order in samples -> A 7 3 9 1 11 5 13 0 14 6 12 2 10 4 8 B
	VoronoiPoint mid = midpoint(a,b);
	samples.push_back(mid);
	mid = midpoint(a,samples[0]);                  
	samples.push_back(mid);
	mid = midpoint(b,samples[0]);
	samples.push_back(mid);

	mid = midpoint(a,samples[1]);
	samples.push_back(mid);
	mid = midpoint(b,samples[2]);
	samples.push_back(mid);

	mid = midpoint(samples[0], samples[1]);
	samples.push_back(mid);
	mid = midpoint(samples[0], samples[2]);
	samples.push_back(mid);

	mid = midpoint(a, samples[3]);
	samples.push_back(mid);
	mid = midpoint(b, samples[4]);
	samples.push_back(mid);

	mid = midpoint(samples[3], samples[1]);
	samples.push_back(mid);
	mid = midpoint(samples[4], samples[2]);
	samples.push_back(mid);

	mid = midpoint(samples[1], samples[5]);
	samples.push_back(mid);
	mid = midpoint(samples[6], samples[2]);
	samples.push_back(mid);

	mid = midpoint(samples[0], samples[5]);
	samples.push_back(mid);
	mid = midpoint(samples[6], samples[0]);
	samples.push_back(mid);

	samples.push_back(a);
	samples.push_back(b);
}
*/

void sampleSegment2(VoronoiPoint a, VoronoiPoint b, std::vector<VoronoiPoint>& samples) {
	//std::cout << "Tratto " << a.a << ", " << a.b << "  ->  " << b.a << ", " << b.b << std::endl;
	samples.push_back(a);
	float d = 5;
	float d_tot2 = (((a.a - b.a) * (a.a - b.a)) + ((a.b - b.b) * (a.b - b.b)));
	float d_tot = (std::sqrt(d_tot2));
	//std::cout << "Lunghezza : " << d_tot << std::endl;
	int x, x1, x2, y, y1, y2;
	if (a.a != b.a) {
		float m = ((float)(a.b - b.b) / (float)(a.a - b.a));
		float q = (a.b - (m * a.a));
		float par_a = ((m * m) + 1);
		float par_b = ((2 * m * q) - (2 * a.a) - (2 * a.b * m));
		float par_c;
		while (d < d_tot) { 
		//	std::cout << "Distanza sample : " << d << std::endl;
			par_c = (a.a * a.a) + (a.b * a.b) + (q * q) - (2 * a.b * q) - (d * d);
			x1 = (int)(- par_b + std::sqrt((par_b * par_b) - (4 * par_a * par_c))) / (2 * par_a);
			x2 = (int)(- par_b - std::sqrt((par_b * par_b) - (4 * par_a * par_c))) / (2 * par_a);
			y1 = (int)(m * x1) + q; 
			y2 = (int)(m * x2) + q;
		//	std::cout << "Soluzione 1 : " << x1 << ", " << y1 << std::endl;
		//	std::cout << "Soluzione 2 : " << x2 << ", " << y2 << std::endl;
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
		//	std::cout << "Sample : " << x << ", " << y << " == " << p.a << ", " << p.b << std::endl;
			d = d + 5;
		}
	}
	else {
		while (d < d_tot) {
		//	std::cout << "Distanza sample : " << d << std::endl;
			x = a.a;
			if(a.b < b.b) {
				y = a.b + d;
			}
			else {
				y = b.b + d;
			}
			VoronoiPoint p = VoronoiPoint(x,y);
			samples.push_back(p);
		//	std::cout << "Sample : " << x << ", " << y << std::endl;
			d = d + 5;
		}
		
	}
	samples.push_back(b);
}

bool findCollision(double x, double y, const std::vector<std::vector<cv::Point>>& obsContours) {
	bool r = false;
	double res;
	for (int i = 0; i < obsContours.size() && !r; i++) {
		res = cv::pointPolygonTest(obsContours[i] , cv::Point2f(x,y) , true);
		if(res > 0)
			r = true;
	}
	return r;
}

void connector_singlepoint(VoronoiPoint point, int offset, int step, int max_offset, int nConnections, double victimGain, VoronoiResults *voronoiPaths, const std::vector<std::vector<cv::Point>>& obsContours)
{
	std::vector<GraphEdge> connections;
	int longId;
	int connectionsSize = 1000;
	longId=encoder(point.a, point.b);	
	voronoiPaths->ids.push_back(longId);
	connect(offset, point, voronoiPaths, true, victimGain, &connections, -1, obsContours); //-1 indicate sthe right id is the last in ids vector
	connectionsSize=connections.size();

	while(connectionsSize<=(nConnections)*2)
	{
		offset=offset+step;
		connections.clear();
		connect(offset, point, voronoiPaths, connectionsSize<=(nConnections*2), victimGain, &connections, -1, obsContours);
		connectionsSize=connections.size();	
		if(offset>max_offset)
			connectionsSize=1000;		
	}
	for(int i=0;i<connections.size();i++)
		voronoiPaths->resultEdges.push_back(connections[i]);
}

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
			
	netVertex.push_back(VoronoiPoint(x, y+netRadius));
	netVertex.push_back(VoronoiPoint(x, y-netRadius));
	netVertex.push_back(VoronoiPoint(x+netRadius, y));
	netVertex.push_back(VoronoiPoint(x-netRadius, y));
	//netVertex.push_back(VoronoiPoint(x+(netRadius/2), y+(netRadius/2)));
	//netVertex.push_back(VoronoiPoint(x-(netRadius/2), y+(netRadius/2)));
	//netVertex.push_back(VoronoiPoint(x+(netRadius/2), y-(netRadius/2)));
	//netVertex.push_back(VoronoiPoint(x-(netRadius/2), y-(netRadius/2)));
	netVertex.push_back(VoronoiPoint(x, y));
	
	for(int i=0;i<netVertex.size();i++)
	{
		x=netVertex[i].a;
		y=netVertex[i].b;
		
		longId=encoder(x,y);
		voronoiPaths->ids.push_back(longId);
		connections.clear();
		connect(offset, netVertex[i], voronoiPaths, true, victimGain, &connections, -1, obsContours);
		connectionsSize=connections.size();
		
		offset_2=offset;
		while(connectionsSize<=((nConnections+i)*2))
		{
			offset_2=offset_2+step;
			connections.clear();
			connect(offset_2, netVertex[i], voronoiPaths, (connectionsSize<=((nConnections+i))*2), victimGain, &connections, -1, obsContours);
			connectionsSize=connections.size();
			if(offset_2>max_offset)
				connectionsSize=1000;
		}
		
		for(int j=0;j<connections.size();j++) {
			voronoiPaths->resultEdges.push_back(connections[j]);
		}
	}
}


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
void connect(int, VoronoiPoint, VoronoiResults*, bool, std::vector<GraphEdge>*, int, const std::vector<std::vector<cv::Point>>&);
void shortestPath(VoronoiPoint, bool, int, VoronoiPoint, bool, int, VoronoiResults *, std::vector<VoronoiPoint> *);
void sampleSegment(VoronoiPoint, VoronoiPoint, std::vector<VoronoiPoint>&);
VoronoiPoint midpoint(VoronoiPoint, VoronoiPoint);
bool edgeOnObstacle(VoronoiPoint, VoronoiPoint, const std::vector<std::vector<cv::Point>>&);
bool findCollision(double, double, const std::vector<std::vector<cv::Point>>&); 
int encoder(int, int);
void decoder(int, int &, int &);

void PathFinder(VoronoiPoint start, bool startNew, VoronoiPoint end, bool endNew, VoronoiResults *voronoiPaths, std::vector<VoronoiPoint> *rightPath, int netRadius, const std::vector<std::vector<cv::Point>>& obsContours)
{
	int offset=20;
	int offset_2;
	int max_offset = 200;
	int offset_3;
	int step=10;
	int nConnection=8;
	int startSize=1000;
	int endSize=1000;
	int startLongId;
	int endLongId;2
	int idPos;
	std::vector<GraphEdge> start_connection, end_connection;
	VoronoiPoint v1, v2, v3, v4, v5, v6, v7, v8;
	std::vector<VoronoiPoint> netVertex;

	//std::vector<GraphEdge> point_connections
	
	//connect(offset, start, end, voronoiPaths, true, true, &start_connection, &end_connection);
	
	if(startNew)
	{
		/*if(start.b<1000)
			startLongId=(start.a*1000)+start.b;
		else
			startLongId=(start.a*10000)+start.b;*/
		startLongId=encoder(start.a, start.b);	
		voronoiPaths->ids.push_back(startLongId);
		std::cout << "start id " <<  startLongId << std::endl;

		connect(offset, start, voronoiPaths, true, &start_connection, -1, obsContours); //-1 indicate sthe right id is the last in ids vector
		startSize=start_connection.size();
		std::cout << "start Size ingress " <<  startSize << std::endl;
	}
	
	if(endNew)
	{
		if(netRadius>0)
		{
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
				/*if(y<1000)
					endLongId=(x*1000)+y;
				else
					endLongId=(x*10000)+y;*/
				endLongId=encoder(x,y);
				voronoiPaths->ids.push_back(endLongId);
				end_connection.clear();
				connect(offset, netVertex[i], voronoiPaths, true, &end_connection, -1, obsContours);
				endSize=end_connection.size();
				std::cout << "end Size ingress r>0 " <<  endSize << std::endl;
				offset_2=offset;
				while(endSize<=(nConnection+i))
				{
					offset_2=offset_2+step;
					end_connection.clear();
					connect(offset_2, netVertex[i], voronoiPaths, endSize<=(nConnection+i), &end_connection, -1, obsContours);
					endSize=end_connection.size();
					if(offset_2>max_offset)
						endSize=100;
				}
				std::cout << "end Size exit r>0" <<  end_connection.size() << std::endl;
				for(int j=0;j<end_connection.size();j++)
					voronoiPaths->resultEdges.push_back(end_connection[i]);
			}
		}
		else
		{
			/*if(end.b<1000)
				endLongId=(end.a*1000)+end.b;
			else
				endLongId=(end.a*10000)+end.b;*/
			endLongId=encoder(end.a,end.b);
			voronoiPaths->ids.push_back(endLongId);
			std::cout << "end id " <<  endLongId << std::endl;
			connect(offset, end, voronoiPaths, true, &end_connection, -1, obsContours);
			endSize=end_connection.size();
			std::cout << "end Size ingress" <<  endSize << std::endl;
		}
	}

	std::cout << "Step " << offset << std::endl;
	
	
	//// END /////////////////////////////////

	while(startSize<=nConnection || endSize<=nConnection)
	{
		offset=offset+step;
		std::cout << "Step " << offset << "size: " << startSize << " " << endSize <<std::endl;

		//connect(offset, start, end, voronoiPaths, startSize<=nConnection, endSize<=nConnection, &start_connection, &end_connection);
		if(startNew && startSize<=nConnection)
		{
			start_connection.clear();
			connect(offset, start, voronoiPaths, startSize<=nConnection, &start_connection, -2, obsContours);
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
	
	if(netRadius==0)
		shortestPath(start, startNew, -2, end, endNew, -1, voronoiPaths, rightPath); //!!! se mission 0 ok -2,-1 
	else
		shortestPath(start, startNew, -6, end, endNew, -1, voronoiPaths, rightPath); //!!! se aggiungo punti, l id si sposta a -netvertex posizioni
}


void shortestPath(VoronoiPoint start, bool startNew, int idStart, VoronoiPoint end, bool endNew, int idEnd, VoronoiResults *voronoiPaths, std::vector<VoronoiPoint> *rightPath)
{
	int V = voronoiPaths->ids.size(); 
	std::cout << "V " << V << std::endl;
	int k=0;
	int idNode1,idNode2=0;
	int path[V];
	int mapPosition[V];
	int encodedCoord,x,y;
	int encoder;
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
		std::cout << "(pathfinder) encodedCoord = " << encodedCoord << std::endl;
		//Save the coordinates in the array, each encoded coordinates in position using the id of the first node (edge A-B, take A id) 
		mapPosition[voronoiPaths->resultEdges[i].idFirstNode]=encodedCoord;
		//std::cout << "id " << voronoiPaths->resultEdges[i].idFirstNode << " coo " << encodedCoord << " coo2 " << mapPosition[voronoiPaths->resultEdges[i].idFirstNode] <<  std::endl; 
		//Create the edge in the graph used by Dijkstra algorithm using for the edge A-B the id of A, the id of B and the lenght of the segment A-B
		addEdge(graph, voronoiPaths->resultEdges[i].idFirstNode, voronoiPaths->resultEdges[i].idSecondNode, voronoiPaths->resultEdges[i].length); 
		
	}

	
	std::cout << "Dijkstra " << voronoiPaths->ids.size()+idStart << " " << voronoiPaths->ids.size()+idEnd << " - " << voronoiPaths->ids[voronoiPaths->ids.size()+idStart] << " " << voronoiPaths->ids[voronoiPaths->ids.size()+idEnd] << std::endl;

	//Call the Dijkstra algorithm using the graph generated before and the ids of the start and destination nodes
	dijkstra(graph,voronoiPaths->ids.size()+idStart,voronoiPaths->ids.size()+idEnd,path); 
	
	std::cout << std::endl;
	std::cout << std::endl;
	std::cout << std::endl;
	
	std::vector<int> startEndPath;
	startEndPath.push_back(voronoiPaths->ids.size()+idStart);
	storePath(path,voronoiPaths->ids.size()+idEnd,&startEndPath);
	
	
	std::cout << std::endl << "right path ";
	for(int j=0;j<startEndPath.size();j++)
		std::cout << " " << startEndPath[j];
	std::cout << std::endl;
	std::cout << std::endl;
	//std::cout << "map " << mapPosition[111] << std::endl;
	
	if(startNew)
		rightPath->push_back(start);
	int x1,y1;
	for(int i=1;i<startEndPath.size()-1;i++)
	{
		std::cout << "(pathfider) mapPosition[voronoiPaths->resultEdges[startEndPath[i]].idFirstNode] : " << mapPosition[voronoiPaths->resultEdges[startEndPath[i]].idFirstNode] << std::endl;
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
	if(endNew)
		rightPath->push_back(end);
	
}

int encoder(int x, int y)
{
	return (x*10000)+y;
}

void decoder(int encoded, int &x, int &y)    
{
	x=encoded/10000;
	y=encoded-(x*10000);
}






//////////////////////////////////////
//void connect(int offset, VoronoiPoint startP, VoronoiPoint endP, VoronoiResults *voronoiPaths, bool startOk, bool endOk, std::vector<GraphEdge> *start, std::vector<GraphEdge> *end)
void connect(int offset, VoronoiPoint pointP, VoronoiResults *voronoiPaths, bool pointOK, std::vector<GraphEdge> *pointEdges, int IDpos, const std::vector<std::vector<cv::Point>>& obsContours)
{
	int a,b,c,d,o,x,y,endId,startId,id1;
	int prev_x,prev_y=-1;
	double len;
	int longId,startLongId,endLongId;

	//a=startP.a-offset;
	//b=startP.b-offset;
	//c=endP.a-offset;
	//d=endP.b-offset;
	//o=offset*2;
	
	///////////INIT////////////
	int pointLongId;
	int pointId; 
	a=pointP.a-offset;
	b=pointP.b-offset;
	o=offset*2;
	
	/*
	if(pointP.b<1000)
		pointLongId=(pointP.a*1000)+pointP.b;
	else
		pointLongId=(pointP.a*10000)+pointP.b;*/
	pointLongId=encoder(pointP.a, pointP.b);
	
	pointId=voronoiPaths->ids.size()+IDpos; // what is the right number??? random greater than ids.size??
	//startId=voronoiPaths->ids.size()-2; //penultimo
	//endId=voronoiPaths->ids.size()-1; //ultimo
	
	for(int i=0;i<voronoiPaths->resultEdges.size();i++)
	{
		x=voronoiPaths->resultEdges[i].p0.a;
		y=voronoiPaths->resultEdges[i].p0.b;
		
		if(pointOK && (a<x) && (x<(a+o)) && ((b<y) && (y<(b+o))))
		{
			len=sqrt(pow((x-pointP.a),2)+pow((y-pointP.b),2));
			if(x!=prev_x || y!=prev_y)
			{
				std::cout << "connect points (" << x << "," << y << ") and (" << pointP.a << "," << pointP.b << ")" << std::endl;
				if(!edgeOnObstacle(pointP,VoronoiPoint(x,y),obsContours))
				{
					/*
					if(y<1000)
						longId=(x*1000)+y;
					else
						longId=(x*10000)+y;*/
					longId=encoder(x,y);
					std::cout << "longId " << longId << std::endl;
					id1=-1;
					for(int j=0;j<voronoiPaths->ids.size() && id1==-1;j++)
						if(voronoiPaths->ids[j]==longId)
							id1=j;
					std::cout << "id1 " << id1 << std::endl;
					GraphEdge e(x,y,pointP.a,pointP.b,len,id1,pointId);
					pointEdges->push_back(e);
				}
				prev_x=x;
				prev_y=y;
			}
		}
	}
	
}	

bool edgeOnObstacle(VoronoiPoint a, VoronoiPoint b, const std::vector<std::vector<cv::Point>>& obsContours)
{
	std::vector<VoronoiPoint> samples;
	bool isCollision=false;
	sampleSegment(a, b, samples);
	std::cout << "------ n samples " << samples.size() << std::endl; 
	for(int i=0; i<samples.size() && isCollision==false;i++)
	{
		std::cout << "------ sample " << i << " coord(" << samples[i].a << "," << samples[i].b << ")" << std::endl; 
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

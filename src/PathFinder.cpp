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
void connect(int, VoronoiPoint, VoronoiResults*, bool, std::vector<GraphEdge>*, int);
void shortestPath(VoronoiPoint, VoronoiPoint, VoronoiResults *, std::vector<VoronoiPoint> *);

void PathFinder(VoronoiPoint start, bool startNew, VoronoiPoint end, bool endNew, VoronoiResults *voronoiPaths, std::vector<VoronoiPoint> *rightPath, bool areVictims)
{
	int offset=20;
	int step=10;
	int nConnection=5;
	int startSize=1000;
	int endSize=1000;
	int startLongId;
	int endLongId;
	int idPos;
	std::vector<GraphEdge> start_connection, end_connection;

	//std::vector<GraphEdge> point_connections
	
	//connect(offset, start, end, voronoiPaths, true, true, &start_connection, &end_connection);
	
	if(startNew)
	{
		if(start.b<1000)
			startLongId=(start.a*1000)+start.b;
		else
			startLongId=(start.a*10000)+start.b;
		voronoiPaths->ids.push_back(startLongId);
	
		connect(offset, start, voronoiPaths, true, &start_connection, -1); //-1 indicate sthe right id is the last in ids vector
		startSize=start_connection.size();
	}
	
	if(endNew)
	{
		if(end.b<1000)
			endLongId=(end.a*1000)+end.b;
		else
			endLongId=(end.a*10000)+end.b;
		voronoiPaths->ids.push_back(endLongId);
	
		connect(offset, end, voronoiPaths, true, &end_connection, -1);
		endSize=end_connection.size();
	}

	std::cout << "Step " << offset << std::endl;
	
	
	//// END /////////////////////////////////
	
	while(startSize<=nConnection || endSize<=nConnection)
	{
		offset=offset+step;
		std::cout << "Step " << offset << "size: " << startSize << " " << endSize <<std::endl;
		if(startSize<=nConnection)
			start_connection.clear();
		else
			end_connection.clear();

		//connect(offset, start, end, voronoiPaths, startSize<=nConnection, endSize<=nConnection, &start_connection, &end_connection);
		if(startNew)
		{
			connect(offset, start, voronoiPaths, startSize<=nConnection, &start_connection, -2);
			startSize=start_connection.size();
		}
		if(endNew)
		{
			connect(offset, end, voronoiPaths, endSize<=nConnection, &end_connection, -1);
			endSize=end_connection.size();
		}
	}
	
	if(startNew)
	{
		for(int i=0;i<start_connection.size();i++)
			voronoiPaths->resultEdges.push_back(start_connection[i]);
	}
	if(endNew)
	{
		for(int i=0;i<end_connection.size();i++)
			voronoiPaths->resultEdges.push_back(end_connection[i]);
	}
	shortestPath(start, end, voronoiPaths, rightPath);
}

/*
void connect(int offset, VoronoiPoint startP, VoronoiPoint endP, VoronoiResults *voronoiPaths, bool startOk, bool endOk, std::vector<GraphEdge> *start, std::vector<GraphEdge> *end)
{
	int a,b,c,d,o,x,y,endId,startId,id1;
	int prev_x,prev_y=-1;
	double len;
	int longId,startLongId,endLongId;

	a=startP.a-offset;
	b=startP.b-offset;
	c=endP.a-offset;
	d=endP.b-offset;
	o=offset*2;

	if(startP.b<1000)
		startLongId=(startP.a*1000)+startP.b;
	else
		startLongId=(startP.a*10000)+startP.b;
	//voronoiPaths->ids.push_back(startLongId);
	startId=voronoiPaths->ids.size()-2;
	
	std::cout << "startID " << startId << " | startLongID " << startLongId << std::endl;
	
	if(endP.b<1000)
		endLongId=(endP.a*1000)+endP.b;
	else
		endLongId=(endP.a*10000)+endP.b;
	//voronoiPaths->ids.push_back(endLongId);
	endId=voronoiPaths->ids.size()-1;

	std::cout << "endID " << endId << " | endLongID " << endLongId << std::endl;

	for(int i=0;i<voronoiPaths->resultEdges.size();i++)
	{
		x=voronoiPaths->resultEdges[i].p0.a;
		y=voronoiPaths->resultEdges[i].p0.b;
		
		if(startOk && (a<x) && (x<(a+o)) && ((b<y) && (y<(b+o))))
		{
			len=sqrt(pow((x-startP.a),2)+pow((y-startP.b),2));
			if(x!=prev_x || y!=prev_y)
			{
				if(y<1000)
					longId=(x*1000)+y;
				else
					longId=(x*10000)+y;
				std::cout << "longId " << longId << std::endl;
				id1=-1;
				for(int j=0;j<voronoiPaths->ids.size() && id1==-1;j++)
					if(voronoiPaths->ids[j]==longId)
						id1=j;
				std::cout << "id1 " << id1 << std::endl;
				GraphEdge e(x,y,startP.a,startP.b,len,id1,startId);
				start->push_back(e);
				prev_x=x;
				prev_y=y;
			}
		}
		if(endOk && (c<x) && (x<(c+o)) && ((d<y) && (y<(d+o))))
		{
			len=sqrt(pow((x-endP.a),2)+pow((y-endP.b),2));
			if(x!=prev_x || y!=prev_y)
			{
				if(y<1000)
					longId=(x*1000)+y;
				else
					longId=(x*10000)+y;
				std::cout << "longId " << longId << std::endl;
				id1=-1;
				for(int k=0;k<voronoiPaths->ids.size() && id1==-1;k++)
					if(voronoiPaths->ids[k]==longId)
						id1=k;
				std::cout << "id1 " << id1 << std::endl;
				GraphEdge e(x,y,endP.a,endP.b,len,id1,endId);
				end->push_back(e);
				prev_x=x;
				prev_y=y;
			}
		}
	}
 	//return id;
}
*/


void shortestPath(VoronoiPoint start,VoronoiPoint end, VoronoiResults *voronoiPaths, std::vector<VoronoiPoint> *rightPath)
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
		std::cout << voronoiPaths->resultEdges[i].idFirstNode << " ; " << voronoiPaths->resultEdges[i].idSecondNode << "  ("<< voronoiPaths->resultEdges[i].p0.a << "," << voronoiPaths->resultEdges[i].p0.b << ")" << "  ("<< voronoiPaths->resultEdges[i].p1.a << "," << voronoiPaths->resultEdges[i].p1.b << ")  => " << voronoiPaths->resultEdges[i].length << std::endl;
			
		x=voronoiPaths->resultEdges[i].p0.a;
		y=voronoiPaths->resultEdges[i].p0.b;
		if(y<1000)
			encodedCoord=(x*1000)+y;
		else
			encodedCoord=(x*10000)+y;
		
		mapPosition[voronoiPaths->resultEdges[i].idFirstNode]=encodedCoord;
		std::cout << "id " << voronoiPaths->resultEdges[i].idFirstNode << " coo " << encodedCoord << " coo2 " << mapPosition[voronoiPaths->resultEdges[i].idFirstNode] <<  std::endl; 
		addEdge(graph, voronoiPaths->resultEdges[i].idFirstNode, voronoiPaths->resultEdges[i].idSecondNode, voronoiPaths->resultEdges[i].length); 
		
	}

	dijkstra(graph,voronoiPaths->ids.size()-2,voronoiPaths->ids.size()-1,path); //modificare aggiunendo il voronoiPaths->ids.size()-1 ossia l'id del nodo destinazione e tracciarlo in modo da avere la lista degli id dei nodi da visitare per arrivare da start a end
	
	std::cout << std::endl;
	
	std::vector<int> startEndPath;
	startEndPath.push_back(voronoiPaths->ids.size()-2);
	storePath(path,voronoiPaths->ids.size()-1,&startEndPath);
	
	
	std::cout << std::endl << "right path ";
	for(int j=0;j<startEndPath.size();j++)
		std::cout << " " << startEndPath[j];
	std::cout << std::endl;
	std::cout << "map " << mapPosition[111] << std::endl;
	
	rightPath->push_back(start);
	int x1,y1;
	for(int i=1;i<startEndPath.size()-1;i++)
	{
		if(mapPosition[voronoiPaths->resultEdges[startEndPath[i]].idFirstNode]<10000)
			encoder=100;
		else
			encoder=1000;
		x1 = mapPosition[startEndPath[i]]/encoder;
		y1 = mapPosition[startEndPath[i]]-(x1*encoder); 
		
		VoronoiPoint vertex = VoronoiPoint(x1,y1);
		rightPath->push_back(vertex);
	
		std::cout << "node: " << startEndPath[i] << " " << mapPosition[startEndPath[i]] << " x,y (" << x1 << "," << y1 << ")" << std::endl;
	}
	rightPath->push_back(end);
	
}






//////////////////////////////////////
//void connect(int offset, VoronoiPoint startP, VoronoiPoint endP, VoronoiResults *voronoiPaths, bool startOk, bool endOk, std::vector<GraphEdge> *start, std::vector<GraphEdge> *end)
void connect(int offset, VoronoiPoint pointP, VoronoiResults *voronoiPaths, bool pointOK, std::vector<GraphEdge> *pointEdges, int IDpos)
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
	
	if(pointP.b<1000)
		pointLongId=(pointP.a*1000)+pointP.b;
	else
		pointLongId=(pointP.a*10000)+pointP.b;
	
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
				if(y<1000)
					longId=(x*1000)+y;
				else
					longId=(x*10000)+y;
				std::cout << "longId " << longId << std::endl;
				id1=-1;
				for(int j=0;j<voronoiPaths->ids.size() && id1==-1;j++)
					if(voronoiPaths->ids[j]==longId)
						id1=j;
				std::cout << "id1 " << id1 << std::endl;
				GraphEdge e(x,y,pointP.a,pointP.b,len,id1,pointId);
				pointEdges->push_back(e);
				prev_x=x;
				prev_y=y;
			}
		}
	}
	
	
	///////////END////////////
/*
	if(startP.b<1000)
		startLongId=(startP.a*1000)+startP.b;
	else
		startLongId=(startP.a*10000)+startP.b;
	//voronoiPaths->ids.push_back(startLongId);
	startId=voronoiPaths->ids.size()-2;
	
	std::cout << "startID " << startId << " | startLongID " << startLongId << std::endl;
	
	if(endP.b<1000)
		endLongId=(endP.a*1000)+endP.b;
	else
		endLongId=(endP.a*10000)+endP.b;
	//voronoiPaths->ids.push_back(endLongId);
	endId=voronoiPaths->ids.size()-1;

	std::cout << "endID " << endId << " | endLongID " << endLongId << std::endl;

	for(int i=0;i<voronoiPaths->resultEdges.size();i++)
	{
		x=voronoiPaths->resultEdges[i].p0.a;
		y=voronoiPaths->resultEdges[i].p0.b;
		
		if(startOk && (a<x) && (x<(a+o)) && ((b<y) && (y<(b+o))))
		{
			len=sqrt(pow((x-startP.a),2)+pow((y-startP.b),2));
			if(x!=prev_x || y!=prev_y)
			{
				if(y<1000)
					longId=(x*1000)+y;
				else
					longId=(x*10000)+y;
				std::cout << "longId " << longId << std::endl;
				id1=-1;
				for(int j=0;j<voronoiPaths->ids.size() && id1==-1;j++)
					if(voronoiPaths->ids[j]==longId)
						id1=j;
				std::cout << "id1 " << id1 << std::endl;
				GraphEdge e(x,y,startP.a,startP.b,len,id1,startId);
				start->push_back(e);
				prev_x=x;
				prev_y=y;
			}
		}
		if(endOk && (c<x) && (x<(c+o)) && ((d<y) && (y<(d+o))))
		{
			len=sqrt(pow((x-endP.a),2)+pow((y-endP.b),2));
			if(x!=prev_x || y!=prev_y)
			{
				if(y<1000)
					longId=(x*1000)+y;
				else
					longId=(x*10000)+y;
				std::cout << "longId " << longId << std::endl;
				id1=-1;
				for(int k=0;k<voronoiPaths->ids.size() && id1==-1;k++)
					if(voronoiPaths->ids[k]==longId)
						id1=k;
				std::cout << "id1 " << id1 << std::endl;
				GraphEdge e(x,y,endP.a,endP.b,len,id1,endId);
				end->push_back(e);
				prev_x=x;
				prev_y=y;
			}
		}
	}
 	//return id;
	*/
}

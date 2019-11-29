#include "PathFinder.h"
#include "Dijkstra.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <stdlib.h> 
#include <limits.h> 
#include <stdio.h> 

void connect(int, VoronoiPoint, VoronoiPoint, VoronoiResults*, bool, bool, std::vector<GraphEdge>*, std::vector<GraphEdge>*);
void shortestPath(VoronoiPoint, VoronoiPoint, VoronoiResults *, std::vector<VoronoiPoint> *);

void PathFinder(VoronoiPoint start,VoronoiPoint end, VoronoiResults *voronoiPaths, std::vector<VoronoiPoint> *rightPath)
{
	int offset=20;
	int step=10;
	int nConnection=5;
	int startSize;
	int endSize;
	int startLongId;
	int endLongId;
	int lessId,mostId;
	std::vector<GraphEdge> start_connection, end_connection;

	connect(offset, start, end, voronoiPaths, true, true, &start_connection, &end_connection);
	std::cout << "Step " << offset << std::endl;
	startSize=start_connection.size();
	endSize=end_connection.size();

	mostId=(int)(start.a*1000);
	lessId=(int)(start.b*1000);
	if(start.b<1)
		startLongId=(mostId*1000)+lessId;
	else
		startLongId=(mostId*10000)+lessId;
	voronoiPaths->ids.push_back(startLongId);
	
	mostId=(int)(end.a*1000);
	lessId=(int)(end.b*1000);
	if(end.b<1)
		endLongId=(mostId*1000)+lessId;
	else
		endLongId=(mostId*10000)+lessId;
	voronoiPaths->ids.push_back(endLongId);
	
	std::cout << "startsize" << startSize << std::endl;

	while(startSize<=nConnection || endSize<=nConnection)
	{
		offset=offset+step;
		std::cout << "Step " << offset << "size: " << startSize << " " << endSize <<std::endl;
		if(startSize<=nConnection)
			start_connection.clear();
		else
			end_connection.clear();

		connect(offset, start, end, voronoiPaths, startSize<=nConnection, endSize<=nConnection, &start_connection, &end_connection);
		startSize=start_connection.size();
		endSize=end_connection.size();
	}
	
	for(int i=0;i<start_connection.size();i++)
		voronoiPaths->resultEdges.push_back(start_connection[i]);
	for(int i=0;i<end_connection.size();i++)
		voronoiPaths->resultEdges.push_back(end_connection[i]);
	
	shortestPath(start, end, voronoiPaths, rightPath);
}

void connect(int offset, VoronoiPoint startP, VoronoiPoint endP, VoronoiResults *voronoiPaths, bool startOk, bool endOk, std::vector<GraphEdge> *start, std::vector<GraphEdge> *end)
{
	int a,b,c,d,o,x,y,endId,startId,id1;
	int prev_x,prev_y=-1;
	double len;
	int longId,startLongId,endLongId,mostId,lessId;

	a=startP.a-offset;
	b=startP.b-offset;
	c=endP.a-offset;
	d=endP.b-offset;
	o=offset*2;

	mostId=(int)(startP.a*1000);
	lessId=(int)(startP.b*1000);
	if(startP.b<1)
		startLongId=(mostId*1000)+lessId;
	else
		startLongId=(mostId*10000)+lessId;
	//voronoiPaths->ids.push_back(startLongId);
	startId=voronoiPaths->ids.size()-2;
	
	std::cout << "startID " << startId << " | startLongID " << startLongId << std::endl;
	
	mostId=(int)(endP.a*1000);
	lessId=(int)(endP.b*1000);
	if(endP.b<1)
		endLongId=(mostId*1000)+lessId;
	else
		endLongId=(mostId*10000)+lessId;
	//voronoiPaths->ids.push_back(endLongId);
	endId=voronoiPaths->ids.size()-1;

	std::cout << "endID " << endId << " | endLongID " << endLongId << std::endl;

	std::cout << "voronoiPaths->resultEdges.size() " << voronoiPaths->resultEdges.size()  << std::endl;

	for(int i=0;i<voronoiPaths->resultEdges.size();i++)
	{
		x=voronoiPaths->resultEdges[i].p0.a;
		y=voronoiPaths->resultEdges[i].p0.b;
		
		if(startOk && (a<x) && (x<(a+o)) && ((b<y) && (y<(b+o))))
		{
			len=sqrt(pow((x-startP.a),2)+pow((y-startP.b),2));
			if(x!=prev_x || y!=prev_y)
			{
				mostId=(int)(x*1000);
				lessId=(int)(y*1000);
				if(y<1)
					longId=(mostId*1000)+lessId;
				else
					longId=(mostId*10000)+lessId;
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
				mostId=(int)(x*1000);
				lessId=(int)(y*1000);
				if(y<1)
					longId=(mostId*1000)+lessId;
				else
					longId=(mostId*10000)+lessId;
				//std::cout << "longId " << longId << std::endl;
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



void shortestPath(VoronoiPoint start,VoronoiPoint end, VoronoiResults *voronoiPaths, std::vector<VoronoiPoint> *rightPath)
{
	int V = voronoiPaths->ids.size(); 
	std::cout << "V " << V << std::endl;
	int k=0;
	int idNode1,idNode2=0;
	int path[V];
	int mapPosition[V];
	int encodedCoord,x,y;
	int encoder,mostId,lessId;
	Graph* graph = createGraph(V); 
	for(int i=0;i<voronoiPaths->resultEdges.size();i++)
	{
		std::cout << voronoiPaths->resultEdges[i].idFirstNode << " ; " << voronoiPaths->resultEdges[i].idSecondNode << "  ("<< voronoiPaths->resultEdges[i].p0.a << "," << voronoiPaths->resultEdges[i].p0.b << ")" << "  ("<< voronoiPaths->resultEdges[i].p1.a << "," << voronoiPaths->resultEdges[i].p1.b << ")  => " << voronoiPaths->resultEdges[i].length << std::endl;
			
		x=voronoiPaths->resultEdges[i].p0.a;
		y=voronoiPaths->resultEdges[i].p0.b;
		mostId=(int)(x*1000);
		lessId=(int)(y*1000);
		if(y<1)
			encodedCoord=(mostId*1000)+lessId;
		else
			encodedCoord=(mostId*10000)+lessId;
		
		mapPosition[voronoiPaths->resultEdges[i].idFirstNode]=encodedCoord;
		std::cout << "id " << voronoiPaths->resultEdges[i].idFirstNode << " coo " << encodedCoord << " coo2 " << mapPosition[voronoiPaths->resultEdges[i].idFirstNode] <<  std::endl; 
		addEdge(graph, voronoiPaths->resultEdges[i].idFirstNode, voronoiPaths->resultEdges[i].idSecondNode, voronoiPaths->resultEdges[i].length); 
		
	}

	dijkstra(graph,voronoiPaths->ids.size()-2,voronoiPaths->ids.size()-1,path); 
	
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







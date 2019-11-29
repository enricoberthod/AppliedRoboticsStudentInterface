
#ifndef Dijkstra_H
#define Dijkstra_H
#include <stdlib.h>
#include <vector>

// A structure to represent a node in adjacency list 
struct AdjListNode 
{ 
    int dest; 
    int weight; 
    struct AdjListNode* next; 
}; 
  
// A structure to represent an adjacency liat 
struct AdjList 
{ 
    struct AdjListNode *head;  // pointer to head node of list 
}; 
  
// A structure to represent a graph. A graph is an array of adjacency lists. 
// Size of array will be V (number of vertices in graph) 
struct Graph 
{ 
    int V; 
    struct AdjList* array; 
}; 
  

void dijkstra(struct Graph* graph, int src, int dest, int path[]);
void addEdge(struct Graph* graph, int src, int dest, int weight);
void storePath(int path[],int j,std::vector<int> startEndPath[]);
struct Graph* createGraph(int V);

#endif


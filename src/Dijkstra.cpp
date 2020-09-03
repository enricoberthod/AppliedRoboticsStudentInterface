// C / C++ program for Dijkstra's shortest path algorithm for adjacency, thanks to https://www.geeksforgeeks.org/

#include "Dijkstra.h"
#include <stdio.h>
#include <limits.h>

/* function newAdjListNode: utility function to create a new adjacency list node 
   -parameters:
   	dest: destination node
	wejght: weight of the edge
    -return: the new AdjListNode 
*/
struct AdjListNode* newAdjListNode(int dest, int weight) 
{ 
    struct AdjListNode* newNode = 
            (struct AdjListNode*) malloc(sizeof(struct AdjListNode)); 
    newNode->dest = dest; 
    newNode->weight = weight; 
    newNode->next = NULL; 
    return newNode; 
} 
  

/* function createGraph: utility function that creates a graph of V vertices 
   -parameters:
   	V: number of vertices of the graph
    -return: the new graph
*/
struct Graph* createGraph(int V) 
{ 
    struct Graph* graph = (struct Graph*) malloc(sizeof(struct Graph)); 
    graph->V = V; 
  
    //create an array of adjacency lists. Size of array will be V 
    graph->array = (struct AdjList*) malloc(V * sizeof(struct AdjList)); 
  
    //initialize each adjacency list as empty by making head as NULL 
    for (int i = 0; i < V; ++i) 
        graph->array[i].head = NULL; 
  
    return graph; 
};
 

/* function addEdge: adds an edge to an undirected graph 
   -parameters:
   	graph: the graph
	src: source 
	dest: destination
	weight: edge weight from src to dest	
    -return: none
*/
void addEdge(struct Graph* graph, int src, int dest, int weight) 
{ 
    //add an edge from src to dest.  A new node is added to the adjacency list of src.  The node is added at the begining 
    struct AdjListNode* newNode = newAdjListNode(dest, weight); 
    newNode->next = graph->array[src].head; 
    graph->array[src].head = newNode; 
  
    //since graph is undirected, add an edge from dest to src also 
    newNode = newAdjListNode(src, weight); 
    newNode->next = graph->array[dest].head; 
    graph->array[dest].head = newNode; 
} 
  
//structure to represent a min heap node 
struct MinHeapNode 
{ 
    int  v; 
    int dist; 
}; 
  
//structure to represent a min heap 
struct MinHeap 
{ 
    int size;      //number of heap nodes present currently 
    int capacity;  //capacity of min heap 
    int *pos;     // this is needed for decreaseKey() 
    struct MinHeapNode **array; 
}; 
  

/* function MinHeapNode: utility function to create a new Min Heap Node 
   -parameters:
   	v: the vertex
	dist: distance
    -return: minHeapNode
*/
struct MinHeapNode* newMinHeapNode(int v, int dist) 
{ 
    struct MinHeapNode* minHeapNode = 
           (struct MinHeapNode*) malloc(sizeof(struct MinHeapNode)); 
    minHeapNode->v = v; 
    minHeapNode->dist = dist; 
    return minHeapNode; 
} 
  

/* function createMinHeap: utility function to create a Min Heap 
   -parameters:
   	capacity: the cpacity
    -return: minHeap
*/
struct MinHeap* createMinHeap(int capacity) 
{ 
    struct MinHeap* minHeap = 
         (struct MinHeap*) malloc(sizeof(struct MinHeap)); 
    minHeap->pos = (int *)malloc(capacity * sizeof(int)); 
    minHeap->size = 0; 
    minHeap->capacity = capacity; 
    minHeap->array = 
         (struct MinHeapNode**) malloc(capacity * sizeof(struct MinHeapNode*)); 
    return minHeap; 
} 
  

/* function swapMinHeapNode: utility function to swap two nodes of min heap. Needed for min heapify 
   -parameters:
   	a: first MinHeapNode
	b: second MinHeapNode
    -return: minHeap
*/
void swapMinHeapNode(struct MinHeapNode** a, struct MinHeapNode** b) 
{ 
    struct MinHeapNode* t = *a; 
    *a = *b; 
    *b = t; 
} 
  
// A standard function to heapify at given idx 
// This function also updates position of nodes when they are swapped. 
// Position is needed for decreaseKey() 
void minHeapify(struct MinHeap* minHeap, int idx) 
{ 
    int smallest, left, right; 
    smallest = idx; 
    left = 2 * idx + 1; 
    right = 2 * idx + 2; 
  
    if (left < minHeap->size && 
        minHeap->array[left]->dist < minHeap->array[smallest]->dist ) 
      smallest = left; 
  
    if (right < minHeap->size && 
        minHeap->array[right]->dist < minHeap->array[smallest]->dist ) 
      smallest = right; 
  
    if (smallest != idx) 
    { 
        //the nodes to be swapped in min heap 
        MinHeapNode *smallestNode = minHeap->array[smallest]; 
        MinHeapNode *idxNode = minHeap->array[idx]; 
  
        //swap positions 
        minHeap->pos[smallestNode->v] = idx; 
        minHeap->pos[idxNode->v] = smallest; 
  
        //swap nodes 
        swapMinHeapNode(&minHeap->array[smallest], &minHeap->array[idx]); 
  
        minHeapify(minHeap, smallest); 
    } 
} 
  
// A utility function to check if the given minHeap is ampty or not 
int isEmpty(struct MinHeap* minHeap) 
{ 
    return minHeap->size == 0; 
} 
  
// Standard function to extract minimum node from heap 
struct MinHeapNode* extractMin(struct MinHeap* minHeap) 
{ 
    if (isEmpty(minHeap)) 
        return NULL; 
  
    // Store the root node 
    struct MinHeapNode* root = minHeap->array[0]; 
  
    // Replace root node with last node 
    struct MinHeapNode* lastNode = minHeap->array[minHeap->size - 1]; 
    minHeap->array[0] = lastNode; 
  
    // Update position of last node 
    minHeap->pos[root->v] = minHeap->size-1; 
    minHeap->pos[lastNode->v] = 0; 
  
    // Reduce heap size and heapify root 
    --minHeap->size; 
    minHeapify(minHeap, 0); 
  
    return root; 
} 
  
// Function to decreasy dist value of a given vertex v. This function 
// uses pos[] of min heap to get the current index of node in min heap 
void decreaseKey(struct MinHeap* minHeap, int v, int dist) 
{ 
    // Get the index of v in  heap array 
    int i = minHeap->pos[v]; 
  
    // Get the node and update its dist value 
    minHeap->array[i]->dist = dist; 
  
    // Travel up while the complete tree is not hepified. 
    // This is a O(Logn) loop 
    while (i && minHeap->array[i]->dist < minHeap->array[(i - 1) / 2]->dist) 
    { 
        // Swap this node with its parent 
        minHeap->pos[minHeap->array[i]->v] = (i-1)/2; 
        minHeap->pos[minHeap->array[(i-1)/2]->v] = i; 
        swapMinHeapNode(&minHeap->array[i],  &minHeap->array[(i - 1) / 2]); 
  
        // move to parent index 
        i = (i - 1) / 2; 
    } 
} 
  
// A utility function to check if a given vertex 
// 'v' is in min heap or not 
bool isInMinHeap(struct MinHeap *minHeap, int v) 
{ 
   if (minHeap->pos[v] < minHeap->size) 
     return true; 
   return false; 
} 
  
/* function storePath: function that stores the best path start to end using the paths computed in Dijkstra
   -parameters:
   	path: the path computed by Dijkstra 
	j: the destination node
	startEndPath: the path from start to end
    -return: none
*/
void storePath(int path[],int j,std::vector<int> startEndPath[])
{
	if(path[j]==-1)
		return;
	storePath(path,path[j],startEndPath);
	startEndPath->push_back(j);
}
  

/* function dijkstra: function that calulates distances of shortest paths from src to all vertices 
   -parameters:
   	graph: the graph representing the road map 
	src: source node
	dest: destination node
	path: array where to store the solution of the Dijkstra algorithm
	j: the destination node
	startEndPath: the path from start to end
    -return: none
*/
// The main function that calulates distances of shortest paths from src to all 
// vertices. It is a O(ELogV) function 
void dijkstra(struct Graph* graph, int src, int dest, int path[]) 
{ 
    int V = graph->V; //get the number of vertices in graph 
    int dist[V];      //dist values used to pick minimum weight edge in cut 
  
    //minHeap represents set E 
    struct MinHeap* minHeap = createMinHeap(V); 
  
    //initialize min heap with all vertices. dist value of all vertices  
    for (int v = 0; v < V; ++v) 
    { 
        dist[v] = INT_MAX; 
        minHeap->array[v] = newMinHeapNode(v, dist[v]); 
        minHeap->pos[v] = v; 
	 	path[v]=-1;
    } 
  
    //make dist value of src vertex as 0 so that it is extracted first 
    minHeap->array[src] = newMinHeapNode(src, dist[src]); 
    minHeap->pos[src]   = src; 
    dist[src] = 0; 
    decreaseKey(minHeap, src, dist[src]); 
  
    //initially size of min heap is equal to V 
    minHeap->size = V; 
  
    //in the followin loop, min heap contains all nodes whose shortest distance is not yet finalized. 
    while (!isEmpty(minHeap)) 
    { 
        //extract the vertex with minimum distance value 
        struct MinHeapNode* minHeapNode = extractMin(minHeap); 
        int u = minHeapNode->v; // Store the extracted vertex number 
  
        //traverse through all adjacent vertices of u (the extracted vertex) and update their distance values 
        struct AdjListNode* pCrawl = graph->array[u].head; 
        while (pCrawl != NULL) 
        { 
            int v = pCrawl->dest; 

            //if shortest distance to v is not finalized yet, and distance to v through u is less than its previously calculated distance 
            if (isInMinHeap(minHeap, v) && dist[u] != INT_MAX && pCrawl->weight + dist[u] < dist[v]) 
            { 
                dist[v] = dist[u] + pCrawl->weight; 
  		path[v]=u;
                //update distance value in min heap also 
                decreaseKey(minHeap, v, dist[v]); 
            } 
            pCrawl = pCrawl->next; 
        } 
    } 
} 

%%cu
#include<bits/stdc++.h>
#include <thrust/device_vector.h>
#include <thrust/device_ptr.h>
#include <thrust/sort.h>
using namespace std;
# define INF 0x3f3f3f3f
void printGrpah( int *a, int *e, int *b, int *c, int *d, int num_v, int num_e ) 
{
    for(int v = 0; v<num_v; v++)
    {
        if(a[v] != -1)
        {
           int start = a[v];
            int end = e[v];
            printf("%d-->", v);
            while( start <= end )
            {
                printf("[%d, {%d, %d} ]---", b[start], c[start], d[start]);
                start++;
            }
        }
        printf("\n");
    }
    return;
}

/************************Host Dijkstras Algorithm for Intitial Seed Path Start ***********************************/

vector<pair<int, pair<int,int>>> seedPathDijkstra(int src, int dest, int* h_offset, int* h_offend, int* h_earr, int* h_carr, int* h_sarr, int num_v, int num_e) 
{ 
	priority_queue< pair<int, int>, vector <pair<int, int>> , greater <pair<int, int>> > pq;
    int V = num_v;
	vector<int> dist(V, INF);
    vector<pair<int, pair<int,int>>> parent(V, {-1, {-1,-1}});
    vector<pair<int, pair<int,int>>> seedPath;

	pq.push(make_pair(0, src));
	dist[src] = 0;
	
	vector<bool> f(V, false);

	while (!pq.empty())
	{
		int u = pq.top().second;
		pq.pop();
		f[u] = true;

        int index = h_offset[u];
        int end = h_offend[u];

		for (int i=index; i <=end; i++)
		{
            int v = h_earr[i];
            int cost = h_carr[i];
            int score = h_sarr[i];

			// If there is shorted path to v through u.
			if (f[v] == false && dist[v] > dist[u] + cost)
			{
				// Updating distance of v
				dist[v] = dist[u] + cost;
				pq.push(make_pair(dist[v], v));
                pair<int, pair<int,int>> temp;
                temp.first = u;
                temp.second.first = cost;
                temp.second.second = score;
                parent[v] = temp;
			}
		}
	}

    //Find shortest path from src to some dest
    stack<pair<int, pair<int,int>>> path;
    int nd = dest;
    while(parent[nd].first != -1)
    {
        path.push(parent[nd]);
        nd = parent[nd].first;
    }

    while(!path.empty())
    {
        seedPath.push_back(path.top());
        path.pop();
    }
    seedPath.push_back({dest, {0,0}});

    int sz_p = seedPath.size();
    for(int i=sz_p-1; i>0; i--)
    {
        seedPath[i].second.first = seedPath[i-1].second.first;
        seedPath[i].second.second = seedPath[i-1].second.second;        
    }
    seedPath[0].second.first = 0;
    seedPath[0].second.second = 0;

    for(int i=1; i<sz_p; i++)
    {
        seedPath[i].second.first += seedPath[i-1].second.first;
        seedPath[i].second.second += seedPath[i-1].second.second;        
    }
        

    return seedPath;      
} 
/********************* Dijkstra seed PAth end*******************************************/

/**************************** Kernel code*********/

__device__ int minDistance(int* dist,int* sptSet, int num_v) 
{ 
    int min = 100000000, min_index; 
    for (int v = 0; v < num_v; v++) 
        if (sptSet[v] == 0 && dist[v] <= min) 
        {    min = dist[v];
            min_index = v;
        } 
  
    return min_index; 
} 

__device__ int EucliDist(int src, int dest, int* dev_offset, int* dev_offend, int* dev_earr, int* dev_carr, int num_v, int num_e) 
{
    int dist[11];
    int sptSet[11];
 
    for(int i=0; i<num_v; i++)
    {
        dist[i] = 100000000;
        sptSet[i] = 0;
    }

    dist[src] = 0; 
  
    for (int count = 0; count < num_v - 1; count++) { 

        int u = minDistance(dist, sptSet, num_v); 
        sptSet[u] = 1; 
        int index = dev_offset[u];
        int end = dev_offend[u];
  
        // Update dist value of the adjacent vertices of the picked vertex. 
        for (int i = index; i <= end; i++) 
        {    
            int v = dev_earr[i]; //adjacent vertex of u
            int cost = dev_carr[i];
            if (!sptSet[v] && dist[u] != 100000000 
                && dist[u] + cost < dist[v]) 
                dist[v] = dist[u] + cost; 
        }
    } 
    return dist[dest];
} 
__device__ struct BestVertex{
    int vertex;
    float gamma;
    int score;
    int cost;
};

__device__ float GammaValue(int cost, int score, int De)
{
    float g = float(1 + score)/float(cost + De);   
    return g;
}

/* Find Best Successor */
__device__ struct BestVertex BestSuccessor(int* dev_offset, int* dev_offend,int* dev_earr,int* dev_carr,int* dev_sarr,int num_v,int num_e, int front_tail, int back_tail )
{
    struct BestVertex VGCS;   // best successor struct
    float gamma = -1;
 
    int start_index = dev_offset[front_tail] ;
    int end_index = dev_offend[front_tail];
	for (int i=start_index; i <= end_index; i++)  //for all adjacent outgoing edge
	{
        int v = dev_earr[i];
        int cost = dev_carr[i];
        int score = dev_sarr[i];
  
        int De = EucliDist(v, back_tail, dev_offset,dev_offend, dev_earr, dev_carr, num_v, num_e);
        //int De = 1;
        float g = GammaValue(cost, score, De);
        
        if( g > gamma )
        {
            gamma = g;
            VGCS.vertex = v;
            VGCS.gamma = g;
            VGCS.score = score;
            VGCS.cost = cost;
        }
    }
    return VGCS;
}

/* Find Best Successor */
__device__ struct BestVertex BestPredecessor(int* dev_offset,int* dev_offend,int* dev_earr,int* dev_carr,int* dev_Ioffset, int* dev_Ioffend,int* dev_Iearr,int* dev_Icarr,int* dev_Isarr, int num_v,int num_e, int front_tail, int back_tail )
{
    struct BestVertex VGCS;   // best successor struct
    float gamma = -1;
 
    int start_index = dev_Ioffset[back_tail] ;
    int end_index = dev_Ioffend[back_tail];
	for (int i=start_index; i <= end_index; i++)  //for all adjacent incoming edge
	{
        int v = dev_Iearr[i];
        int cost = dev_Icarr[i];
        int score = dev_Isarr[i];
  
        int De = EucliDist(front_tail, v, dev_offset,dev_offend, dev_earr, dev_carr, num_v, num_e);
        //int De = 1;
        float g = GammaValue(cost, score, De);
        
        if( g > gamma )
        {
            gamma = g;
            VGCS.vertex = v;
            VGCS.gamma = g;
            VGCS.score = score;
            VGCS.cost = cost;
        }
    }
    return VGCS;
}

__global__ void display(int pathSize, int* d_seedarr,int* dev_offset, int* dev_offend,int* dev_earr,int* dev_carr,int* dev_sarr,
                        int num_v,int num_e,int* dev_Ioffset, int* dev_Ioffend,int* dev_Iearr,int* dev_Icarr,int* dev_Isarr) 
{
    int i = EucliDist(0, 10, dev_offset, dev_offend, dev_earr, dev_carr, num_v, num_e);
    float gamma = GammaValue(2,1,2);
    struct BestVertex Succ = BestSuccessor(dev_offset,dev_offend,dev_earr,dev_carr,dev_sarr,num_v,num_e, 1, 5);
    struct BestVertex Pred = BestPredecessor(dev_offset,dev_offend,dev_earr,dev_carr,dev_Ioffset,dev_Ioffend,dev_Iearr,dev_Icarr,dev_Isarr,num_v,num_e, 1, 5);
    printf("-----%d-----%d--------%f-------%d\n", i, Succ.score, gamma, Succ.vertex);
    printf("-Pred--sc--%d---v--%d------gm--%f-----cost--%d", Pred.score, Pred.vertex,Pred.gamma,Pred.cost);
    return;
}
/****************************************************/
int main()
{
/****************************Graph Initialisation in host and device
*************************************************************************************/  
    cout<<"hello";
    int num_v = 11;
    int num_e = 10;   
//Graph for storing outgoing edges
    int h_offset[num_v] = {0,1,4,5,6,7,-1,8,9,-1,-1};
    int h_offend[num_v] = {0,3,4,5,6,7,-1,8,9,-1,-1};     
    int h_earr[num_e] = {1,2,7,3,5,4,5,10,8,5};
    int h_carr[num_e] = {2,5,6,16,7,17,4,3,9,3}; 
    int h_sarr[num_e] = {1,3,4,10,0,15,2,5,15,0};    //variables declared for representing graph in cpu
    int *dev_offset, *dev_offend, *dev_earr, *dev_carr, *dev_sarr;  
//Graph for storing incoming edges
    int h_Ioffset[num_v] = {-1,0,1,2,3,4,-1,7,8,-1,9};
    int h_Ioffend[num_v] = {-1,0,1,2,3,6,-1,7,8,-1,9};     
    int h_Iearr[num_e] = {0,1,1,3,4,2,8,1,7,5};
    int h_Icarr[num_e] = {2,5,16,17,4,7,3,6,9,3}; 
    int h_Isarr[num_e] = {1,3,10,15,2,0,0,4,15,5};    //variables declared for representing graph in cpu
    int *dev_Ioffset, *dev_Ioffend, *dev_Iearr, *dev_Icarr, *dev_Isarr;  

    cudaMalloc( (void**)&dev_offset, num_v*sizeof(int) ) ;
    cudaMalloc( (void**)&dev_offend, num_v*sizeof(int) ) ;    
    cudaMalloc( (void**)&dev_earr, num_e*sizeof(int) ) ;
    cudaMalloc( (void**)&dev_carr, num_e*sizeof(int) ) ;
    cudaMalloc( (void**)&dev_sarr, num_e*sizeof(int) ) ; 
    cudaMalloc( (void**)&dev_Ioffset, num_v*sizeof(int) ) ;
    cudaMalloc( (void**)&dev_Ioffend, num_v*sizeof(int) ) ;    
    cudaMalloc( (void**)&dev_Iearr, num_e*sizeof(int) ) ;
    cudaMalloc( (void**)&dev_Icarr, num_e*sizeof(int) ) ;
    cudaMalloc( (void**)&dev_Isarr, num_e*sizeof(int) ) ; 
    cudaDeviceSynchronize();
    cudaMemcpy( dev_offset, h_offset, num_v*sizeof(int), cudaMemcpyHostToDevice ) ;
    cudaMemcpy( dev_offend, h_offend, num_v*sizeof(int), cudaMemcpyHostToDevice ) ;    
    cudaMemcpy( dev_earr, h_earr, num_e*sizeof(int), cudaMemcpyHostToDevice ) ; 
    cudaMemcpy( dev_carr, h_carr, num_e*sizeof(int), cudaMemcpyHostToDevice ) ;
    cudaMemcpy( dev_sarr, h_sarr, num_e*sizeof(int), cudaMemcpyHostToDevice ) ;
    cudaMemcpy( dev_Ioffset, h_Ioffset, num_v*sizeof(int), cudaMemcpyHostToDevice ) ;
    cudaMemcpy( dev_Ioffend, h_Ioffend, num_v*sizeof(int), cudaMemcpyHostToDevice ) ;    
    cudaMemcpy( dev_Iearr, h_Iearr, num_e*sizeof(int), cudaMemcpyHostToDevice ) ; 
    cudaMemcpy( dev_Icarr, h_Icarr, num_e*sizeof(int), cudaMemcpyHostToDevice ) ;
    cudaMemcpy( dev_Isarr, h_Isarr, num_e*sizeof(int), cudaMemcpyHostToDevice ) ;
    cudaDeviceSynchronize();

/****************************Seed Path find, copy to device
*************************************************************************************/    

    //find intial sead path
    vector<pair<int, pair<int,int>>> seedPath;
    seedPath = seedPathDijkstra( 0, 10, h_offset, h_offend,h_earr, h_carr, h_sarr, num_v, num_e);
    
    int pathSize = seedPath.size();
    int h_seedarr[3][pathSize];    
    
    cout<<"seedPath---";
    int iseed=0;
    for(auto v : seedPath)
    {
        cout<<"("<<v.first<<", "<<"("<<v.second.first<<","<<v.second.second<<")"<<endl;
        h_seedarr[0][iseed] = v.first;
        h_seedarr[1][iseed] = v.second.first;
        h_seedarr[2][iseed] = v.second.second;
        iseed++;
    }
 
 //alloate memory to device and copy
    int *d_seedarr;
    cudaMalloc((void **)&d_seedarr,3*pathSize*sizeof(int));
    cudaMemcpy( d_seedarr, h_seedarr, 3*pathSize*sizeof(int), cudaMemcpyHostToDevice ) ;
    cudaDeviceSynchronize();

/******************************************************************************************************/
 //Kernel call to check functionalities
  display<<<1,1>>>( pathSize, d_seedarr, dev_offset, dev_offend,dev_earr, dev_carr, dev_sarr, num_v, num_e, dev_Ioffset, dev_Ioffend,dev_Iearr, dev_Icarr, dev_Isarr );
    cudaDeviceSynchronize();
/*****************************************************************************************************/ 
    cudaFree(dev_offset);
    cudaFree(dev_offend); 
    cudaFree(dev_earr);
    cudaFree(dev_carr);
    cudaFree(dev_sarr);
    cudaFree(d_seedarr); 
    cudaFree(dev_Ioffset);
    cudaFree(dev_Ioffend); 
    cudaFree(dev_Iearr);
    cudaFree(dev_Icarr);
    cudaFree(dev_Isarr);
    cudaDeviceSynchronize();
    return 0;

}
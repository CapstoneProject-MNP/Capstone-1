%%cu
#include<bits/stdc++.h>
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

/************************ Dijkstras Algorithm for Intitial Seed Path Start ***********************************/
  
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

/**************************** Kernel test code*********/

__global__ void display(int pathSize, int* d_seedarr,int* dev_offset, int* dev_offend,int* dev_earr,int* dev_carr,int* dev_sarr,int num_v,int num_e) 
{
    return;
}
/****************************************************/

int main()
{

/****************************Graph Initialisation in host and device
*************************************************************************************/    
    int num_v = 11;
    int num_e = 10;   

    int h_offset[num_v] = {0,1,4,5,6,7,-1,8,9,-1,-1};
    int h_offend[num_v] = {0,3,4,5,6,7,-1,8,9,-1,-1};     
    int h_earr[num_e] = {1,2,7,3,5,4,5,10,8,5};
    int h_carr[num_e] = {2,5,6,16,7,17,4,3,9,3}; 
    int h_sarr[num_e] = {1,3,4,10,0,15,2,5,15,0};    //variables declared for representing graph in cpu
    int *dev_offset, *dev_offend, *dev_earr, *dev_carr, *dev_sarr;  

    cudaMalloc( (void**)&dev_offset, num_v*sizeof(int) ) ;
    cudaMalloc( (void**)&dev_offend, num_v*sizeof(int) ) ;    
    cudaMalloc( (void**)&dev_earr, num_e*sizeof(int) ) ;
    cudaMalloc( (void**)&dev_carr, num_e*sizeof(int) ) ;
    cudaMalloc( (void**)&dev_sarr, num_e*sizeof(int) ) ; 

    cudaMemcpy( dev_offset, h_offset, num_v*sizeof(int), cudaMemcpyHostToDevice ) ;
    cudaMemcpy( dev_offend, h_offend, num_v*sizeof(int), cudaMemcpyHostToDevice ) ;    
    cudaMemcpy( dev_earr, h_earr, num_e*sizeof(int), cudaMemcpyHostToDevice ) ; 
    cudaMemcpy( dev_carr, h_carr, num_e*sizeof(int), cudaMemcpyHostToDevice ) ;
    cudaMemcpy( dev_sarr, h_sarr, num_e*sizeof(int), cudaMemcpyHostToDevice ) ;

    printGrpah(h_offset, h_offend,h_earr, h_carr, h_sarr, num_v, num_e);

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


/******************************************************************************************************/
 //Kernel call to display global device memory
   display<<<1,1>>>( pathSize, d_seedarr, dev_offset, dev_offend,dev_earr, dev_carr, dev_sarr, num_v, num_e );


/*****************************************************************************************************/ 
    cudaFree(dev_offset);
    cudaFree(dev_offend); 
    cudaFree(dev_earr);
    cudaFree(dev_carr);
    cudaFree(dev_sarr);
    cudaFree(d_seedarr); 
 
    return 0;

}
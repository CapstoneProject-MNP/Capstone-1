#include<iostream>
#include<fstream>
#include<unordered_map>
#include<vector>
#include<climits>
#include <sstream>
using namespace std;

/********Graph Definition Start******************************************************/
#define V 9
//graph[i] contains list of (adjacent_node to i,(cost, score))
vector<vector<pair<int, pair<int,int>>>> graph;


void DataPreprocessing(string node_file, string edge_file)
{
    unordered_map<string,int>map_nodes;
    ifstream nodeFile;
	string line;
	long int count_nodes = 0;

	nodeFile.open(node_file);
	
    //map unique serial number to each node
	while (nodeFile >> line)
	{
        count_nodes++;
        map_nodes[line] = count_nodes;
	}
	
	nodeFile.close();

    ifstream edgeFile;
    edgeFile.open(edge_file);

    //Build Graph
    graph.resize(count_nodes+1);
    while(edgeFile>>line)
    {
        //split the line by the delimiter ','
        vector<string>v;
        stringstream ss(line);
        while(ss.good())
        {
            string substr;
            getline(ss, substr, ',');
            v.push_back(substr);
        }

        //Extract the nodes, score and cost

        if((map_nodes.find(v[0]) != map_nodes.end()) && (map_nodes.find(v[1]) != map_nodes.end())){
            graph[map_nodes[v[0]]].push_back(make_pair(map_nodes[v[1]],make_pair(stoi(v[2]),stoi(v[3]))));
        }
        else{
            cout<<"Invalid input";
            exit(1);
        }
    }
}


/************************ Dijkstras Algorithm Start ***********************************/

int minDistance(int dist[], bool sptSet[]) 
{ 
    // Initialize min value 
    int min = INT_MAX, min_index; 
  
    for (int v = 0; v < V; v++) 
    {    
        if (sptSet[v] == false && dist[v] <= min) 
        {    
            min = dist[v], min_index = v; 
        }
    }
    return min_index; 
} 
  
// Function to print shortest path from source to j using parent array 
void printPath(int parent[], int j) 
{ 
    // Base Case : If j is source 
    if (parent[j] == - 1) 
        return; 
  
    printPath(parent, parent[j]); 
  
    printf("%d ", j); 
} 
  
// A utility function to print the constructed distance array 
int printSolution(int dist[], int n, int parent[]) 
{ 
    int src = 0; 
    printf("Vertex\t Distance\tPath"); 
    for (int i = 1; i < V; i++) 
    { 
        printf("\n%d -> %d \t\t %d\t\t%d ", src, i, dist[i], src); 
        printPath(parent, i); 
    } 
 return 0;
} 
  
// Funtion that implements Dijkstra single source shortest path 
// algorithm for a graph represented 
// using adjacency matrix representation 
void dijkstra(int src) 
{ 
      
    // The output array. dist[i] 
    // will hold the shortest 
    // distance from src to i 
    int dist[V];  
  
    // sptSet[i] will true if vertex 
    // i is included / in shortest 
    // path tree or shortest distance  
    // from src to i is finalized 
    bool sptSet[V]; 
  
    // Parent array to store 
    // shortest path tree 
    int parent[V]; 
  
    // Initialize all distances as  
    // INFINITE and stpSet[] as false 
    for (int i = 0; i < V; i++) 
    { 
        parent[0] = -1; 
        dist[i] = INT_MAX; 
        sptSet[i] = false; 
    } 
  
    // Distance of source vertex  
    // from itself is always 0 
    dist[src] = 0; 
  
    // Find shortest path 
    // for all vertices 
    for (int count = 0; count < V - 1; count++) 
    { 
        // Pick the minimum distance 
        // vertex from the set of 
        // vertices not yet processed.  
        // u is always equal to src 
        // in first iteration. 
        int u = minDistance(dist, sptSet); 
  
        // Mark the picked vertex  
        // as processed 
        sptSet[u] = true; 
  
        // Update dist value of the  
        // adjacent vertices of the 
        // picked vertex. 
        for (int v = 0; v < V; v++) 
  
            // Update dist[v] only if is 
            // not in sptSet, there is 
            // an edge from u to v, and  
            // total weight of path from 
            // src to v through u is smaller 
            // than current value of 
            // dist[v] 
            if (!sptSet[v] && graph[u][v].first && 
                dist[u] + graph[u][v].first < dist[v]) 
            { 
                parent[v] = u; 
                dist[v] = dist[u] + graph[u][v].first; 
            }  
    } 
  
    // print the constructed 
    // distance array 
    printSolution(dist, V, parent); 
} 

/************************ Dijkstras Algorithm End ***********************************/

/* Find Best Successor */
void BestSuccessor()
{
    return;
}

/* Find Best Predecessor */
void BestPredecessor()
{
    return;
}

/* find new segment S*ij to replace Sij  */
void WBS()
{
    return;
}

/* Compute optimal DSS */
void MultipleSegmentReplacement()
{
    return;
}

vector<int> FindOptimalDSS(vector<vector<int>> score_gain_mat, int num_edges)
{   // Stores the maximum possible score of each subset and the first cut position in the corresponding subset.
    vector<vector<pair<int,int>>> max_scores_subset(num_edges, vector<pair<int,int>>(num_edges));
    for(int i=0;i<num_edges;i++)
    {
        max_scores_subset[i][i] = make_pair(score_gain_mat[i][i],i);
    }
    for(int spsize = 2; spsize <= num_edges; spsize++)
    {
        int j;
        for(int i = 0; i < num_edges-spsize+1;i++)
        {
            j = i + spsize -1 ;
            pair<int,int> p;
         p.first = 0;
         p.second = 0;
         max_scores_subset[i][j] = p;
            for(int k = i;k <= j ;k++)
            {
                int val;
                if(k+1 >j)
                    val = 0;
                else{
                    val = max_scores_subset[k+1][j].first;
                }
                if(max_scores_subset[i][j].first < score_gain_mat[i][k] + val)
                {
                    max_scores_subset[i][j] = make_pair(score_gain_mat[i][k] + val, k);
                }
            }
          
        }
    }   
  
    vector<int> split_positions;
 
    // Create the vector which contains the split positions
    int j =0;
    while(j<= num_edges-1)    
    {
        split_positions.push_back(max_scores_subset[j][num_edges-1].second+1);
        j = max_scores_subset[j][num_edges-1].second + 1;
    }
    return split_positions;
}

int main() 
{ 
    // Take the filenames of node and edge
    string edge_file,node_file;
    cout<<"Enter the node file name"<<endl;
    node_file = "edit_nodes.txt";
    cout<<"Enter the edge file name"<<endl;
    edge_file = "edit_edges.txt";
 
    // data preprocessing and build graph
    DataPreprocessing(node_file,edge_file);

    // for(auto n:graph)
    // {
    //     for(auto v:n)
    //     cout<<"("<<v.first<<", "<<"("<<v.second.first<<","<<v.second.second<<")";
    //     cout<<endl;
    // }
    //dijkstra(0);
    
    //find intial sead path
 
    //find score gain for all possible segments of seed path
 
    //find optimal DSS 
    //
 
    //invoke WBS on all pieces of optimal DSS in decreasing order of score gain values
    
 
    return 0;
}
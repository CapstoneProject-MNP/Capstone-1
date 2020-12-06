#include<bits/stdc++.h>
#include<fstream>
#include<climits>
#include <sstream>
using namespace std;

# define INF 0x3f3f3f3f

/********Graph Definition Start******************************************************/

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

void printGraph()
{
    for(int i=0; i<graph.size(); i++)
    {
        cout<<i<<"---->";
        for(int j=0; j<graph[i].size(); j++)
        {
        	pair<int, pair<int,int>> node = graph[i][j];
            cout<<node.first<<"->";
        }
        cout<<endl;
    }
}


/************************ Dijkstras Algorithm for Intitial Seed Path Start ***********************************/
  
vector<int> seedPathDijkstra(int src, int dest) 
{ 
	priority_queue< pair<int, int>, vector <pair<int, int>> , greater <pair<int, int>> > pq;
    int V = graph.size();
    cout<<"no. of nodes"<<V<<endl;
	vector<int> dist(V, INF);
    vector<int> parent(V, -1);
    vector<int> seedPath;

	pq.push(make_pair(0, src));
	dist[src] = 0;
	
	vector<bool> f(V, false);

	while (!pq.empty())
	{
		int u = pq.top().second;
		pq.pop();
		f[u] = true;

		pair<int, pair<int,int>> node;
        
		for (int i=0; i < graph[u].size(); i++)
		{
			node = graph[u][i];
            int v = node.first;
			pair<int, int> cost_score = node.second;
            int cost = cost_score.first;

			// If there is shorted path to v through u.
			if (f[v] == false && dist[v] > dist[u] + cost)
			{
				// Updating distance of v
				dist[v] = dist[u] + cost;
				pq.push(make_pair(dist[v], v));
                parent[v] = u;
			}
		}
	}

    //Find shortest path from src to some dest
    stack<int> path;
    int nd = dest;
    while(parent[nd] != -1)
    {
        path.push(parent[nd]);
        nd = parent[nd];
    }

    while(!path.empty())
    {
        seedPath.push_back(path.top());
        path.pop();
    }
    seedPath.push_back(dest);

    return seedPath;      
} 

/************************ Dijkstras Algorithm End ***********************************/

/***************************************Best Successor/Predeccessor Algo Start************************/

  
int EucliDist(int src, int dest) 
{ 
	priority_queue< pair<int, int>, vector <pair<int, int>> , greater <pair<int, int>> > pq;
    int V = graph.size();
    vector<int> dist(V, INF);
	pq.push(make_pair(0, src));
	dist[src] = 0;
	vector<bool> f(V, false);

	while (!pq.empty())
	{
		int u = pq.top().second;
		pq.pop();
		f[u] = true;

		pair<int, pair<int,int>> node;
        
		for (int i=0; i < graph[u].size(); i++)
		{
			node = graph[u][i];
            int v = node.first;
			pair<int, int> cost_score = node.second;
            int cost = cost_score.first;

			if (f[v] == false && dist[v] > dist[u] + cost)
			{
				dist[v] = dist[u] + cost;
				pq.push(make_pair(dist[v], v));
            }
		}
	}
    return dist[dest];
} 

//Compute Gamma value
float GammaValue(int cost, int score, int De)
{
    //cout<<"score = "<<score<<" cost = "<<cost<<" De = "<<De<<endl;;
    float g = float(1 + score)/float(cost + De);
    
    return g;
}

/* Find Best Successor */
pair<int, float> BestSuccessor(int front_tail, int back_tail )
{
    pair<int, float> VG_pair;   // best successor, gamma pair
	pair<int, pair<int,int>> node;
    float gamma = -1;    
	for (int i=0; i < graph[front_tail].size(); i++)  //for all adjacent outgoing edge
	{
		node = graph[front_tail][i];
        int v = node.first;
		pair<int, int> cost_score = node.second;
        int cost = cost_score.first;
        int score = cost_score.second;
        int De = EucliDist(v, back_tail);

        float g = GammaValue(cost, score, De);
        
        if( g > gamma )
        {
            //cout<<" gamma = "<<gamma<<" g = "<<g<<endl;
            gamma = g;
            //cout<<" gmam = "<<gamma<<endl;
            VG_pair.first = v;
            VG_pair.second = g; 
        }
    }

    return VG_pair;
}

/* Find Best Predecessor */
pair<int, float> BestPredecessor(int front_tail, int back_tail )
{
    pair<int, float> VG_pair;  // best predecessor, gamma pair
    float gamma = -1;    

    for(int i=0; i<graph.size(); i++)
    {
        for(int j=0; j<graph[i].size(); j++)
        {
        	pair<int, pair<int,int>> node = graph[i][j];
            if(node.first == back_tail)     //adjacent incoming edge found
            {
                int v = i; // v is incoming edge vertex
		        pair<int, int> cost_score = node.second;
                int cost = cost_score.first;
                int score = cost_score.second;
                int De = EucliDist(front_tail, v);
                float g = GammaValue(cost, score, De);
        
                if( g > gamma )
                {
                    gamma = g;
                    VG_pair.first = v;
                    VG_pair.second = g; 
                }
            }
        }
    }
    return VG_pair;
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
    //printGraph();

    //find intial sead path
    vector<int> seedPath;
    seedPath = seedPathDijkstra(6, 1761);
    for(auto x : seedPath)
    {
        cout<<x<<"->";
    }    
    

    //Best Successor/Pred code check
    pair<int, float> temp_pair = BestSuccessor(6, 1761);
    cout<<"best succ of 6 is "<<temp_pair.first<<" gamma= "<<temp_pair.second<<endl;
    pair<int, float> temp_pair2 = BestPredecessor(6, 1761);
    cout<<"best pred of 1761 is "<<temp_pair2.first<<" gamma= "<<temp_pair2.second<<endl;
 
    //find score gain for all possible segments of seed path
 
    //find optimal DSS 
    //
 
    //invoke WBS on all pieces of optimal DSS in decreasing order of score gain values
    
 
    return 0;
}

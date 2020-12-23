/***
 * Print paths using dfs
 * **/
#include<bits/stdc++.h>
#include <fstream>
using namespace std;
# define INF 0x3f3f3f3f

vector<vector<pair<int, pair<int,int>>>> graph;

void DataPreprocessing(int count_nodes, string edge_file)
{
	string line;
    ifstream edgeFile;
    edgeFile.open(edge_file);
 
    //Build Graph
    graph.resize(count_nodes);
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
        
        graph[stoi(v[0])].push_back(make_pair(stoi(v[1]),make_pair(stoi(v[2]),stoi(v[3]))));
    }
}

void DFSUtil(int v, bool visited[])
{
    visited[v] = true;
    cout << v << " ";
 
    // Recur for all the vertices adjacent
    // to this vertex
    pair<int, pair<int,int>> node;
    for (int i = 0; i != graph[v].size(); ++i)
    {
        node = graph[v][i];
        int u = node.first;
        if (!visited[u])
            DFSUtil(u, visited);
    }
}
 
void dfs(int v)
{
    bool* visited = new bool[v];
    for (int i = 0; i < v; i++)
        visited[i] = false;
    for(int i=0; i<v; i++)
    {
        if(!visited[i])
        {
            DFSUtil(i, visited);
            cout<<endl<<">>";
        }
    }
}

/************************ Dijkstras Algorithm for Intitial Seed Path Start ***********************************/
  
vector<pair<int, pair<int,int>>> seedPathDijkstra(int src, int dest) 
{ 
	priority_queue< pair<int, int>, vector <pair<int, int>> , greater <pair<int, int>> > pq;
    int V = graph.size();
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

		pair<int, pair<int,int>> node;
        
		for (int i=0; i < graph[u].size(); i++)
		{
			node = graph[u][i];
            int v = node.first;
			pair<int, int> cost_score = node.second;
            int cost = cost_score.first;
            int score = cost_score.second;
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


int main()
{
    int num_v = 18263;
    string edge_file = "edges_18k.csv";
    DataPreprocessing(num_v, edge_file);
    //dfs(num_v);
    ofstream MyFile("seedPaths.txt");

    for(int i=0; i<18263; i++)
    {
        for(int j=0; j<18263; j++)
        {
            vector<pair<int, pair<int,int>>> seedPath;
            seedPath = seedPathDijkstra(i, j);
            if(seedPath.size()<25)
                continue;
            for(auto v : seedPath)
            {
                MyFile<<"("<<v.first<<", "<<"("<<v.second.first<<","<<v.second.second<<")";
            }
        MyFile<<endl;
        }

    }  
      // Close the file
  MyFile.close();



    return 0;

}
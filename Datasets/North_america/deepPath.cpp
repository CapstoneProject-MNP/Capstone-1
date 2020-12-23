#include<bits/stdc++.h>
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

void dfs(int num_v)
{
    stack<int> st;

}

int main()
{
    int num_v = 18263;
    string edge_file = "edges_18k.csv";
    DataPreprocessing(num_v, edge_file);
    dfs(num_v);
    return 0;

}
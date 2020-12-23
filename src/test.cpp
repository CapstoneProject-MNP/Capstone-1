#include<bits/stdc++.h>
#include<fstream>
#include<climits>
#include <sstream>
using namespace std;

# define INF 0x3f3f3f3f

/********Graph Definition Start******************************************************/

//graph[i] contains list of (adjacent_node to i,(cost, score))
vector<vector<pair<int, pair<int,int>>>> graph;

// stores all the incoming edges to node i, (cost and score).
vector<vector<pair<int, pair<int,int>>>> incoming_edges;




// void DataPreprocessing(string node_file, string edge_file)
// {
//     ofstream outputFile("bigDataset_65.txt");
//     unordered_map<string,int>map_nodes;
//     ifstream nodeFile;
// 	string line;
// 	long int count_nodes = 0;
    
// 	nodeFile.open(node_file);
	
//     //map unique serial number to each node
// 	while (nodeFile >> line)
// 	{
//         map_nodes[line] = count_nodes;
//         count_nodes++;
// 	}
	
// 	nodeFile.close();

//     ifstream edgeFile;
//     edgeFile.open(edge_file);

//     //Build Graph
//     graph.resize(count_nodes);
//     incoming_edges.resize(count_nodes);
//     while(edgeFile>>line)
//     {
//         //split the line by the delimiter ','
//         vector<string>v;
//         stringstream ss(line);
//         while(ss.good())
//         {
//             string substr;
//             getline(ss, substr, ',');
//             v.push_back(substr);
//         }

//         outputFile<<map_nodes[v[0]]<<","<<map_nodes[v[1]]<<","<<v[2]<<","<<v[3]<<'\n';

//         //Extract the nodes, score and cost

//         // if((map_nodes.find(v[0]) != map_nodes.end()) && (map_nodes.find(v[1]) != map_nodes.end())){
//         //     graph[map_nodes[v[0]]].push_back(make_pair(map_nodes[v[1]],make_pair(stoi(v[2]),stoi(v[3]))));

//         //     //store the incoming edge of a node
//         //     incoming_edges[map_nodes[v[1]]].push_back(make_pair(map_nodes[v[0]],make_pair(stoi(v[2]),stoi(v[3]))));
//         // }
//         // else{
//         //     cout<<"Invalid input";
//         //     exit(1);
//         // }

        
        
        
//     }
//     outputFile.close();
// }

void DataPreprocessing(int count_nodes, string edge_file)
{
    
	string line;
    ifstream edgeFile;
    edgeFile.open(edge_file);

    //Build Graph
    graph.resize(count_nodes);
    incoming_edges.resize(count_nodes);
    int count_edges =0;
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
        count_edges++;
        //Extract the nodes, score and cost

        
        graph[stoi(v[0])].push_back(make_pair(stoi(v[1]),make_pair(stoi(v[2]),stoi(v[3]))));

            //store the incoming edge of a node
        incoming_edges[stoi(v[1])].push_back(make_pair(stoi(v[0]),make_pair(stoi(v[2]),stoi(v[3]))));
        
        
    }
    cout<<count_edges;
}

void printGraph()
{
    for(int i=0; i<graph.size(); i++)
    {
        cout<<i<<"---->>>> ";
        for(int j=0; j<graph[i].size(); j++)
        {
        	pair<int, pair<int,int>> node = graph[i][j];
            cout<<" Node no. = "<<node.first<<" cost "<<node.second.first<<" score "<<node.second.second<<"-------";
        }
        cout<<endl;
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
pair<pair<int,float>,pair<int,int>> BestSuccessor(list<int> forward_tails, vector<int> seed_path, vector<int>path_till_now,int front_tail, int back_tail )
{
    pair<int, float> VG_pair;   // best successor, gamma pair
	pair<int, pair<int,int>> node;
    pair<int,int> final_cost_score;
    float gamma = -1;
    vector<int>::iterator it = find(seed_path.begin(),seed_path.end(),back_tail) ;  
    int index_btail = it - seed_path.begin();
    VG_pair.second = gamma;
	for (int i=0; i < graph[front_tail].size(); i++)  //for all adjacent outgoing edge
	{
		node = graph[front_tail][i];
        int v = node.first;

        if(find(forward_tails.begin(),forward_tails.end(),v)!= forward_tails.end())
        continue;

        if ((seed_path.size()!=0 && (find(seed_path.begin()+index_btail+1,seed_path.end(),v) != seed_path.end())) 
        || (path_till_now.size()!=0 &&  (find(path_till_now.begin(),path_till_now.end(),v) != path_till_now.end() )))
        {
            continue;
        }
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
            final_cost_score.first = cost;
            final_cost_score.second = score;
        }
    }
    pair<pair<int,float>,pair<int,int>> p ;
    p.first = VG_pair;
    p.second = final_cost_score;
    return p;
}

/* Find Best Predecessor */
pair<pair<int,float>,pair<int,int>> BestPredecessor(list<int> backward_tails, vector<int> seed_path, vector<int>path_till_now,int front_tail, int back_tail )
{
    pair<int, float> VG_pair;  // best predecessor, gamma pair
    float gamma = -1;    
    pair<int, pair<int,int>> node;
    pair<int,int> final_cost_score;
    VG_pair.second = gamma;
    //find the index of the btail in the seed path
    vector<int>::iterator it = find(seed_path.begin(),seed_path.end(),back_tail) ;  
    int index_btail = it - seed_path.begin();

	for (int i=0; i < incoming_edges[back_tail].size(); i++)  //for all adjacent incoming edge
	{
		node = incoming_edges[back_tail][i];
        int v = node.first;    //v is incoming edge vertex

        if(find(backward_tails.begin(),backward_tails.end(),v)!= backward_tails.end())
        continue;

        //If any of the two conditions--node already covered in the new path or
        // covered in the seed path in any segment which comes after the current segment then we will drop this node.
        if ((seed_path.size()!=0 && (find(seed_path.begin()+index_btail+1,seed_path.end(),v) != seed_path.end())) 
        || (path_till_now.size()!=0 &&  (find(path_till_now.begin(),path_till_now.end(),v) != path_till_now.end() )))
        {
            continue;
        }

		pair<int, int> cost_score = node.second;   //cost, score pair for incoming edge vertex
        int cost = cost_score.first;
        int score = cost_score.second;
        int De = EucliDist(front_tail, v);     //Euc Dist between front tail in back tail's incoming edge vertex

        float g = GammaValue(cost, score, De);
        
        if( g > gamma )
        {
            gamma = g;
            VG_pair.first = v;
            VG_pair.second = g; 
            final_cost_score.first = cost;
            final_cost_score.second = score;
        }
    }

    pair<pair<int,float>,pair<int,int>> p ;
    p.first = VG_pair;
    p.second = final_cost_score;
    return p;
}

/* find new segment S*ij to replace Sij  */
pair<vector<int>,pair<int,int> >WBS(vector<int> seed_path, vector<int>path_till_now, int f_tail, int b_tail, int budget)
{
    list<int>forward;
    list<int>backward;
    vector<int>candidate_path;
    int candidate_score=0, score_till_now=0, cost_till_now =0,candidate_cost =0;


    // cout<<"budget"<<budget<<endl;
    // cout<<"ftail"<<f_tail<<" btail: "<<b_tail;
    forward.push_back(f_tail);
    backward.push_back(b_tail);
    while(1)
    {
        // Minium cost between ftail and btail
        // vector<pair<int, pair<int,int>>> path_ftail_btail = seedPathDijkstra(f_tail,b_tail);

        // int length_path = path_ftail_btail.size();

        // // if no of hops between the path is 1 or 2 and final cost of the path is less than budget,
        // // then we store the path as candidate path
        // if (length_path == 3 || length_path == 4)
        // {
        //     if(cost_till_now + path_ftail_btail[length_path-1].second.first < budget)
        //     {
        //         // update only when it score is greater than candidate score
        //         if(candidate_score <= score_till_now + path_ftail_btail[length_path-1].second.second)
        //         {
        //             for(auto it = forward.begin();it!=forward.end();it++)
        //             {
        //                 candidate_path.push_back(*it);
        //             }
        //             for(int i=1;i<length_path-1;i++)
        //             {
        //                 candidate_path.push_back(path_ftail_btail[i].first);
        //             }
        //             for(auto it = backward.begin();it!=backward.end();it++)
        //             {
        //                 candidate_path.push_back(*it);
        //             }
        //             candidate_score = score_till_now + path_ftail_btail[length_path-1].second.second;
        //         }
        //     }
        // }

        // vector<int>path;

        // for(auto adj_ftail: graph[f_tail])
        // {
            
        //     if(adj_ftail.first == b_tail || adj_ftail.first == f_tail)
        //     continue;

        //     for(auto edg: graph[adj_ftail.first])
        //     {
        //         if(edg.first == adj_ftail.first || edg.first == f_tail)
        //             continue;
        //         if(edg.first==b_tail && candidate_score < adj_ftail.second.second + edg.second.second+score_till_now 
        //         &&  budget>= adj_ftail.second.first + edg.second.first + cost_till_now )
        //         {
        //             path.clear();
        //             path.push_back(adj_ftail.first);
        //             candidate_score = adj_ftail.second.second + edg.second.second+score_till_now ;
        //             candidate_cost = adj_ftail.second.first + edg.second.first + cost_till_now;
        //         }
        //         if(edg.first!=b_tail){
        //             for(auto e:graph[edg.first])
        //             {
        //                 if(e.first==b_tail && candidate_score < e.second.second + adj_ftail.second.second + edg.second.second+score_till_now 
        //                 &&  budget>= e.second.first + adj_ftail.second.first + edg.second.first + cost_till_now )
        //                 {
        //                     path.clear();
        //                     path.push_back(adj_ftail.first);
        //                     path.push_back(edg.first);
        //                     candidate_score = e.second.second + adj_ftail.second.second + edg.second.second+score_till_now;
        //                     candidate_cost = adj_ftail.second.first + edg.second.first + cost_till_now;
        //                     break;
        //                 }
        //             }
        //         }
        //     }
        // }

        // if(path.size()>0)
        // {
        //     candidate_path.clear();
        //     for(auto it = forward.begin();it!=forward.end();it++)
        //     {
        //         candidate_path.push_back(*it);
        //     }
        //     for(auto it = path.begin();it!=path.end();it++)
        //     {
        //         candidate_path.push_back(*it);
        //     }
        //     for(auto it = backward.begin();it!=backward.end();it++)
        //     {
        //         candidate_path.push_back(*it);
        //     }
        //     path.clear();

        // }

        pair<pair<int,float>,pair<int,int>>best_succ = BestSuccessor(forward,seed_path, path_till_now,f_tail,b_tail);
        pair<pair<int,float>,pair<int,int>>best_pred = BestPredecessor(backward, seed_path, path_till_now,f_tail, b_tail);

        if(best_succ.first.second == -1 || best_pred.first.second == -1)
            break;


        //update ftail
        if(best_succ.first.second >= best_pred.first.second) // gamma value comparison
        {
            cost_till_now += best_succ.second.first;
            score_till_now += best_succ.second.second;

            if(best_succ.first.first != b_tail){
                //Move forward search
                forward.push_back(best_succ.first.first);
                f_tail = best_succ.first.first;
            }
            //terminating condition
            else{
                if(candidate_score < score_till_now)
                {
                    candidate_path.clear();
                    for(auto it = forward.begin();it!=forward.end();it++)
                    {
                        candidate_path.push_back(*it);
                    }
                    for(auto it = backward.begin();it!=backward.end();it++)
                    {
                        candidate_path.push_back(*it);
                    }
                    candidate_score = score_till_now;
                }
                
                break;
            }

            
            //Move backward search
            pair<pair<int,float>,pair<int,int>>pred = BestPredecessor(backward, seed_path, path_till_now, f_tail, b_tail);

            cost_till_now+= pred.second.first;
            score_till_now+=pred.second.second;

            if(pred.first.first != f_tail){
                //Move forward search
                backward.push_front(pred.first.first);
                b_tail = pred.first.first;
            }
            //terminating condition
            else{
                if(candidate_score < score_till_now)
                {
                    candidate_path.clear();
                    for(auto it = forward.begin();it!=forward.end();it++)
                    {
                        candidate_path.push_back(*it);
                    }
                    for(auto it = backward.begin();it!=backward.end();it++)
                    {
                        candidate_path.push_back(*it);
                    }
                    candidate_score = score_till_now;
                }
                
                break;
            }
            
            
        }
        else{

            cost_till_now+= best_pred.second.first;
            score_till_now+= best_pred.second.second;
            //Move backward search
            if(best_pred.first.first!=f_tail)
            {
                backward.push_front(best_pred.first.first);
                b_tail = best_pred.first.first;
            }
            else{
                if(candidate_score < score_till_now)
                {
                    candidate_path.clear();
                    for(auto it = forward.begin();it!=forward.end();it++)
                    {
                        candidate_path.push_back(*it);
                    }
                    for(auto it = backward.begin();it!=backward.end();it++)
                    {
                        candidate_path.push_back(*it);
                    }
                    candidate_score = score_till_now;
                }
               
                break;
            }
            
            
            //Move forward search
            pair<pair<int,float>,pair<int,int>>succ = BestSuccessor(forward, seed_path, path_till_now, f_tail, b_tail);

            cost_till_now+= succ.second.first;
            score_till_now+= succ.second.second;
            //Move backward search
            if(succ.first.first!=b_tail)
            {
                forward.push_back(succ.first.first);
                f_tail = succ.first.first;
            }
            else{
                if(candidate_score < score_till_now)
                {
                    candidate_path.clear();
                    for(auto it = forward.begin();it!=forward.end();it++)
                    {
                        candidate_path.push_back(*it);
                    }
                    for(auto it = backward.begin();it!=backward.end();it++)
                    {
                        candidate_path.push_back(*it);
                    }
                    candidate_score = score_till_now;
                }
               
                break;
            }
            
            
            
        }

        if(cost_till_now > budget)
            break;
    }

    //stores the path and its total score
    pair<vector<int>,pair<int,int>> updated_path;
    updated_path.first = candidate_path;

    pair<int,int> cost_score_final;
    cost_score_final.first = cost_till_now;
    cost_score_final.second = candidate_score;
    updated_path.second = cost_score_final;
    return updated_path;
}

/* Compute optimal DSS */

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


vector<vector<int>> CalculateScoreGain(vector<pair<int,pair<int,int>>> seedPath,long int overhead)
{
    vector<int>path;
    vector<int>seed_path;
    int budget_segment;
    int num_nodes = seedPath.size();
    vector<vector<int>> score_gain_mat(num_nodes-1,vector<int>(num_nodes-1,0));

    for(int i=0; i<num_nodes-1;i++)
    {
        for(int j=i+1;j<num_nodes;j++)
        {
            pair<vector<int>,pair<int,int>> updated_segment;
            updated_segment = WBS(seed_path,path,seedPath[i].first,seedPath[j].first,overhead + seedPath[j].second.first - seedPath[i].second.first);
            
            score_gain_mat[i][j-1] = updated_segment.second.second == 0 ? 0: updated_segment.second.second - (seedPath[j].second.second - seedPath[i].second.second);
        }
    }
    return score_gain_mat;
}


void PrintFinalPath(vector<int> split_positions, vector<pair<int, pair<int,int>>> seedPath, int overhead)
{
    vector<int> path_till_now;
    long int total_cost = 0,total_score = 0;
    vector<int>seed_path;
    int flag = 0;
    int current_overhead = overhead;
    pair<vector<int>,pair<int,int>> updated_segment;

    for(auto v:seedPath)
    {
        seed_path.push_back(v.first);
    }

    int i=0;
    for(i=0;i<split_positions.size();i++)
    {
        
        if(i==0)
        {
            updated_segment = WBS(seed_path, path_till_now, seed_path[0],seed_path[split_positions[i]],seedPath[split_positions[i]].second.first + overhead);
            int cost = updated_segment.second.first;
            int score = updated_segment.second.second;
            int overhead_seg = cost - seedPath[split_positions[i]].second.first;

            if(current_overhead - overhead_seg < 0)
            {
                flag = 1 ;
                break;
            }
            if(score == 0)
            {
                for(int j=0;j<=split_positions[i];j++)
                path_till_now.push_back(seed_path[j]);

                total_cost += seedPath[split_positions[i]].second.first;
                total_score += seedPath[split_positions[i]].second.second;
            }
            else
            {
                path_till_now = updated_segment.first;
                current_overhead -= overhead_seg;
                total_cost+=cost;
                total_score +=score;

            }
            
        }
        else 
        {
            int ind1 = split_positions[i-1];
            int ind2 = split_positions[i];
            updated_segment = WBS(seed_path, path_till_now, seed_path[ind1], seed_path[ind2],seedPath[ind2].second.first - seedPath[ind1].second.first + overhead);
            int cost = updated_segment.second.first;
            int score = updated_segment.second.second;
            int overhead_seg = cost - (seedPath[ind2].second.first - seedPath[ind1].second.first);

            if(current_overhead - overhead_seg < 0)
            {
                flag = 1 ;
                break;
            }
            if(score == 0)
            {
                for(int j=ind1+1;j<=ind2;j++)
                path_till_now.push_back(seed_path[j]);
                total_cost += seedPath[ind2].second.first - seedPath[ind1].second.first;
                total_score += seedPath[ind2].second.second - seedPath[ind1].second.second;
            }
            else
            {
                for(int j=1;j<updated_segment.first.size();j++)
                {
                    path_till_now.push_back(updated_segment.first[j]);
                }
                current_overhead -= overhead_seg;
                total_cost+=cost;
                total_score+=score;
            }
         
            
        }
        
        // for(auto n:updated_segment.first)
        // {
        //     cout<<n<<" ";
        // }
        // cout<<"In loop "<<i<<endl;
        // for(auto v: path_till_now)
        // {
        //     cout<<v<<" ";
        // }
        // cout<<"out"<<endl;
    


    }

    // if(flag == 0)
    // {
    //     updated_segment = WBS(seed_path, path_till_now, seed_path[split_positions[i-1]],seed_path[seed_path.size()-1],overhead+seedPath[seed_path.size()-1].second.first - seedPath[split_positions[i-1]].second.first );
    //     int overhead_seg = updated_segment.second.first - (seedPath[seed_path.size()-1].second.first - seedPath[split_positions[i-1]].second.first);

    //     if(overhead - overhead_seg < 0 || updated_segment.second.second == 0)
    //     {
    //         int ind1 = split_positions[i-1];
    //         int ind2 = seed_path.size()-1;
    //         for(int j=ind1+1;j<=ind2;j++)
    //         path_till_now.push_back(seed_path[j]);
    //         total_cost += seedPath[ind2].second.first - seedPath[ind1].second.first;
    //         total_score += seedPath[ind2].second.second - seedPath[ind1].second.second;
    //     }
    //     else
    //     {
    //         for(int j=1;j<updated_segment.first.size();j++)
    //         {
    //             path_till_now.push_back(updated_segment.first[j]);
    //         }
    //         overhead -= overhead_seg;
    //         total_cost+= updated_segment.second.first;
    //         total_score+= updated_segment.second.second;
    //     }


    // }
    if(flag == 1)
    {
        for(int j=split_positions[i-1];j<seed_path.size();j++)
        {
            path_till_now.push_back(seed_path[j]);
        }
        total_cost+= seedPath[seed_path.size()-1].second.first - seedPath[split_positions[i-1]].second.first;
        total_score += seedPath[seed_path.size()-1].second.second - seedPath[split_positions[i-1]].second.second;
        
    }
    

    for(auto v: path_till_now)
    {
        cout<<v<<" ";
    }
    

    cout<<"Cost: "<<total_cost<<endl;
    cout<<"Score: "<<total_score<<endl;

    //cout<<"  "<<path_till_now.size()<<endl;

}



int main() 
{ 
    clock_t start,end;
    start = clock();
    int num_nodes;
    cout<<"Number of nodes: ";
    cin>>num_nodes;
    cout<<endl;

    // Take the filenames of node and edge
    string edge_file;
    cout<<"Enter the edge file name"<<endl;
    cin>>edge_file;

    int src, dest;
    cout<<"Enter the souce and the destination: ";
    cin>>src>>dest;
    cout<<endl;

    //Overhead
    int overhead;
    cout<<"Enter the value of overhead";
    cin>>overhead;
 
    // data preprocessing and build graph
    DataPreprocessing(num_nodes,edge_file);

    for(auto n:graph)
    {
        for(auto v:n)
        cout<<"("<<v.first<<", "<<"("<<v.second.first<<","<<v.second.second<<")";
        cout<<endl;
    }
    for(auto v:graph[6])
        cout<<"("<<v.first<<", "<<"("<<v.second.first<<","<<v.second.second<<")";
        cout<<endl;

    printGraph();

    //find intial sead path
    vector<pair<int, pair<int,int>>> seedPath;
    seedPath = seedPathDijkstra(src,dest);
    for(auto v : seedPath)
    {
        cout<<"("<<v.first<<", "<<"("<<v.second.first<<","<<v.second.second<<")"<<endl;
    }    
    cout<<"Cost: "<<seedPath[seedPath.size()-1].second.first<<endl;
    cout<<"Score: "<<seedPath[seedPath.size()-1].second.second<<endl;
    // //Budget is the sum of overhead and cost of the seed path
    long int budget = overhead + seedPath[seedPath.size()-1].second.first;

  

    // // // calculte score gain for each possible segment
    vector<vector<int>>score_gain_mat;
    score_gain_mat = CalculateScoreGain(seedPath, overhead);

    for(int i=0;i<score_gain_mat.size();i++)
    {
        for(int j=0;j<score_gain_mat[i].size();j++)
        {
            cout<<score_gain_mat[i][j]<<" ";
        }
        cout<<endl;
    }
    end = clock();
    double time_taken = double(end - start) / double(CLOCKS_PER_SEC); 
    cout << "Time taken by program is : " << fixed  
         << time_taken << setprecision(5); 
    cout << " sec " << endl; 
    // //Find optimal DSS
    // vector<int> split_positions = FindOptimalDSS(score_gain_mat, seedPath.size()-1);

    // for(auto v : split_positions)
    // {
    //     cout<<v<<" ";
    // }
    // cout<<"New Path: "<<endl;

    // //Final path
    // PrintFinalPath(split_positions,seedPath, overhead);

    
 
    return 0;
}

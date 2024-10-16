#include <iostream>
#include <vector>
#include <stack>
#include <set>
using namespace std;

class Graph
{
    int V;
    void DFSUtil(int v, vector<vector<int>> &adj, vector<bool> &visited)
    {
        visited[v] = true;
        cout << v << " ";

        for (auto u : adj[v])
        {
            if (!visited[u])
                DFSUtil(u, adj, visited);
        }
    }

public:
    Graph(int V)
    {
        this->V = V;
    }

    void TopologicalSort(vector<vector<int>> &adj)
    {
        vector<int> indegree(adj.size(), 0);
        int V = adj.size();
        set<int> s;
        for (int i = 0; i < V; i++)
        {
            for (auto u : adj[i])
                indegree[u]++;
        }
        for (int i = 0; i < V; i++)
            if (indegree[i] == 0)
                s.insert(i);
        while (!s.empty())
        {
            auto it = s.begin();
            cout << *it << ' ';
            int v = *it;
            s.erase(it);
            for (auto u : adj[v])
            {
                indegree[u]--;
                if (indegree[u] == 0)
                    s.insert(u);
            }
        }
        cout << endl;
    }

    void DFS(vector<vector<int>> &adj, int start)
    {
        vector<bool> visited(V, false);
        for (int i = 0; i < V; i++)
            if (!visited[i])
                DFSUtil(i, adj, visited);
        cout << endl;
    }
};

int main()
{
    int V = 6;
    Graph g(V);

    // Adjacency matrix for the graph
    vector<vector<int>> Adj_mat = {
        {0, 1, 1, 0, 0, 0},
        {0, 0, 1, 1, 0, 0},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 1, 1},
        {0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0}};

    vector<vector<int>> adj_list(Adj_mat.size());
    for (int i = 0; i < adj_list.size(); i++)
    {
        for (int j = 0; j < adj_list.size(); j++)
            if (Adj_mat[i][j] != 0)
                adj_list[i].push_back(j);
    }
    g.TopologicalSort(adj_list);

    return 0;
}
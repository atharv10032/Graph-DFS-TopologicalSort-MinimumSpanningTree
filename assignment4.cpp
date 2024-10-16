#include <iostream>
#include <vector>
#include <stack>
#include <set>
#include <queue>
#include <algorithm>
using namespace std;

class Graph
{
    int V;

    int find(vector<int> &parent, int i)
    {
        if (parent[i] != i)
            parent[i] = find(parent, parent[i]);
        return parent[i];
    }

    void unionSets(vector<int> &parent, vector<int> &rank, int x, int y)
    {
        int rootX = find(parent, x);
        int rootY = find(parent, y);

        if (rank[rootX] < rank[rootY])
            parent[rootX] = rootY;
        else if (rank[rootX] > rank[rootY])
            parent[rootY] = rootX;
        else
        {
            parent[rootY] = rootX;
            rank[rootX]++;
        }
    }

    void DFSUtil(int v, vector<vector<int>> &adj, vector<bool> &visited, vector<int> &entry_time, vector<int> &exit_time, int &time)
    {
        visited[v] = true;
        entry_time[v] = ++time;

        for (auto u : adj[v])
        {
            if (!visited[u])
                DFSUtil(u, adj, visited, entry_time, exit_time, time);
        }
        exit_time[v] = ++time;
    }

    bool cycle_detection_util(int v, vector<vector<int>> &adj, vector<bool> &visited, vector<bool> &rec_stack)
    {
        visited[v] = rec_stack[v] = true;

        for (auto u : adj[v])
        {
            if (!visited[u])
            {
                if (cycle_detection_util(u, adj, visited, rec_stack))
                    return true;
            }
            else if (rec_stack[u])
                return true;
        }
        rec_stack[v] = false;
        return false;
    }

public:
    Graph(int V)
    {
        this->V = V;
    }

    void TopologicalSort(vector<vector<int>> &adj)
    {
        vector<int> indegree(V, 0);
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

    void DFS(vector<vector<int>> &adj, vector<int> &entry_time, vector<int> &exit_time)
    {
        vector<bool> visited(V, false);
        int time = 0;
        for (int i = 0; i < V; i++)
            if (!visited[i])
                DFSUtil(i, adj, visited, entry_time, exit_time, time);
    }

    bool cycle_detection(vector<vector<int>> &adj)
    {
        vector<bool> visited(V, false);
        vector<bool> rec_stack(V, false);

        for (int i = 0; i < V; i++)
            if (!visited[i])
                if (cycle_detection_util(i, adj, visited, rec_stack))
                    return true;

        return false;
    }

    vector<vector<int>> Prim(vector<vector<pair<int, int>>> &adj)
    {
        vector<bool> selected(V, false);
        priority_queue<pair<int, pair<int, int>>, vector<pair<int, pair<int, int>>>, greater<pair<int, pair<int, int>>>> min_heap;

        vector<vector<int>> mst(V, vector<int>(V, 0)); // To store the MST

        // Start from the first node (node 0)
        selected[0] = true;

        // Add all edges from the starting node (0) to the priority queue
        for (auto edge : adj[0])
            min_heap.push({edge.first, {0, edge.second}});

        while (!min_heap.empty())
        {
            auto [weight, edge] = min_heap.top(); // Extract the edge with minimum weight
            min_heap.pop();

            int u = edge.first;
            int v = edge.second;

            if (!selected[v])
            {
                // Mark the vertex v as selected (included in the MST)
                selected[v] = true;

                // Add the edge u-v to the MST
                mst[u][v] = weight;
                mst[v][u] = weight;

                // Add all edges from the newly included vertex v
                for (auto next_edge : adj[v])
                {
                    if (!selected[next_edge.second])
                        min_heap.push({next_edge.first, {v, next_edge.second}});
                }
            }
        }

        return mst; // Return the adjacency matrix representing the MST
    }

    vector<vector<pair<int, int>>> Kruskal(vector<vector<pair<int, int>>> &adj)
    {
        vector<pair<int, pair<int, int>>> edges;

        for (int u = 0; u < V; u++)
        {
            for (auto edge : adj[u])
            {
                int v = edge.second;
                int weight = edge.first;
                if (u < v)
                    edges.push_back({weight, {u, v}});
            }
        }

        sort(edges.begin(), edges.end());

        vector<int> parent(V), rank(V, 0);
        for (int i = 0; i < V; i++)
            parent[i] = i;

        vector<vector<pair<int, int>>> mst(V);

        for (auto edge : edges)
        {
            int u = edge.second.first;
            int v = edge.second.second;
            int weight = edge.first;

            int rootU = find(parent, u);
            int rootV = find(parent, v);

            if (rootU != rootV)
            {
                mst[u].push_back({weight, v});
                mst[v].push_back({weight, u});

                unionSets(parent, rank, rootU, rootV);
            }
        }

        return mst;
    }
};

vector<vector<pair<int, int>>> weighted_adj_list(vector<vector<int>> &mat)
{
    vector<vector<pair<int, int>>> adj_list(mat.size());
    for (int i = 0; i < mat.size(); i++)
    {
        for (int j = 0; j < mat.size(); j++)
        {
            if (mat[i][j] > 0)
                adj_list[i].push_back({mat[i][j], j});
        }
    }

    return adj_list;
}

int main()
{
    int algorithm;
    cin >> algorithm;

    int test_cases;
    cin >> test_cases;

    while (test_cases--)
    {
        int V;
        cin >> V;

        vector<vector<int>> Adj_mat(V, vector<int>(V));

        for (int i = 0; i < V; i++)
            for (int j = 0; j < V; j++)
                cin >> Adj_mat[i][j];

        Graph g(V);

        if (algorithm == 1)
        {
            vector<vector<int>> adj_list(V);
            for (int i = 0; i < V; i++)
                for (int j = 0; j < V; j++)
                    if (Adj_mat[i][j] == 1)
                        adj_list[i].push_back(j);

            g.TopologicalSort(adj_list);
        }
        else if (algorithm == 2)
        {
            vector<vector<int>> adj_list(V);
            for (int i = 0; i < V; i++)
                for (int j = 0; j < V; j++)
                    if (Adj_mat[i][j] == 1)
                        adj_list[i].push_back(j);

            vector<int> entry_time(V, 0), exit_time(V, 0);
            bool cycle_exists = g.cycle_detection(adj_list);

            if (cycle_exists)
                cout << "Yes" << endl;
            else
                cout << "No" << endl;

            g.DFS(adj_list, entry_time, exit_time);

            for (int i = 0; i < V; i++)
                cout << entry_time[i] << ' ';
            cout << endl;

            for (int i = 0; i < V; i++)
                cout << exit_time[i] << ' ';
            cout << endl;
        }
        else if (algorithm == 3)
        {
            vector<vector<pair<int, int>>> adj_list = weighted_adj_list(Adj_mat);
            vector<vector<int>> mst_mat = g.Prim(adj_list);

            for (int i = 0; i < V; i++)
            {
                for (int j = 0; j < V; j++)
                    cout << mst_mat[i][j] << ' ';
                cout << endl;
            }
        }
        else if (algorithm == 4)
        {
            vector<vector<pair<int, int>>> adj_list = weighted_adj_list(Adj_mat);
            vector<vector<pair<int, int>>> mst = g.Kruskal(adj_list);

            vector<vector<int>> mst_mat(V, vector<int>(V, 0));
            for (int i = 0; i < V; i++)
            {
                for (auto x : mst[i])
                    mst_mat[i][x.second] = x.first;
            }

            for (int i = 0; i < V; i++)
            {
                for (int j = 0; j < V; j++)
                    cout << mst_mat[i][j] << ' ';
                cout << endl;
            }
        }
    }

    return 0;
}
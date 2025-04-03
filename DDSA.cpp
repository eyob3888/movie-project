
#include <iostream>
#include <vector>
#include <queue>
#include <climits>       
#include <unordered_map>  
#include <algorithm>     
using namespace std;

class WeightedGraph {
private:
    // Adjacency list representation of the graph
    
    unordered_map<char, vector<pair<char, int>>> adjList;

public:
    // Function to add an edge between two nodes with a given weight
    
    void addEdge(char u, char v, int weight) {
        adjList[u].push_back({v, weight});
        adjList[v].push_back({u, weight}); 
    }

    // Function to delete a node from the graph
    void deleteNode(char node) {
        
        for (auto& neighbor : adjList[node]) {
            char v = neighbor.first;
            
             
            adjList[v].erase(
                remove_if(adjList[v].begin(), adjList[v].end(),
                    [node](const pair<char, int>& p) { return p.first == node; }),
                adjList[v].end()
            );
        }
        
         
        adjList.erase(node);
    }

    // Function to update the weight of an edge between two nodes
    void updateWeight(char u, char v, int newWeight) {
        bool edgeFound = false;
        
        // Update weight in u's adjacency list
        for (auto& neighbor : adjList[u]) {
            if (neighbor.first == v) {
                neighbor.second = newWeight;
                edgeFound = true;
                break;
            }
        }
        if (edgeFound) {
            for (auto& neighbor : adjList[v]) {
                if (neighbor.first == u) {
                    neighbor.second = newWeight;
                    break;
                }
            }
        }
    }

    // Breadth-First Search traversal starting from a given node
    void bfs(char start) {
        
        if (adjList.find(start) == adjList.end()) return;

        unordered_map<char, bool> visited;  
        queue<char> q;                     

        
        q.push(start);
        visited[start] = true;

        cout << "BFS Traversal: ";
        while (!q.empty()) {
            char current = q.front();
            q.pop();
            cout << current << " ";  

            // Visit all adjacent nodes
            for (const auto& neighbor : adjList[current]) {
                if (!visited[neighbor.first]) {
                    visited[neighbor.first] = true;
                    q.push(neighbor.first);
                }
            }
        }
        cout << endl;
    }

    // Dijkstra's algorithm to find the shortest path between two nodes
    void shortestPath(char start, char end) {
        // Check if both nodes exist in the graph
        if (adjList.find(start) == adjList.end() || adjList.find(end) == adjList.end()) {
            cout << "Start or end node not found!" << endl;
            return;
        }

        
        priority_queue<pair<int, char>, vector<pair<int, char>>, greater<pair<int, char>>> pq;
        unordered_map<char, int> distances;    
        unordered_map<char, char> previous;    

        
        for (const auto& node : adjList) {
            distances[node.first] = INT_MAX;
        }

        // Distance to start node is 0
        distances[start] = 0;
        pq.push({0, start});

        while (!pq.empty()) {
            char current = pq.top().second;
            int currentDist = pq.top().first;
            pq.pop();

            // Skip if we've already found a better path to this node
            if (currentDist > distances[current]) continue;

            
            for (const auto& neighbor : adjList[current]) {
                char v = neighbor.first;
                int weight = neighbor.second;
                int distanceThroughCurrent = currentDist + weight;

                // If we found a shorter path to neighbor
                if (distanceThroughCurrent < distances[v]) {
                    distances[v] = distanceThroughCurrent;
                    previous[v] = current; 
                    pq.push({distances[v], v});
                }
            }
        }

        if (distances[end] == INT_MAX) {
            cout << "No path exists from " << start << " to " << end << endl;
            return;
        }

        // Reconstruct and print the path
        cout << "Shortest path from " << start << " to " << end << ": ";
        vector<char> path;
        // Backtrack from end to start using the previous map
        for (char at = end; at != start; at = previous[at]) {
            path.push_back(at);
        }
        path.push_back(start);

        // Reverse to get the correct order
        reverse(path.begin(), path.end());
        for (char node : path) {
            cout << node << " ";
        }
        cout << "\nTotal distance: " << distances[end] << endl;
    }

    // Function to print the graph's adjacency list representation
    void printGraph() {
        cout << "Graph representation (Adjacency List):" << endl;
        for (const auto& node : adjList) {
            cout << node.first << " -> ";
            for (const auto& neighbor : node.second) {
                cout << "(" << neighbor.first << ", " << neighbor.second << ") ";
            }
            cout << endl;
        }
    }
};

int main() {
    WeightedGraph graph;

    // Build the graph by adding edges
    graph.addEdge('A', 'B', 4);
    graph.addEdge('A', 'C', 2);
    graph.addEdge('B', 'C', 1);
    graph.addEdge('B', 'D', 5);
    graph.addEdge('C', 'D', 8);
    graph.addEdge('C', 'E', 10);
    graph.addEdge('D', 'E', 2);
    graph.addEdge('D', 'F', 6);
    graph.addEdge('E', 'F', 2);

    // Print the initial graph
    graph.printGraph();

    // Perform various operations to demonstrate functionality
    cout << "\nOperations:\n";
    
    // Perform BFS traversal starting from 'A'
    graph.bfs('A');
    
    // Find shortest path from 'A' to 'F'
    graph.shortestPath('A', 'F');
    
    // Update an edge weight and show the result
    cout << "\nUpdating weight of edge A-B from 4 to 3\n";
    graph.updateWeight('A', 'B', 3);
    graph.printGraph();
    
    // Find shortest path again after the update
    graph.shortestPath('A', 'F');
    
    // Delete a node and show the result
    cout << "\nDeleting node C\n";
    graph.deleteNode('C');
    graph.printGraph();
    
    // Find shortest path after node deletion
    graph.shortestPath('A', 'F');

    return 0;
}
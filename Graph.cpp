#include "Graph.h"
#include "Disjoint.h"
#include <fstream>
#include <sstream>
#include <queue>
#include <iostream>

bool tupleComp::operator()(tuple<string,string,double>& first, tuple<string,string,double>& second) const{
    return get<2>(first) > get<2>(second);
}

Graph::Graph(const char* const & edgelist_csv_fn) {
    // TODO
    ifstream my_file(edgelist_csv_fn);      // open the file
    string line;                     // helper var to store current line
    while(getline(my_file, line)) {  // read one line from the file
        istringstream ss(line);      // create istringstream of current line
        string first, second; // helper vars
        double third;
        getline(ss, first, ',');     // store first column in "first"
        getline(ss, second, ',');    // store second column in "second"
        // getline(ss, third, '\n');    // store third column column in "third"
        ss >> third;

        adjacencyList[first].push_back(make_tuple(first,second,third));
        adjacencyList[second].push_back(make_tuple(second,first,third));
    }

    my_file.close();                 // close file when done
}

unsigned int Graph::num_nodes() {
    return adjacencyList.size();
}

vector<string> Graph::nodes() {
    vector<string> nodes;
    for(auto& pair : adjacencyList){
        nodes.push_back(pair.first);
    }
    return nodes;
}

unsigned int Graph::num_edges() {
    int numEdges = 0;
    for(auto& pair : adjacencyList){
        numEdges += pair.second.size();
    }
    return numEdges / 2;
}

unsigned int Graph::num_neighbors(string const & node_label) {
    return adjacencyList[node_label].size();

}

double Graph::edge_weight(string const & u_label, string const & v_label) {
    for(auto& neighbor : adjacencyList[u_label]){
        if(get<1>(neighbor) == v_label){
            return get<2>(neighbor);
        }
    }
    return -1;
}

vector<string> Graph::neighbors(string const & node_label) {
    vector<string> list;
    for(auto& neighbor : adjacencyList[node_label]){
        list.push_back(get<1>(neighbor));
    }

    return list;
}

vector<string> Graph::shortest_path_unweighted(string const & start_label, string const & end_label) {
    // BFS
    unordered_map<string, bool> visited;
    unordered_map<string, string> parent;
    queue<string> q;
    vector<string> path;

    q.push(start_label);
    while(!q.empty())
    {
        string currNode = q.front();
        q.pop();
        if(currNode == end_label){
            string endNode = end_label;
            while(endNode != start_label){
                path.insert(path.begin(),endNode);
                endNode = parent[endNode];
            }
            path.insert(path.begin(),start_label);
            return path;
        }
        for(auto & neighbor : adjacencyList[currNode]){
            string neighborNode = get<1>(neighbor);
            if(!visited[neighborNode]){
                q.push(neighborNode);
                visited[neighborNode] = true;
                parent[neighborNode] = currNode;
            }
        }
    }
    vector<string> empty;
    return empty;
}

vector<tuple<string,string,double>> Graph::shortest_path_weighted(string const & start_label, string const & end_label) {
    priority_queue<tuple<string, string, double>, vector<tuple<string, string, double>>, tupleComp> pq;

    unordered_map<string, double> distances;
    unordered_map<string, string> parent;

    for (const auto& pair : adjacencyList) {
        const string& nodeLabel = pair.first;
        distances[nodeLabel] = 100000000000;
    }

    pq.push(make_tuple(start_label, start_label,0.0));
    distances[start_label] = 0.0;

    if(start_label == end_label){
        vector<tuple<string, string, double>> path;
        path.push_back(make_tuple(start_label,start_label,-1));
        return path;
    }

    while (!pq.empty()) {
        auto current = pq.top();
        pq.pop();
        string currentLabel = get<1>(current);
        double distance = get<2>(current);

        if(distance > distances[currentLabel]){
            continue;
        }

        if (currentLabel == end_label) {
            vector<tuple<string, string, double>> path;
            string node = end_label;
            
            while (node != start_label) {
                string parentNode = parent[node];
                double weight = edge_weight(parentNode, node);
                path.insert(path.begin(), make_tuple(parentNode, node, weight));
                node = parentNode;
            }
            return path;
        }

        const auto& neighbors = adjacencyList[currentLabel];
        for (const auto& neighbor : neighbors) {
            string neighborLabel = get<1>(neighbor);
            double weight = get<2>(neighbor);
            double newDistance = distances[currentLabel] + weight;

            if (newDistance < distances[neighborLabel]) {
                distances[neighborLabel] = newDistance;
                parent[neighborLabel] = currentLabel;
                pq.push(make_tuple(currentLabel,neighborLabel,newDistance));
            }
        }
    }

    return vector<tuple<string, string, double>>(); // No path found
}

vector<vector<string>> Graph::connected_components(double const & threshold) {
    // TODO
    unordered_map<string, bool> visited;
    queue<string> q;
    vector<vector<string>> totalPaths;

    for(auto node : adjacencyList){
        string nodeLabel = node.first;
        visited[nodeLabel] = false;
    }

    for(auto newNodes : adjacencyList){
        string nodeLabel = newNodes.first;
        if(visited[nodeLabel] == true){
            continue;
        }
        //Run BFS 
        vector<string> connectedPaths;
        q.push(nodeLabel);
        visited[nodeLabel] = true;
        while(!q.empty())
        {
            string currNode = q.front();
            q.pop();
            connectedPaths.push_back(currNode);
            for(auto & neighbor : adjacencyList[currNode]){
                string neighborNode = get<1>(neighbor);
                double edgeWeight = get<2>(neighbor);
                if(!visited[neighborNode] && edgeWeight <=  threshold){
                    q.push(neighborNode);
                    visited[neighborNode] = true;
                    // parent[neighborNode] = currNode;
                }
            }
        }
        totalPaths.push_back(connectedPaths);
    }
    return totalPaths;
}

double Graph::smallest_connecting_threshold(string const & start_label, string const & end_label) {
    if(start_label == end_label){
        return 0;
    }
        // Create a disjoint set object
    Disjoint ds;

    // Make a set for each node in the graph
    for (const auto& entry : adjacencyList) {
        const string& node = entry.first;
        ds.makeSet(node);
    }

    // Traverse the graph using breadth-first search (BFS)
    queue<string> q;
    unordered_map<string, bool> visited;
    unordered_map<string, double> thresholds;

    q.push(start_label);
    visited[start_label] = true;
    thresholds[start_label] = 1000000000000000;  // Initialize the threshold with maximum value

    while (!q.empty()) {
        string current_node = q.front();
        q.pop();

        // Process the neighbors of the current node
        for (const auto& edge : adjacencyList[current_node]) {
            // string source, destination;
            // double weight;
            // tie(source, destination, weight) = edge;
            string source = get<0>(edge);
            string destination = get<1>(edge);
            double weight = get<2>(edge);
            // Update the threshold for the destination node
            double current_threshold = min(thresholds[current_node], weight);
            if (current_threshold < thresholds[destination]) {
                thresholds[destination] = current_threshold;

                // Union the sets of the source and destination nodes
                ds.unite(source, destination);

                // Visit the destination node if not visited before
                if (!visited[destination]) {
                    visited[destination] = true;
                    q.push(destination);
                }
            }
        }
    }

    // Check if the start and end nodes are in the same set
    if (ds.find(start_label) == ds.find(end_label)) {
        return thresholds[end_label];
    }

    // The start and end nodes are not connected
    return -1.0;
}

void Disjoint::makeSet(const string& node) {
    parent[node] = node;
    rank[node] = 0;
}

string Disjoint::find(const string& node) {
    if (parent[node] != node) {
        parent[node] = find(parent[node]);
    }
    return parent[node];
}

void Disjoint::unite(const string& nodeA, const string& nodeB) {
    string rootA = find(nodeA);
    string rootB = find(nodeB);

    if (rootA == rootB) {
        return;  // Nodes are already in the same set
    }

    // Attach the smaller rank tree under the root of the higher rank tree
    if (rank[rootA] < rank[rootB]) {
        parent[rootA] = rootB;
    } else if (rank[rootA] > rank[rootB]) {
        parent[rootB] = rootA;
    } else {
        parent[rootA] = rootB;
        rank[rootB]++;
    }
}

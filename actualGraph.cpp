#include "Graph.h"
#include <fstream>
#include <sstream>
#include <iostream>

Graph::Graph(const char* const & edgelist_csv_fn) {
    std::ifstream file(edgelist_csv_fn);
    std::string line;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string u, v, w;
        std::getline(ss, u, ',');
        std::getline(ss, v, ',');
        std::getline(ss, w, ',');

        adjacencyList[u].push_back(std::make_tuple(u, v, std::stod(w)));
        adjacencyList[v].push_back(std::make_tuple(v, u, std::stod(w)));
    }
}

unsigned int Graph::num_nodes() {
    return adjacencyList.size();
}

std::vector<std::string> Graph::nodes() {
    std::vector<std::string> nodeLabels;
    for (const auto& pair : adjacencyList) {
        nodeLabels.push_back(pair.first);
    }
    return nodeLabels;
}

unsigned int Graph::num_edges() {
    unsigned int count = 0;
    for (const auto& pair : adjacencyList) {
        count += pair.second.size();
    }
    return count / 2; // Divide by 2 since edges are stored twice for each node
}

double Graph::edge_weight(const std::string& u_label, const std::string& v_label) {
    if (adjacencyList.count(u_label) == 0 || adjacencyList.count(v_label) == 0) {
        return -1; // Node does not exist
    }

    const auto& neighbors = adjacencyList[u_label];
    for (const auto& neighbor : neighbors) {
        if (std::get<1>(neighbor) == v_label) {
            return std::get<2>(neighbor); // Found the edge
        }
    }

    return -1; // Edge does not exist
}

unsigned int Graph::num_neighbors(const std::string& node_label) {
    if (adjacencyList.count(node_label) == 0) {
        return 0; // Node does not exist
    }

    return adjacencyList[node_label].size();
}

std::vector<std::string> Graph::neighbors(const std::string& node_label) {
    std::vector<std::string> neighborLabels;

    if (adjacencyList.count(node_label) == 0) {
        return neighborLabels; // Node does not exist
    }

    const auto& neighbors = adjacencyList[node_label];
    for (const auto& neighbor : neighbors) {
        neighborLabels.push_back(std::get<1>(neighbor));
    }

    return neighborLabels;
}

std::vector<std::string> Graph::shortest_path_unweighted(const std::string& start_label, const std::string& end_label) {
    std::unordered_map<std::string, bool> visited;
    std::unordered_map<std::string, std::string> parent;

    std::queue<std::string> queue;
    queue.push(start_label);
    visited[start_label] = true;

    while (!queue.empty()) {
        std::string current = queue.front();
        queue.pop();

        if (current == end_label) {
            std::vector<std::string> path;
            while (current != start_label) {
                path.insert(path.begin(), current);
                current = parent[current];
            }
            path.insert(path.begin(), start_label);
            return path;
        }

        const auto& neighbors = adjacencyList[current];
        for (const auto& neighbor : neighbors) {
            std::string neighborLabel = std::get<1>(neighbor);
            if (!visited[neighborLabel]) {
                queue.push(neighborLabel);
                visited[neighborLabel] = true;
                parent[neighborLabel] = current;
            }
        }
    }

    return std::vector<std::string>(); // No path found
}

//
     // Dijkstra's algorithm
    unordered_map<string, bool> visited;
    unordered_map<string, double> distance;
    unordered_map<string, string> parent;
    priority_queue<pair<double, string>, vector<pair<double, string>>, greater<pair<double, string>>> pq;
    vector<tuple<string, string, double>> path;

    pq.push({0.0, start_label});
    distance[start_label] = 0.0;

    while (!pq.empty()) {
        string currNode = pq.top().second;
        pq.pop();

        if (currNode == end_label) {
            // Reconstruct the path from end_label to start_label
            string node = end_label;
            while (node != start_label) {
                string parentNode = parent[node];
                // double edgeWeight = get_edge_weight(parentNode, node);
                // double edgeWeight = adjacencyList
                path.insert(path.begin(), make_tuple(parentNode, node, edgeWeight));
                node = parentNode;
            }
            path.insert(path.begin(), make_tuple(start_label, start_label, -1.0));
            return path;
        }

        if (visited[currNode])
            continue;

        visited[currNode] = true;

        for (auto& neighbor : adjacencyList[currNode]) {
            string neighborNode = get<1>(neighbor);
            double edgeWeight = get<2>(neighbor);

            double newDistance = distance[currNode] + edgeWeight;
            if (!distance.count(neighborNode) || newDistance < distance[neighborNode]) {
                distance[neighborNode] = newDistance;
                parent[neighborNode] = currNode;
                pq.push({newDistance, neighborNode});
            }
        }
    }

    // No path found
    return path;
//
std::vector<std::tuple<std::string, std::string, double>> Graph::shortest_path_weighted(const std::string& start_label, const std::string& end_label) {
    std::priority_queue<std::tuple<double, std::string, std::string>> pq;
    std::unordered_map<std::string, double> distances;
    std::unordered_map<std::string, std::string> parent;

    for (const auto& pair : adjacencyList) {
        const std::string& nodeLabel = pair.first;
        distances[nodeLabel] = std::numeric_limits<double>::infinity();
    }

    pq.push(std::make_tuple(0.0, start_label, start_label));
    distances[start_label] = 0.0;

    while (!pq.empty()) {
        auto current = pq.top();
        pq.pop();

        std::string currentLabel = std::get<1>(current);
        if (currentLabel == end_label) {
            std::vector<std::tuple<std::string, std::string, double>> path;
            std::string node = end_label;

            while (node != start_label) {
                std::string parentLabel = parent[node];
                double weight = edge_weight(parentLabel, node);
                path.insert(path.begin(), std::make_tuple(parentLabel, node, weight));
                node = parentLabel;
            }

            return path;
        }

        const auto& neighbors = adjacencyList[currentLabel];
        for (const auto& neighbor : neighbors) {
            std::string neighborLabel = std::get<1>(neighbor);
            double weight = std::get<2>(neighbor);
            double newDistance = distances[currentLabel] + weight;

            if (newDistance < distances[neighborLabel]) {
                distances[neighborLabel] = newDistance;
                parent[neighborLabel] = currentLabel;
                pq.push(std::make_tuple(-newDistance, neighborLabel, currentLabel));
            }
        }
    }

    return std::vector<std::tuple<std::string, std::string, double>>(); // No path found
}

std::vector<std::vector<std::string>> Graph::connected_components(const double& threshold) {
    std::unordered_map<std::string, bool> visited;
    std::vector<std::vector<std::string>> components;

    for (const auto& pair : adjacencyList) {
        const std::string& nodeLabel = pair.first;
        visited[nodeLabel] = false;
    }

    for (const auto& pair : adjacencyList) {
        const std::string& nodeLabel = pair.first;
        if (!visited[nodeLabel]) {
            std::vector<std::string> component;
            std::stack<std::string> stack;
            stack.push(nodeLabel);
            visited[nodeLabel] = true;

            while (!stack.empty()) {
                std::string current = stack.top();
                stack.pop();
                component.push_back(current);

                const auto& neighbors = adjacencyList[current];
                for (const auto& neighbor : neighbors) {
                    double weight = std::get<2>(neighbor);
                    if (weight <= threshold) {
                        std::string neighborLabel = std::get<1>(neighbor);
                        if (!visited[neighborLabel]) {
                            stack.push(neighborLabel);
                            visited[neighborLabel] = true;
                        }
                    }
                }
            }

            components.push_back(component);
        }
    }

    return components;
}

//
double Graph::smallest_connecting_threshold(string const & start_label, string const & end_label) {
    double minThreshold = std::numeric_limits<double>::max();
    double maxThreshold = std::numeric_limits<double>::lowest();

    // Find the minimum and maximum edge weights in the graph
    for (const auto& pair : adjacencyList) {
        const auto& neighbors = pair.second;
        for (const auto& neighbor : neighbors) {
            double weight = get<2>(neighbor);
            if (weight < minThreshold) {
                minThreshold = weight;
            }
            if (weight > maxThreshold) {
                maxThreshold = weight;
            }
        }
    }

    double smallestThreshold = maxThreshold;

    // Perform binary search to find the smallest threshold that connects the start and end nodes
    while (minThreshold <= maxThreshold) {
        double midThreshold = (minThreshold + maxThreshold) / 2;

        vector<tuple<string, string, double>> path = shortest_path_weighted(start_label, end_label);

        if (!path.empty()) {
            // A path exists with the current threshold, update the smallest threshold and continue searching
            smallestThreshold = midThreshold;
            maxThreshold = midThreshold - 1;
        } else {
            // No path exists with the current threshold, increase the threshold and continue searching
            minThreshold = midThreshold + 1;
        }
    }

    return smallestThreshold;
}

//

double Graph::smallest_connecting_threshold(string const & start_label, string const & end_label) {
    if (start_label == end_label) {
        return 0;
    }
    
    // Perform a breadth-first search (BFS) to find the smallest connecting threshold
    unordered_map<string, bool> visited;
    unordered_map<string, double> threshold;
    queue<string> q;

    // Initialize all nodes as unvisited and set their thresholds to infinity
    for (const auto& node : nodes) {
        visited[node.label] = false;
        threshold[node.label] = numeric_limits<double>::infinity();
    }

    // Start node
    visited[start_label] = true;
    threshold[start_label] = 0;
    q.push(start_label);

    // BFS
    while (!q.empty()) {
        string current_label = q.front();
        q.pop();

        // Check if the current node is the destination node
        if (current_label == end_label) {
            return threshold[current_label];  // Return the smallest connecting threshold
        }

        // Visit all neighboring nodes
        for (const auto& neighbor : get_neighbors(current_label)) {
            if (!visited[neighbor.label] && neighbor.threshold <= threshold[current_label]) {
                visited[neighbor.label] = true;
                threshold[neighbor.label] = neighbor.threshold;
                q.push(neighbor.label);
            }
        }
    }

    // If no path exists between the start and end nodes, return -1
    return -1;
}



#include "Graph.h"
#include <fstream>
#include <sstream>
#include <queue>
#include <unordered_map>

using namespace std;

Graph::Graph(const char* const & edgelist_csv_fn) {
    ifstream file(edgelist_csv_fn);
    if (!file.is_open()) {
        throw runtime_error("Failed to open the edge list file.");
    }

    string line;
    while (getline(file, line)) {
        stringstream ss(line);
        string u_label, v_label;
        double weight;

        if (getline(ss, u_label, ',') && getline(ss, v_label, ',') && (ss >> weight)) {
            // Add the edge to the graph
            edges[u_label].push_back(make_pair(v_label, weight));
            edges[v_label].push_back(make_pair(u_label, weight));
        }
    }

    file.close();
}

unsigned int Graph::num_nodes() {
    return edges.size();
}

vector<string> Graph::nodes() {
    vector<string> node_labels;
    for (const auto& entry : edges) {
        node_labels.push_back(entry.first);
    }
    return node_labels;
}

unsigned int Graph::num_edges() {
    unsigned int count = 0;
    for (const auto& entry : edges) {
        count += entry.second.size();
    }
    // Divide by 2 since each edge is counted twice (for both directions)
    return count / 2;
}

unsigned int Graph::num_neighbors(string const & node_label) {
    if (edges.count(node_label) == 0) {
        throw out_of_range("Node label does not exist in the graph.");
    }
    return edges[node_label].size();
}

double Graph::edge_weight(string const & u_label, string const & v_label) {
    if (edges.count(u_label) == 0 || edges.count(v_label) == 0) {
        return -1;
    }
    for (const auto& neighbor : edges[u_label]) {
        if (neighbor.first == v_label) {
            return neighbor.second;
        }
    }
    return -1;
}

vector<string> Graph::neighbors(string const & node_label) {
    if (edges.count(node_label) == 0) {
        throw out_of_range("Node label does not exist in the graph.");
    }
    vector<string> neighbor_labels;
    for (const auto& neighbor : edges[node_label]) {
        neighbor_labels.push_back(neighbor.first);
    }
    return neighbor_labels;
}

vector<string> Graph::shortest_path_unweighted(string const & start_label, string const & end_label) {
    unordered_map<string, string> prev_node;
    prev_node[start_label] = start_label;

    queue<string> q;
    q.push(start_label);

    while (!q.empty()) {
        string current_label = q.front();
        q.pop();

        if (current_label == end_label) {
            break;
        }

        for (const auto& neighbor : edges[current_label]) {
            string neighbor_label = neighbor.first;
            if (prev_node.count(neighbor_label) == 0) {
                prev_node[neighbor_label] = current_label;
                q.push(neighbor_label);
            }
        }
    }

    if (prev_node.count(end_label) == 0) {
        return {};  // No path found
    }

    // Reconstruct the path from end to start
    vector<string> path;
    string current = end_label;
    while (current != start_label) {
        path.push_back(current);
        current = prev_node[current];
    }
    path.push_back(start_label);

    // Reverse the path to get the correct order
    reverse(path.begin(), path.end());

    return path;
}

vector<tuple<string,string,double>> Graph::shortest_path_weighted(string const & start_label, string const & end_label) {
    unordered_map<string, double> distances;
    unordered_map<string, string> prev_node;
    for (const auto& entry : edges) {
        string node_label = entry.first;
        distances[node_label] = numeric_limits<double>::infinity();
        prev_node[node_label] = "";
    }
    distances[start_label] = 0;

    priority_queue<pair<double, string>, vector<pair<double, string>>, greater<pair<double, string>>> pq;
    pq.push(make_pair(0, start_label));

    while (!pq.empty()) {
        string current_label = pq.top().second;
        double current_distance = pq.top().first;
        pq.pop();

        if (current_label == end_label) {
            break;
        }

        if (current_distance > distances[current_label]) {
            continue;
        }

        for (const auto& neighbor : edges[current_label]) {
            string neighbor_label = neighbor.first;
            double edge_weight = neighbor.second;
            double new_distance = current_distance + edge_weight;
            if (new_distance < distances[neighbor_label]) {
                distances[neighbor_label] = new_distance;
                prev_node[neighbor_label] = current_label;
                pq.push(make_pair(new_distance, neighbor_label));
            }
        }
    }

    if (prev_node.count(end_label) == 0) {
        return {};  // No path found
    }

    // Reconstruct the path from end to start
    vector<tuple<string, string, double>> path;
    string current = end_label;
    while (current != start_label) {
        string previous = prev_node[current];
        double weight = edge_weight(previous, current);
        path.push_back(make_tuple(previous, current, weight));
        current = previous;
    }
    reverse(path.begin(), path.end());

    return path;
}

vector<vector<string>> Graph::connected_components(double const & threshold) {
    vector<vector<string>> components;
    unordered_map<string, bool> visited;
    for (const auto& entry : edges) {
        visited[entry.first] = false;
    }

    for (const auto& entry : edges) {
        string node_label = entry.first;
        if (!visited[node_label]) {
            vector<string> component;
            dfs_connected_components(node_label, threshold, visited, component);
            components.push_back(component);
        }
    }

    return components;
}

void Graph::dfs_connected_components(string const & node_label, double const & threshold, unordered_map<string, bool>& visited, vector<string>& component) {
    visited[node_label] = true;
    component.push_back(node_label);

    for (const auto& neighbor : edges[node_label]) {
        string neighbor_label = neighbor.first;
        double edge_weight = neighbor.second;
        if (!visited[neighbor_label] && edge_weight <= threshold) {
            dfs_connected_components(neighbor_label, threshold, visited, component);
        }
    }
}

double Graph::smallest_connecting_threshold(string const & start_label, string const & end_label) {
    double min_threshold = numeric_limits<double>::infinity();
    double max_threshold = numeric_limits<double>::infinity();

    for (const auto& entry : edges[start_label]) {
        if (entry.first == end_label) {
            min_threshold = 0.0;
            break;
        }
        if (entry.second < max_threshold) {
            max_threshold = entry.second;
        }
    }

    return (min_threshold == 0.0) ? min_threshold : max_threshold;
}


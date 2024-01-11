#include <iostream>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <set>

class Graph {
public:
    Graph(const char* const & edgelist_csv_fn) {
        loadGraphFromFile(edgelist_csv_fn);
    }

    unsigned int num_nodes() {
        return node_labels.size();
    }

    std::vector<std::string> nodes() {
        std::vector<std::string> node_list;
        for (const auto& pair : node_labels) {
            node_list.push_back(pair.first);
        }
        return node_list;
    }

    unsigned int num_edges() {
        return edges.size();
    }

    double edge_weight(const std::string& u_label, const std::string& v_label) {
        const std::string key = generateEdgeKey(u_label, v_label);
        if (edges.count(key) > 0) {
            return edges[key];
        }
        return -1.0; // No edge found
    }

    unsigned int num_neighbors(const std::string& node_label) {
        if (adjacency_list.count(node_label) > 0) {
            return adjacency_list[node_label].size();
        }
        return 0;
    }

    std::vector<std::string> neighbors(const std::string& node_label) {
        std::vector<std::string> neighbor_list;
        if (adjacency_list.count(node_label) > 0) {
            const auto& neighbors = adjacency_list[node_label];
            for (const std::string& neighbor : neighbors) {
                neighbor_list.push_back(neighbor);
            }
        }
        return neighbor_list;
    }

    std::vector<std::string> shortest_path_unweighted(const std::string& start_label, const std::string& end_label) {
        if (start_label == end_label) {
            return { start_label };
        }

        std::unordered_map<std::string, std::string> parent_map;
        std::unordered_map<std::string, bool> visited;
        std::queue<std::string> queue;

        visited[start_label] = true;
        queue.push(start_label);

        while (!queue.empty()) {
            std::string current = queue.front();
            queue.pop();

            if (current == end_label) {
                break;
            }

            if (adjacency_list.count(current) > 0) {
                const auto& neighbors = adjacency_list[current];
                for (const std::string& neighbor : neighbors) {
                    if (!visited[neighbor]) {
                        visited[neighbor] = true;
                        parent_map[neighbor] = current;
                        queue.push(neighbor);
                    }
                }
            }
        }

        if (visited[end_label]) {
            std::vector<std::string> path;
            std::string current = end_label;

            while (current != start_label) {
                path.push_back(current);
                current = parent_map[current];
            }

            path.push_back(start_label);
            std::reverse(path.begin(), path.end());

            return path;
        }

        return {}; // Empty path
    }

    std::vector<std::tuple<std::string, std::string, double>> shortest_path_weighted(const std::string& start_label, const std::string& end_label) {
        if (start_label == end_label) {
            return { std::make_tuple(start_label, start_label, -1.0) };
        }

        std::unordered_map<std::string, double> distance;
        std::unordered_map<std::string, std::string> parent_map;
        std::unordered_map<std::string, bool> visited;

        for (const auto& pair : node_labels) {
            distance[pair.first] = std::numeric_limits<double>::infinity();
            parent_map[pair.first] = "";
            visited[pair.first] = false;
        }

        distance[start_label] = 0.0;

        while (true) {
            std::string current_node = getMinDistanceNode(distance, visited);
            if (current_node.empty()) {
                break; // No more nodes to visit
            }

            visited[current_node] = true;

            if (current_node == end_label) {
                break;
            }

            if (adjacency_list.count(current_node) > 0) {
                const auto& neighbors = adjacency_list[current_node];
                for (const std::string& neighbor : neighbors) {
                    if (!visited[neighbor]) {
                        double edge_wt = edge_weight(current_node, neighbor);
                        double total_distance = distance[current_node] + edge_wt;

                        if (total_distance < distance[neighbor]) {
                            distance[neighbor] = total_distance;
                            parent_map[neighbor] = current_node;
                        }
                    }
                }
            }
        }

        std::vector<std::tuple<std::string, std::string, double>> path;

        if (parent_map[end_label].empty()) {
            return path; // Empty path
        }

        std::string current = end_label;

        while (current != start_label) {
            std::string parent = parent_map[current];
            double edge_wt = edge_weight(parent, current);
            path.push_back(std::make_tuple(parent, current, edge_wt));
            current = parent;
        }

        std::reverse(path.begin(), path.end());

        return path;
    }

    std::vector<std::vector<std::string>> connected_components(double const & threshold) {
        std::unordered_map<std::string, bool> visited;
        std::vector<std::vector<std::string>> components;

        for (const auto& pair : node_labels) {
            visited[pair.first] = false;
        }

        for (const auto& pair : node_labels) {
            std::string current_node = pair.first;
            if (!visited[current_node]) {
                std::vector<std::string> component;
                DFS(current_node, visited, threshold, component);
                components.push_back(component);
            }
        }

        return components;
    }

    double smallest_connecting_threshold(const std::string& start_label, const std::string& end_label) {
        double min_threshold = std::numeric_limits<double>::infinity();

        std::unordered_map<std::string, bool> visited;
        std::queue<std::string> queue;

        visited[start_label] = true;
        queue.push(start_label);

        while (!queue.empty()) {
            std::string current = queue.front();
            queue.pop();

            if (current == end_label) {
                break;
            }

            if (adjacency_list.count(current) > 0) {
                const auto& neighbors = adjacency_list[current];
                for (const std::string& neighbor : neighbors) {
                    double edge_wt = edge_weight(current, neighbor);
                    if (edge_wt <= min_threshold && !visited[neighbor]) {
                        visited[neighbor] = true;
                        queue.push(neighbor);
                    }
                }
            }
        }

        if (visited[end_label]) {
            return min_threshold;
        }

        return -1.0; // No path found
    }

private:
    std::unordered_map<std::string, double> edges;
    std::unordered_map<std::string, std::set<std::string>> adjacency_list;
    std::unordered_map<std::string, bool> node_labels;

    std::string generateEdgeKey(const std::string& u_label, const std::string& v_label) {
        return u_label + "-" + v_label;
    }

    void loadGraphFromFile(const char* const & edgelist_csv_fn) {
        std::ifstream file(edgelist_csv_fn);
        std::string line;

        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string u_label, v_label;
            double weight;

            if (!(iss >> u_label >> v_label >> weight)) {
                break; // Error parsing line
            }

            edges[generateEdgeKey(u_label, v_label)] = weight;
            adjacency_list[u_label].insert(v_label);
            adjacency_list[v_label].insert(u_label);
            node_labels[u_label] = true;
            node_labels[v_label] = true;
        }

        file.close();
    }

    std::string getMinDistanceNode(const std::unordered_map<std::string, double>& distance, const std::unordered_map<std::string, bool>& visited) {
        double min_distance = std::numeric_limits<double>::infinity();
        std::string min_node = "";

        for (const auto& pair : distance) {
            const std::string& node = pair.first;
            double dist = pair.second;

            if (!visited.at(node) && dist < min_distance) {
                min_distance = dist;
                min_node = node;
            }
        }

        return min_node;
    }

    void DFS(const std::string& current_node, std::unordered_map<std::string, bool>& visited, double const & threshold, std::vector<std::string>& component) {
        visited[current_node] = true;
        component.push_back(current_node);

        if (adjacency_list.count(current_node) > 0) {
            const auto& neighbors = adjacency_list[current_node];
            for (const std::string& neighbor : neighbors) {
                double edge_wt = edge_weight(current_node, neighbor);
                if (edge_wt >= threshold && !visited[neighbor]) {
                    DFS(neighbor, visited, threshold, component);
                }
            }
        }
    }
};

// int main() {
//     // Example usage
//     Graph graph("edgelist.csv");

//     std::cout << "Number of nodes: " << graph.num_nodes() << std::endl;
//     std::cout << "Number of edges: " << graph.num_edges() << std::endl;

//     std::cout << "Nodes: ";
//     for (const std::string& node : graph.nodes()) {
//         std::cout << node << " ";
//     }
//     std::cout << std::endl;

//     std::cout << "Neighbors of 'A': ";
//     for (const std::string& neighbor : graph.neighbors("A")) {
//         std::cout << neighbor << " ";
//     }
//     std::cout << std::endl;

//     double weight = graph.edge_weight("A", "B");
//     if (weight != -1.0) {
//         std::cout << "Weight of edge (A, B): " << weight << std::endl;
//     } else {
//         std::cout << "No edge found between A and B" << std::endl;
//     }

//     std::cout << "Shortest path (unweighted) from A to D: ";
//     for (const std::string& node : graph.shortest_path_unweighted("A", "D")) {
//         std::cout << node << " ";
//     }
//     std::cout << std::endl;

//     std::cout << "Shortest path (weighted) from A to D: ";
//     for (const auto& tuple : graph.shortest_path_weighted("A", "D")) {
//         std::cout << "(" << std::get<0>(tuple) << ", " << std::get<1>(tuple) << ", " << std::get<2>(tuple) << ") ";
//     }
//     std::cout << std::endl;

//     std::cout << "Connected components (threshold=2.5):" << std::endl;
//     std::vector<std::vector<std::string>> components = graph.connected_components(2.5);
//     for (const auto& component : components) {
//         for (const std::string& node : component) {
//             std::cout << node << " ";
//         }
//         std::cout << std::endl;
//     }

//     double connect_threshold = graph.smallest_connecting_threshold("A", "D");
//     if (connect_threshold != -1.0) {
//         std::cout << "Smallest connecting threshold between A and D: " << connect_threshold << std::endl;
//     } else {
//         std::cout << "No path found between A and D" << std::endl;
//     }

//     return 0;
// }


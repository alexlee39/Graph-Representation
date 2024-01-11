#include <fstream>
#include <sstream>
#include <unordered_map>
#include <set>
#include <queue>

class Graph {
    private:
        struct Edge {
            std::string u;
            std::string v;
            double weight;

            Edge(const std::string& node1, const std::string& node2, double edgeWeight)
                : u(node1), v(node2), weight(edgeWeight) {}
        };

        std::unordered_map<std::string, std::vector<Edge>> adjacencyList;

        void addEdge(const std::string& node1, const std::string& node2, double weight) {
            Edge edge1(node1, node2, weight);
            Edge edge2(node2, node1, weight);
            adjacencyList[node1].push_back(edge1);
            adjacencyList[node2].push_back(edge2);
        }

    public:
        Graph(const char* const & edgelist_csv_fn) {
            std::ifstream file(edgelist_csv_fn);
            std::string line;

            while (std::getline(file, line)) {
                std::istringstream iss(line);
                std::string node1, node2;
                double weight;
                std::getline(iss, node1, ',');
                std::getline(iss, node2, ',');
                iss >> weight;

                addEdge(node1, node2, weight);
            }

            file.close();
        }

        unsigned int num_nodes() {
            return adjacencyList.size();
        }

        std::vector<std::string> nodes() {
            std::vector<std::string> nodeLabels;
            for (const auto& pair : adjacencyList) {
                nodeLabels.push_back(pair.first);
            }
            return nodeLabels;
        }

        unsigned int num_edges() {
            unsigned int count = 0;
            for (const auto& pair : adjacencyList) {
                count += pair.second.size();
            }
            return count / 2; // Divide by 2 to account for undirected edges
        }

        double edge_weight(const std::string& u_label, const std::string& v_label) {
            if (adjacencyList.count(u_label) && adjacencyList.count(v_label)) {
                const auto& edges = adjacencyList[u_label];
                for (const auto& edge : edges) {
                    if (edge.v == v_label) {
                        return edge.weight;
                    }
                }
            }
            return -1;
        }

        unsigned int num_neighbors(const std::string& node_label) {
            if (adjacencyList.count(node_label)) {
                return adjacencyList[node_label].size();
            }
            return 0;
        }

        std::vector<std::string> neighbors(const std::string& node_label) {
            std::vector<std::string> neighborLabels;
            if (adjacencyList.count(node_label)) {
                const auto& edges = adjacencyList[node_label];
                for (const auto& edge : edges) {
                    neighborLabels.push_back(edge.v);
                }
            }
            return neighborLabels;
        }

        std::vector<std::string> shortest_path_unweighted(const std::string& start_label, const std::string& end_label) {
            if (start_label == end_label) {
                return {start_label};
            }

            std::unordered_map<std::string, std::string> previous;
            std::queue<std::string> queue;
            std::set<std::string> visited;

            previous[start_label] = "";
            queue.push(start_label);
            visited.insert(start_label);

            while (!queue.empty()) {
                std::string current = queue.front();
                queue.pop();

                if


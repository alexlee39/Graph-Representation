#include <unordered_map>
#include <string>
using namespace std;

class Disjoint{
private:
    unordered_map<string, string> parent;  // Maps a node to its parent node
    unordered_map<string, int> rank;  // Maps a node to its rank

public:
    // Creates a set with a single element
    void makeSet (const string& node);

    // Finds the representative (root) of the set that the given node belongs to
    string find(const string& node);

    // Unites two sets together
    void unite(const string& nodeA, const string& nodeB);
};



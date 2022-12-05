#include <vector>
#include <string>

typedef std::string string;

// "Graph" is not a suitable name for this class because we have multiple graphs per fuel type
class Algorithms {
    public:
    /**
    * Dijkstra's Algorithm
    *
    * Given a start node and an adjacency matrix, returns a vector of the minimum distances
    * from start to nodes 0 to n, where n is the number of nodes in the graph of the start node's fuel type
    * 
    * @param start starting node
    * @param graph adjacency matrix for given fuel type (as indicated by fuel type of start node)
    * @return vector which holds the shortest distances from start to node i, where i is ith index of vector
    */
    std::vector<int> findShortest(Node& start, const std::vector<std::vector<int>>& graph);

    // next: create a mapping from std::vector<int> to std::vector<Node>

    // function takes in data.csv and creates graph from it
    
    // TO-DO
    // - Prim's and Kruskal's
    // - Constructor for each fuel type (one graph per fuel type)
    // - For each fuel type, we are going to need to assign an integer value from 0 to n for each node
    //      so that we can create an adjacency matrix; the adjacency matrix holds the weights of the edges
    //      (for Dijkstra's, weights are distances)
    // - How are we going to create an adjacency matrix for each fuel type? Are we going to do this in main()?
    //      Or will we create a class for this? I am unsure how to go about this. Also, I think we should
    //      use two dimensional vectors as opposed to arrays since this will be simpler imo
    
    private:
        /*  
            map from fuelType to number of nodes in graph corresponding to that fuelType 
            (we find the number of nodes for the fuelType during contruction of graph)
        */
        // std::map<std::string, int> map; // need a better name for this

        std::map<std::string, std::vector<Node>> map; // map from fuelType to vector of nodes of that fuel type

        std::vector<Node> vector; // vector of all nodes that we will read in from csv file

        struct Node {
            std::string fuelType, streetAddress, state;
            double latitude, longitude;
        };
};
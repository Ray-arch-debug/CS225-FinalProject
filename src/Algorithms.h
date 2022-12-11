/**
 * @file Algorithms.h
 * Implementation of Dijkstra's and MST algorithms on graphs.
 *
 * @author Brendan Biernacki
 * @author Shaurya Singh
 * @author Alexander Wang
 * @author Aakash Kumar
 */

#pragma once

#include <fstream>
#include <vector>
#include <string>
#include <set>
#include <map>

using namespace std;

const double ROAD_CURVINESS_QUOTIENT = 1.24; // https://gispoint.de/fileadmin/user_upload/paper_gis_open/537521034.pdf
const double MILES_PER_DEGREE_LATITUDE = 69.055;
const double MILES_PER_DEGREE_LONGITUDE = 54.6;

struct Node {

    unsigned index;
    std::string fuelType;
    std::string streetAddress;
    std::string state;
    double latitude;
    double longitude;

    bool operator<(const Node& other) const {
        return index < other.index;
    };

    bool operator==(const Node& other) const {
        return index == other.index; // assume indices are distinct
    }

};

bool AreWithinRange(const Node& first, const Node& second, double range);

/*
ofstream& operator<<(ofstream& os, Node node) {
    os << node.index << "\t";
    os << node.fuelType << "\t";
    os << node.streetAddress << "\t";
    os << node.state << "\t";
    os << node.latitude << "\t";
    os << node.longitude << endl;
    return os;
}
*/

// "Graph" is not a suitable name for this class because we have multiple graphs per fuel type
class Algorithms {

    public:

    Algorithms(const string& filePath /*, const string& input_location  not sure about this */, const string& input_fuel_type, double input_range, bool useKDTree);

    // NEED TO WRITE A DESTRUCTOR FOR ALGORITHMS
    //~Algorithms() { delete graph_; }

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

    
    // TO-DO
    // - Prim's and Kruskal's
    // - Constructor for each fuel type (one graph per fuel type)
    // - For each fuel type, we are going to need to assign an integer value from 0 to n for each node
    //      so that we can create an adjacency matrix; the adjacency matrix holds the weights of the edges
    //      (for Dijkstra's, weights are distances)
    // - How are we going to create an adjacency matrix for each fuel type? Are we going to do this in main()?
    //      Or will we create a class for this? I am unsure how to go about this. Also, I think we should
    //      use two dimensional vectors as opposed to arrays since this will be simpler imo

    // helper for testing
    const set<Node>& GetVertices() const;

    // helper for testing
    const map<Node, set<Node>>& GetGraph() const;

    
    private:

        class Graph { // possibly make Node* instead of Node

            public:
                //Graph() = default;
                Graph(const std::string& filePath, const string& input_fuel_type, double input_range, bool useKDTree);
                //Graph(const Graph& other) = delete;
                //Graph& operator=(const Graph& other) = default;
                //~Graph() = default;
                const set<Node>& GetVertices() const;
                const set<Node>& GetNeighbors(const Node& node) const;

                // helper for testing
                const map<Node, set<Node>>& GetGraph() const;



            private:

                /**
                * Creates a graph from the csv at the given filePath. Uses the naive approach where all pairs
                * of vertices are checked to determine if they are connected by an edge. The expected runtime
                * is O(n^2).
                *
                * @param filePath the relative path of the csv file.
                */
                void createGraphNaive(const std::string& filePath);

                /**
                * Creates a graph from the csv at the given filePath. Uses the KDTree approach where only
                * the pairs of vertices that are adjacent are ever visited. The expected runtime is O(n+m).
                *
                * @param filePath the relative path of the csv file.
                */
                void createGraphKDTree(const std::string& filePath);

                void populateVertices(const std::string& filePath);

                map<Node, set<Node>> graph_;
                set<Node> vertices_;
                string fuel_type_;
                double range_;
        };
        Graph graph_;
        //Graph* graph_ = NULL;
};
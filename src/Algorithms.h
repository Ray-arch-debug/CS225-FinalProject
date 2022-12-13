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
#include <iostream>
#include <limits>
#include <vector>
#include <string>
#include <set>
#include <map>

using namespace std;

const double ROAD_CURVINESS_QUOTIENT = 1.24; // Not exactly accurate but close. For more, see: https://gispoint.de/fileadmin/user_upload/paper_gis_open/537521034.pdf
const double MILES_PER_DEGREE_LATITUDE = 69.055;
const double MILES_PER_DEGREE_LONGITUDE = 54.6;
const unsigned long MAXIMUM_ADDRESS_LENGTH = 55;

struct Node {

    int index;
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
    };

    string ToString() const {
        string output;
        output += to_string(index);
        output += "\t";
        output += fuelType;
        output += "\t";
        output += streetAddress;
        for (size_t i = 0; i < (MAXIMUM_ADDRESS_LENGTH - streetAddress.length()) / 8; i++) {
            output += "\t";
        }
        output += "\t";
        output += state;
        output += "\t";
        output += to_string(latitude);
        output += "\t";
        output += to_string(longitude);
        return output;
    }

};


bool AreWithinRange(const Node& first, const Node& second, double range);

double FindDistance(const Node& first, const Node& second);


class Algorithms {

    public:

    Algorithms(const string& filePath, const string& input_fuel_type, double input_range, bool useKDTree);


    /**
    * Dijkstra's Algorithm
    *
    * Given a start node and an adjacency matrix, returns a vector of the minimum distances
    * from start to nodes 0 to n, where n is the number of nodes in the graph of the start node's fuel type
    * 
    * @return vector which holds the shortest distances from start to node i, where i is ith index of vector
    */
    std::vector<int> Dijkstra(int startCSVIdx, int endCSVIdx) const;

    const std::set<Node> BFS(Node start) const;

    const Node& nodefinder(const std::string& start) const;

    std::vector<int> findPath(int startCSVIdx, int endCSVIdx, const std::map<int, double>& distances) const;

    // helper for testing
    const set<Node>& GetVertices() const;

    // helper for testing
    const map<Node, set<Node>>& GetGraph() const;

    //void printDijkstras(std::vector<int> path) const;
    const Node& GetNode(int index) const;

    
    private:

        class Graph {

            public:
                Graph(const std::string& filePath, const string& input_fuel_type, double input_range, bool useKDTree);
                const set<Node>& GetVertices() const;
                const set<Node>& GetNeighbors(const Node& node) const;

                const Node& GetNode(int index) const;

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
                * the pairs of vertices that are adjacent are ever visited. The expected runtime is O(n*log(n)+m),
                * though I'm not 100% sure on this.
                *
                * @param filePath the relative path of the csv file.
                */
                void createGraphKDTree(const std::string& filePath);

                void populateVertices(const std::string& filePath);

                void populateMapping();

                // need a map from csv index to its node
                map<int, Node> index_map_;

                // use map from csv index to adjacency list (need for dijkstra's)
                //map<int, set<Node>> adjacency2_;
                
                map<Node, set<Node>> adjacency_;
                map<Node, set<Node>> graph_;
                set<Node> vertices_;
                string fuel_type_;
                double range_;
        };
        Graph graph_;
        const double maxDistance = numeric_limits<double>::max(); // represents infinity
        //std::map<int, double> distances_;
};
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
    * Given a starting destination and ending destination, Dijkstra's returns a vector of ints which 
    * stores the sequence of nodes of the shortest path from the starting destination to the ending destination
    * 
    * @param startCSVIdx CSV index of the starting destination (fuel station). CSV index comes from the CSV file.
    * @param endCSVIdx CSV index of the end destination (fuel station). CSV index comes from the CSV file.
    * @return vector of ints which stores the CSV indeces of the shortest path from startCSVIdx to endCSVIdx
    */
    std::vector<int> Dijkstra(int startCSVIdx, int endCSVIdx) const;

    /**
    * BFS (USED FOR FINDING RANGE FROM GIVEN LOCATION)
    *  
    * This function takes in the node of the starting location found using node finder. 
    * 
    * It then runs a bfs, checking if the newly visited node is in range and if it is 
    * you add it to the set of nodes you return and mark it as visited. 
    * 
    * If the visited node is not within range, dont add it to the return variable and 
    * mark it as visited.
    * 
    * The set of verticies can be compared with the total vertices for the fuel type 
    * in the data set to show how comprehensive the transport system is for that fuel type. 
    * A mismatch between the connected component (of the bfs) and the total set of nodes in data
    * show that there are gaps in the infrastructre. 
    * 
    * @param start The starting node, found using the 'nodefinder' function. 
    * @return a set of nodes containing all the visited nodes from the starting node. 
    */
    const std::set<Node> BFS(Node start) const;

    /**
    * NODE FINDER FUNCTION - 
    *  
    * This function takes in the starting location input by the user and finds the
    * corresponding node associated with it in the constructed graph variable. 
    * 
    * If no node is found then it outputs accordingly.
    * 
    * @param start The starting location address, std::string type. 
    * @return the node associated with the starting location. 
    */
    const Node& nodefinder(const std::string& start) const;

    // helper for testing
    const set<Node>& GetVertices() const;

    // helper for testing
    const map<Node, set<Node>>& GetGraph() const;

    const Node& GetNode(int index) const;

    
    private:
        /**
        * findPath (helper/extension of Dijkstras Algorithm)
        *
        * Given a starting location and ending destination, findpath returns a vector of ints which 
        * stores the sequence of nodes of the shortest path from the starting destination to the ending destination
        * (returns the same vector as Dijkstra). 
        * 
        * @param startCSVIdx CSV index of the starting location (fuel station). CSV index comes from the CSV file.
        * @param endCSVIdx CSV index of the end destination (fuel station). CSV index comes from the CSV file.
        * @param distances a map from CSV index to the shortest distance from that CSV index to the starting location. 
        * distances contains all the CSV indeces in the Shortest Path Tree.
        * @return vector of ints (called path) which stores the CSV indeces of the shortest path from startCSVIdx to endCSVIdx.
        */
        std::vector<int> findPath(int startCSVIdx, int endCSVIdx, const std::map<int, double>& distances) const;

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

                // map from csv index to its node; NEEDED FOR DIJKSTRAS
                map<int, Node> index_map_;
                
                map<Node, set<Node>> graph_;
                set<Node> vertices_;
                string fuel_type_;
                double range_;
        };
        Graph graph_;
        const double maxDistance = numeric_limits<double>::max(); // represents infinity
};
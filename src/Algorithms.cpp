/**
 * @file Algorithms.cpp
 * Implementation of Algorithms class.
 */

#include "Algorithms.h"
#include "kdtree.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits.h>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

bool AreWithinRange(const Node& first, const Node& second, double range) {
    double delta_latitude = first.latitude - second.latitude;
    double delta_longitude = first.longitude - second.longitude;
    double euclidean_distance = pow( pow(delta_latitude * MILES_PER_DEGREE_LATITUDE, 2) +
                                     pow(delta_longitude * MILES_PER_DEGREE_LONGITUDE, 2), 0.5 );
    return euclidean_distance < range / ROAD_CURVINESS_QUOTIENT;
}

std::vector<int> findShortest(Node& start, const std::vector<std::vector<int>>& graph) {
    /*
    int distances[graph.size()]; // value at index i represents shortest distance from start node to node i 
    // (reminder: each node is assigned an index value, as explained in Algorithms.h)
 
    bool set[graph.size()]; // sptSet[i] will be true if vertex i is
                    // included in shortest
    // path tree or shortest distance from src to i is
    // finalized 
 
    // Initialize all distances as INFINITE and stpSet[] as
    // false
    // set distances to be infinite and 
    for (int index = 0; index < graph.size(); ++index) {
        distances[index] = INT_MAX;
        set[index] = false;
    }
    // Distance of source vertex from itself is always 0
    dist[src] = 0;

    // Find shortest path for all vertices
    for (int count = 0; count < V - 1; count++) {
        // Pick the minimum distance vertex from the set of
        // vertices not yet processed. u is always equal to
        // src in the first iteration.
        int u = minDistance(dist, sptSet);
 
        // Mark the picked vertex as processed
        sptSet[u] = true;
 
        // Update dist value of the adjacent vertices of the
        // picked vertex.
        for (int v = 0; v < V; v++)
 
            // Update dist[v] only if is not in sptSet,
            // there is an edge from u to v, and total
            // weight of path from src to  v through u is
            // smaller than current value of dist[v]
            if (!sptSet[v] && graph[u][v]
                && dist[u] != INT_MAX
                && dist[u] + graph[u][v] < dist[v])
                dist[v] = dist[u] + graph[u][v];
    }
    */
    std::vector<int> v;
    return v;
}



Algorithms::Algorithms(const string& filePath, 
                       const string& input_fuel_type, 
                       double input_range, 
                       bool useKDTree) : 
                       graph_(filePath, input_fuel_type, input_range, useKDTree) {}

const set<Node>& Algorithms::GetVertices() const { return graph_.GetVertices(); }
const map<Node, set<Node>>& Algorithms::GetGraph() const { return graph_.GetGraph(); }

const set<Node>& Algorithms::Graph::GetVertices() const { return vertices_; }
const set<Node>& Algorithms::Graph::GetNeighbors(const Node& node) const {
    if (vertices_.find(node) != vertices_.end()) {
        return graph_.at(node);
    }
    throw invalid_argument("node not present in graph");
}
const map<Node, set<Node>>& Algorithms::Graph::GetGraph() const { return graph_; }


Algorithms::Graph::Graph(const std::string& filePath,
                         const string& input_fuel_type, 
                         double input_range,
                         bool useKDTree) :
                         fuel_type_(input_fuel_type), 
                         range_(input_range) {

    if (useKDTree) {
        createGraphKDTree(filePath);
    } else {
        createGraphNaive(filePath);
    }

}

void Algorithms::Graph::createGraphNaive(const std::string& filePath) {

    populateVertices(filePath);

    for (const Node& node : vertices_) {
        for (const Node& potential_neighbor : vertices_) {
            if (&node != &potential_neighbor && AreWithinRange(node, potential_neighbor, range_)) {
                graph_.at(node).insert(potential_neighbor);
            }
        }
    }

}

void Algorithms::Graph::createGraphKDTree(const std::string& filePath) {
    
    populateVertices(filePath);

    std::vector<Point<2>> points;
    std::map<Point<2>, const Node&> points_to_nodes;

    for (const Node& node : vertices_) {
        Point<2> point { MILES_PER_DEGREE_LATITUDE * node.latitude, MILES_PER_DEGREE_LONGITUDE * node.longitude };
        points.push_back(point);
        points_to_nodes.insert({point, node});
    }
    KDTree<2> kdtree(points);

    // may want to improve this later, Shaurya and I discussed doing a queue approach

    for (const Node& node : vertices_) {
        Point<2> point { MILES_PER_DEGREE_LATITUDE * node.latitude, MILES_PER_DEGREE_LONGITUDE * node.longitude };
        const vector<Point<2>>& neighbors = kdtree.findWithinDistance(point, range_ / ROAD_CURVINESS_QUOTIENT);
        for (const Point<2>& neighbor : neighbors) {
            if (point != neighbor) { 
                graph_.at(node).insert(points_to_nodes.at(neighbor));
            }
        }
    }

}

void Algorithms::Graph::populateVertices(const std::string& filePath) {

    fstream file(filePath, ios::in);

    if (!file.is_open()) {
        throw std::invalid_argument("file " + filePath + " did not open");
        return;
    }
    
    file.ignore(INT_MAX, '\n'); // ignore first line

    for (string line; getline(file, line); line.clear()) {
        vector<string> row;
        stringstream str(line);
        for (string word; getline(str, word, ','); word.clear()) {
            row.push_back(word);
        }

        if (row.size() != 6) { continue; }

        try {
            Node node;
            node.index         = stoi(row.at(0));
            node.fuelType      = row.at(1);
            node.streetAddress = row.at(2);
            node.state         = row.at(3);
            node.latitude      = stod(row.at(4));
            node.longitude     = stod(row.at(5));
            if (node.fuelType == fuel_type_) { 
                vertices_.insert(node);
                graph_.insert({node, set<Node>()}); 
            }
        } catch (std::invalid_argument& e) {
            vertices_.clear();
            graph_.clear();
            throw std::invalid_argument("input file " + filePath + " formatted incorrectly");
        }

    }
}

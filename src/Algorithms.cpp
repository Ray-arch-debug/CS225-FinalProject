/**
 * @file Algorithms.cpp
 * Implementation of Algorithms class.
 */

#include "Algorithms.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits.h>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

bool AreRelated(const Node& first, const Node& second, double range) {
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



Algorithms::Algorithms(const string& filePath, const string& input_fuel_type, double input_range) {
    graph_ = new Graph(filePath, input_fuel_type, input_range);
}

const set<Node>& Algorithms::GetVertices() { return graph_->GetVertices(); }
const map<Node, set<Node>>& Algorithms::GetGraph() { return graph_->GetGraph(); }

const set<Node>& Algorithms::Graph::GetVertices() { return vertices_; }
const set<Node>& Algorithms::Graph::GetNeighbors(const Node& node) {
    if (vertices_.find(node) != vertices_.end()) {
        return graph_.at(node);
    }
    throw invalid_argument("node not present in graph");
}
const map<Node, set<Node>>& Algorithms::Graph::GetGraph() { return graph_; }


Algorithms::Graph::Graph(const std::string& filePath,
                         const string& input_fuel_type, 
                         double input_range) :
                         fuel_type_(input_fuel_type), 
                         range_(input_range) {

    createGraphNaive(filePath);

}

void Algorithms::Graph::createGraphNaive(const std::string& filePath) {

    populateVertices(filePath);

    //unsigned count = 0;
    for (const Node& node : vertices_) {
        for (const Node& potential_neighbor : vertices_) {
            if (&node != &potential_neighbor && AreRelated(node, potential_neighbor, range_)) {
                graph_.at(node).insert(potential_neighbor);
                //std::cout << node.index << "\t" << potential_neighbor.index << std::endl;
                //count++;
            }
        }
    }
    //std::cout << count << std::endl;

}

void Algorithms::Graph::createGraphKDTree(const std::string& filePath) {
    
    populateVertices(filePath);

    // TODO

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

/**
 * @file Algorithms.cpp
 * Implementation of Algorithms class.
*/

#include "Algorithms.h"
#include "heap.h"
#include "kdtree.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits.h>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

/**
  * Dijkstra's Algorithm
  *
  * Given a starting location and ending destination, Dijkstra's returns a vector of ints which 
  * stores the sequence of nodes of the shortest path from the starting destination to the ending destination.
  * This function first creates a Shortest Path Tree (SPT) and stores the shortest distance from the starting
  * location within a map called distances. distances maps each CSV index in the SPT to its distance from
  * the start location. 
  * 
  * @param startCSVIdx CSV index of the starting location (fuel station). CSV index comes from the CSV file.
  * @param endCSVIdx CSV index of the end destination (fuel station). CSV index comes from the CSV file.
  * @return vector of ints which stores the CSV indeces of the shortest path from startCSVIdx to endCSVIdx.
*/
std::vector<int> Algorithms::Dijkstra(int startCSVIdx, int endCSVIdx) const {

    std::map<int, double> distances;
    int num_vertices = graph_.GetVertices().size();
    heap minHeap;

    for (const Node& node : graph_.GetVertices()) {
        int csvIdx = node.index;
        std::pair<int, double> pair(csvIdx, maxDistance);
        minHeap.push(pair);

        distances[csvIdx] = maxDistance;
    }

    distances[startCSVIdx] = 0.0;
    minHeap.updateDistance(startCSVIdx, 0.0);

    while (!minHeap.empty()) {
        const std::pair<int, double>& minHeapNode = minHeap.pop();
        int minHeapNodeCSVIdx = minHeapNode.first;

        const Node& currMinNode = graph_.GetNode(minHeapNodeCSVIdx);
        const std::set<Node>& neighbors = graph_.GetGraph().at(currMinNode);

        for (const Node& next : neighbors) {
            int nextNodeCSVIdx = next.index;
            
            double distanceFromCurrToNext = FindDistance(currMinNode, next);

            if (minHeap.isInHeap(nextNodeCSVIdx) && (distances[minHeapNodeCSVIdx] != maxDistance) && 
                (distanceFromCurrToNext + distances[minHeapNodeCSVIdx] < distances[nextNodeCSVIdx])) {
                distances[nextNodeCSVIdx] = distances[minHeapNodeCSVIdx] + distanceFromCurrToNext;
            
                // update distance in minHeap as well
                minHeap.updateDistance(nextNodeCSVIdx, distances[nextNodeCSVIdx]);
            }
        }
    }

    return findPath(startCSVIdx, endCSVIdx, distances);
}

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
std::vector<int> Algorithms::findPath(int startCSVIdx, int endCSVIdx, const std::map<int, double>& distances) const {
    std::vector<int> path;

    std::map<int, bool> visited;
    std::map<int, int> prevStorage;

    for (auto& [csvIndex, csvNode] : distances) {
        visited[csvIndex] = false;
    }

    std::queue<int> queue;

    visited[startCSVIdx] = true;
    queue.push(startCSVIdx);

    while (!queue.empty()) {
        int currCSVIdx = queue.front();

        if (queue.front() == endCSVIdx) { break; }

        for (const Node& neighbor : graph_.GetNeighbors(graph_.GetNode(currCSVIdx))) {
            if (!visited[neighbor.index]) {
                visited[neighbor.index] = true;
                queue.push(neighbor.index);
                prevStorage[neighbor.index] = currCSVIdx;
            }
        }
        queue.pop();
    }

    std::stack<int> stack;
    int currIdx = endCSVIdx;

    unsigned i = 0;
    while (currIdx != startCSVIdx) {
        stack.push(currIdx);
        currIdx = prevStorage[currIdx];
        i++;
        if (i > prevStorage.size()) {
            return std::vector<int>();
        }
    }
    stack.push(startCSVIdx);

    while (!stack.empty()) {
        path.push_back(stack.top());
        stack.pop();
    }

    return path;
}

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
const std::set<Node> Algorithms::BFS(Node start) const {
    std::set<Node> visited; 
    std::queue<Node> q;
 
    visited.insert(start);
    q.push(start);
 
    while (!q.empty()) {
        Node s = q.front();
        q.pop();
 
        for (auto adjacent : graph_.GetNeighbors(s)) {
            if (visited.find(adjacent) == visited.end()) {
                q.push(adjacent);
                visited.insert(adjacent);
            }
        }
    }

    return visited;
}

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
const Node& Algorithms::nodefinder(const std::string& start) const {
    const set<Node>& nodes = graph_.GetVertices();

    for (auto& node : nodes) {
        if (node.streetAddress == start) {
            return node;
        }
    }
    throw invalid_argument("node not found");
}

bool AreWithinRange(const Node& first, const Node& second, double range) {
    return FindDistance(first, second) < range / ROAD_CURVINESS_QUOTIENT;
}

double FindDistance(const Node& first, const Node& second) {
    double delta_latitude = first.latitude - second.latitude;
    double delta_longitude = first.longitude - second.longitude;
    return pow( pow(delta_latitude * MILES_PER_DEGREE_LATITUDE, 2) +
                                     pow(delta_longitude * MILES_PER_DEGREE_LONGITUDE, 2), 0.5 );
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


const Node& Algorithms::GetNode(int index) const { return graph_.GetNode(index); }


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

const Node& Algorithms::Graph::GetNode(int index) const {
    return index_map_.at(index);
}

void Algorithms::Graph::populateMapping() {
    for (const Node& node : vertices_) {
        index_map_.insert({ node.index, node });
    }
}


void Algorithms::Graph::populateVertices(const std::string& filePath) {

    fstream file(filePath, ios::in);

    if (!file.is_open()) {
        throw std::invalid_argument("file " + filePath + " did not open");
        return;
    }
    
    file.ignore(INT_MAX, '\n'); // ignore first line
    //MAXIMUM_ADDRESS_LENGTH = 0;

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

    populateMapping();
}

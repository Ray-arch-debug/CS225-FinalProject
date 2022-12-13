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

bool AreWithinRange(const Node& first, const Node& second, double range) {
    return FindDistance(first, second) < range / ROAD_CURVINESS_QUOTIENT;
}

double FindDistance(const Node& first, const Node& second) {
    double delta_latitude = first.latitude - second.latitude;
    double delta_longitude = first.longitude - second.longitude;
    return pow( pow(delta_latitude * MILES_PER_DEGREE_LATITUDE, 2) +
                                     pow(delta_longitude * MILES_PER_DEGREE_LONGITUDE, 2), 0.5 );
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



// dijkstra uses heap.cpp and heap.h; heap.cpp and heap.h have been edited to use 
// std::pair<std::string, double>, where first is node index and second is distance from start node
std::vector<int> Algorithms::Dijkstra(int startCSVIdx, int endCSVIdx) const {
    
    std::map<int, double> distances;
    int num_vertices = graph_.GetVertices().size();
    heap minHeap;

    for (const Node& node : graph_.GetVertices()) {
        int csvIdx = node.index;
        // actual heap
        std::pair<int, double> pair(csvIdx, maxDistance);
        minHeap.push(pair);

        // positions
        // minHeap->positions_[csvIdx] = num_vertices;

        // distances
        distances[csvIdx] = maxDistance;
    }

    distances[startCSVIdx] = 0.0;

    // std::pair<int, double> startNode(start.index, 0.0); // root of heap
    // minHeap->updateElem(startHeapIdx, startNode); // note to self: check if updateElem works properly in test case
    minHeap.updateDistance(startCSVIdx, 0.0); // function in heap.cpp

    while (!minHeap.empty()) {
        // assert that top of minHeap is startNode during the first iteration of while loop
        std::pair<int, double> minHeapNode = minHeap.pop();
        int minHeapNodeCSVIdx = minHeapNode.first;

        const Node& currMinNode = graph_.GetNode(minHeapNodeCSVIdx);

        const std::set<Node>& neighbors = graph_.GetGraph().at(currMinNode);

        for (const Node& next : neighbors) {
            int nextNodeCSVIdx = next.index;
            
            double distanceFromCurrToNext = FindDistance(currMinNode, next);

            if (minHeap.isInHeap(nextNodeCSVIdx) && (distances[minHeapNodeCSVIdx] != maxDistance) && 
                (distanceFromCurrToNext + distances[minHeapNodeCSVIdx] < distances[nextNodeCSVIdx])) {
                distances[nextNodeCSVIdx] = distances[minHeapNodeCSVIdx] + distanceFromCurrToNext;
            
                // updateDistance in minHeap as well
                minHeap.updateDistance(nextNodeCSVIdx, distances[nextNodeCSVIdx]);
            }
        }
    }

    // call helper function w/ start and end (use BFS)
    return findPath(startCSVIdx, endCSVIdx, distances);
}

const Node& Algorithms::Graph::GetNode(int index) const {
    return index_map_.at(index);
}

std::vector<int> Algorithms::findPath(int startCSVIdx, int endCSVIdx, const std::map<int, double>& distances) const {
    std::vector<int> path;
    std::map<int, bool> visited;

    std::map<int, int> prevStorage;

    for (auto& [csvIndex, csvNode] : distances) {
        //int csvIndex = it->first;
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

    // go backwards
    std::stack<int> stack;
    int currIdx = endCSVIdx;
    //for (std::map<int, int>::iterator it = prevStorage.begin(); it != prevStorage.end(); ++it) {
        //std::cout << "previous of " << it->first << " goes to " << it->second << std::endl;
    //}
    int i = 0;
    while (currIdx != startCSVIdx) {
        //std::cout << currIdx << std::endl;
        stack.push(currIdx);
        currIdx = prevStorage[currIdx];
        i++;
        if (i == 250) {
            //std::cout << "There is no path from starting node to ending node" << std::endl;
            return std::vector<int>();
        }
    }
    stack.push(startCSVIdx);

    //std::cout << stack.size() << std::endl;

    while (!stack.empty()) {
        //std::cout << stack.top() << std::endl;
        path.push_back(stack.top());
        stack.pop();
    }

    //printDijkstras(path);
    return path;
}

/*
void Algorithms::printDijkstras(std::vector<int> path) const {
    std::cout << "The sequence of nodes to go from " << path[0] << " to " << path[path.size() - 1] << " following Dijkstra's Algorithm is:" << std::endl;
    for (int vertex : path) {
        if (vertex == path[path.size() - 1]) {
            std::cout << vertex << std::endl;
        } else {
            std::cout << vertex << " --> ";
        }
    }
    std::cout << std::endl;
*/

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

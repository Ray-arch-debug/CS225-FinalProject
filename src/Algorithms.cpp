/**
 * @file Algorithms.cpp
 * Implementation of Algorithms class.
 */

#include "Algorithms.h"
#include <limits.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

using namespace std;

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

void Algorithms::createGraphs(std::string filePath) {
    vector<vector<string>> contents;
    vector<string> row;
    string line, word;
    
    fstream file(filePath, ios::in);

    if (!file.is_open()) {
        throw std::invalid_argument("file did not open");
        return;
    }
    
    while (getline(file, line)) {
        row.clear();
        stringstream str(line);
        while (getline(str, word, ',')) {
            row.push_back(word);
        }
        contents.push_back(row);
    }
    
    for (unsigned i = 1; i < contents.size(); i++) {
        if (contents[i].size() != 6) {
            continue;
        }
        Node node;
        try {
            node.index = stoi(contents[i][0]);
            node.fuelType = contents[i][1];
            node.streetAddress = contents[i][2];
            node.state = contents[i][3];
            node.latitude = stod(contents[i][4]);
            node.longitude = stod(contents[i][5]);
        } catch (std::invalid_argument& e) {
            throw std::invalid_argument("input file formatted incorrectly");
            nodes.clear();
            return;
        }
        nodes.push_back(node);
    }

    separateNodes();
}

void Algorithms::separateNodes() {
    for (unsigned index = 0; index < nodes.size(); ++index) {
        string currFuelType = nodes[index].fuelType;
        if (map.find(currFuelType) == map.end()) {
            std::vector<Node> v;
            v.push_back(nodes[index]);
            map.insert({currFuelType, v});
        } else {
            map.at(currFuelType).push_back(nodes[index]);
        }
    }
}

void Algorithms::printGraphs(std::string fuelType) {
    // std::vector<Node>& v = map.at(fuelType);
    for (Node node : map[fuelType]) {
        std::cout << node.streetAddress << std::endl;
    }
}
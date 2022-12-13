#include "Algorithms.h"
#include <chrono>
#include <iostream>

int main(int argc, char** argv) {

    // run (for example): ./driver data/champaign_urbana_data.csv  200.0 
    if (argc != 6) {
        std::cerr << "Usage: ./driver file_path fuel_type range starting_address 0/ending_address" << std::endl;
        return 1;
    }
    
    string file_path = argv[1];
    string fuel_type = argv[2];
    double range = stod(argv[3]);
    string starting_address = argv[4];
    string ending_address = argv[5];

    auto start = std::chrono::steady_clock::now();

    Algorithms a("../" + file_path, fuel_type, range, 1 /* use KDTree constructor */);

    if (ending_address != "0") {

        // Perform Dijkstra's
        const vector<int>& sequence = a.Dijkstra(a.nodefinder(starting_address).index, a.nodefinder(ending_address).index);
        
        if (sequence.empty()) {
            cout << "There is no path from " << starting_address <<  " to " << ending_address << "." << endl;
            return 0;
        } else {
            cout << "The path from " << starting_address <<  " to " << ending_address << " is:" << endl;
            for (int index : sequence) {
                std::cout << a.GetNode(index).ToString() << std::endl;
            }
        }

    } else {

        // Perform BFS
        std::cout << "These are the " << fuel_type << " stations you can visit: " << std::endl;
        const set<Node>& visited = a.BFS(a.nodefinder(starting_address));
        for (const Node& node : visited) {
            std::cout << node.ToString() << std::endl;
        }

        std::cout << "The total number of " << fuel_type <<" stations you can visit you can visit from " << starting_address << " is " << visited.size() << "." << std::endl;

    }
    
    auto end = std::chrono::steady_clock::now();
    double duration = (end - start).count() / 1000000000.0;
    std::cout << "Time taken: " << duration << std::endl;
}
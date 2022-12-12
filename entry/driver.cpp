#include "Algorithms.h"
#include <iostream>

int main(int argc, char** argv) {

    // run (for example): ./driver data/champaign_urbana_data.csv  200.0 
    if (argc != 4) {
        std::cerr << "Usage: ./driver file_path fuel_type range 0/1(BFS/Dijkstra's) starting_address" << std::endl;
        return 1;
    }
    
    string file_path = argv[1];
    string fuel_type = argv[2];
    double range = stod(argv[3]);
    bool dijkstra = argv[4];
    string starting_address = argv[5];

    Algorithms a("../" + file_path, fuel_type, range, 1 /* use KDTree constructor */);

    if (dijkstra) {
        
        // do something
    } else {
        a.BFS(nodefinder(starting_address));
        // do something
    }

    return 0;
}
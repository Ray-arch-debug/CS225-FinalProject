#include "Algorithms.h"
#include <iostream>

int main(int argc, char** argv) {

    // run (for example): ./driver data/champaign_urbana_data.csv ELEC 200.0
    if (argc != 4) {
        std::cerr << "Usage: ./driver file_path fuel_type range" << std::endl;
        return 1;
    }
    
    string file_path = argv[1];
    string fuel_type = argv[2];
    double range = stod(argv[3]);
    Algorithms a("../" + file_path, fuel_type, range, 1 /* or 0 */);

    return 0;
}
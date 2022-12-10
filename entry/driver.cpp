#include "Algorithms.h"

int main() {

    /*
    if (argc != 4) {
        std::cerr << "Usage: ./driver file_path fuel_type range" << std::endl;
        return 1;
    }
    */
    Algorithms a("../data/champaign_urbana_data.csv", "ELEC", 200.0);

    return 0;
}
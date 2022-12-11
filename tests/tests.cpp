#include "Algorithms.h"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <ctime>
#include <iostream>

void VerifyEqual(const Algorithms& first, const Algorithms& second) {
    bool equal_size = (first.GetGraph().size() == second.GetGraph().size() && 
                       first.GetVertices().size() == second.GetVertices().size() &&
                       first.GetGraph().size() == first.GetVertices().size());
    REQUIRE(equal_size);

    for (const Node& vertex : first.GetVertices()) {
        std::set<Node> first_neighbors = first.GetGraph().at(vertex);
        std::set<Node> second_neighbors = second.GetGraph().at(vertex);
        REQUIRE( first_neighbors == second_neighbors );
    }
};

TEST_CASE("Graph Constructor Simple", "[valgrind]") {
    Algorithms algorithms( "../data/champaign_urbana_data.csv", "ELEC", 200.0, 0 );
    REQUIRE( algorithms.GetGraph().size() == 23 );
    Node node { 26689, "ELEC", "1401 W Green St", "IL", 40.109651, -88.226594 };
    REQUIRE( algorithms.GetVertices().find(node) != algorithms.GetVertices().end() );
    const std::set<Node>& neighbors = algorithms.GetGraph().at(node);
    REQUIRE( neighbors.size() == 22 );
    REQUIRE( neighbors.find(node) == neighbors.end() );
}

TEST_CASE("Graph Constructor Edge Case", "[valgrind]") {
    Algorithms algorithms( "../data/champaign_urbana_data.csv", "ELEC",  1.75 * ROAD_CURVINESS_QUOTIENT, 0 );
    Node hyatt  { 17316, "ELEC", "217 N Neil St"       , "IL", 40.11799 , -88.243955 };
    Node ihotel { 17314, "ELEC", "1900 S 1st St"       , "IL", 40.093533, -88.237677 };
    Node hilton { 17315, "ELEC", "2013 S Neil St"      , "IL", 40.092448, -88.247278 };
    Node uhaul  { 756  , "LPG" , "306 E University Ave", "IL", 40.116631, -88.23472 };
    const std::set<Node>& neighbors = algorithms.GetGraph().at(hyatt);
    REQUIRE( neighbors.size() == 11 );
    REQUIRE( neighbors.find(ihotel) != neighbors.end() );
    REQUIRE( neighbors.find(hyatt)  == neighbors.end() );
    REQUIRE( neighbors.find(hilton) == neighbors.end() );
    REQUIRE( neighbors.find(uhaul)  == neighbors.end() );
}

TEST_CASE("Graph Constructor Simple KDTree", "[valgrind]") {
    Algorithms algorithms_naive ( "../data/champaign_urbana_data.csv", "ELEC", 2.022, 0 );
    Algorithms algorithms_kdtree( "../data/champaign_urbana_data.csv", "ELEC", 2.022, 1 );

    VerifyEqual(algorithms_naive, algorithms_kdtree);
}

// still need to write tests for kdtree graph constructor and all of algorithms
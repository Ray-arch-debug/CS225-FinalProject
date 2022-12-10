#include <catch2/catch_test_macros.hpp>
#include "Algorithms.h"

TEST_CASE("Graph Constructor Simple (to rename later...)", "[valgrind]") {
    Algorithms algorithms( "../data/champaign_urbana_data.csv", "ELEC", 200.0 );
    REQUIRE( algorithms.GetGraph().size() == 25 );
    Node node { 26689, "ELEC", "1401 W Green St", "IL", 40.109651, -88.226594 };
    REQUIRE( algorithms.GetVertices().find(node) != algorithms.GetVertices().end() );
    const std::set<Node>& neighbors = algorithms.GetGraph().at(node);
    REQUIRE( neighbors.size() == 24 );
    REQUIRE( neighbors.find(node) == neighbors.end() );
}

// I will need to write a more involved test later
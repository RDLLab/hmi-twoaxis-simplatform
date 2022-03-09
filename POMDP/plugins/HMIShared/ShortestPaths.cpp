#include "ShortestPaths.hpp"

namespace oppt {

namespace hmi {

ShortestPaths::ShortestPaths(Grid &grid) : grid_(&grid) { }

std::string ShortestPaths::getPath(Coordinate from, Coordinate to) {
    return paths_[from.toPosition(*grid_)][to.toPosition(*grid_)];
}

void ShortestPaths::computePaths() {
    
}

}

}
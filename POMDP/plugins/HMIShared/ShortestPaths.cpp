#include "ShortestPaths.hpp"

namespace oppt {

namespace hmi {

ShortestPaths::ShortestPaths() : paths_() { }

ShortestPaths::ShortestPaths(Grid &grid) {
    int size = grid.getWidth() * grid.getHeight();
    paths_ = std::vector<VectorString>(size, VectorString(size));
    computePaths(grid);
}

std::string ShortestPaths::getPath(int from, int to) const {
    return paths_[from][to];
}

int ShortestPaths::getLongestPath() const {
    return longestPath_;
}

void ShortestPaths::computePaths(Grid &grid) {
    longestPath_ = 0;
    for (size_t i = 0; i != paths_.size(); ++i) {
        Coordinate src(i, grid);
        if (grid.getGrid()[src.toPosition(grid)]) {
            for (size_t j = 0; j != paths_.size(); ++j) {
                Coordinate dest(j, grid);
                if (grid.getGrid()[dest.toPosition(grid)]) {
                    std::string path = getShortestPath(grid, src.getX(), src.getY(), dest.getX(), dest.getY()).second;
                    paths_[src.toPosition(grid)][dest.toPosition(grid)] = path;
                    longestPath_ = std::max(longestPath_, (int) path.size());
                }
            }
        }
    }
}

}

}
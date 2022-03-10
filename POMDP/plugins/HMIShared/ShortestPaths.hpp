#ifndef _SHORTEST_PATHS_HPP_
#define _SHORTEST_PATHS_HPP_

#include "HMIDataStructures.hpp"
#include "oppt/opptCore/core.hpp"

namespace oppt {

namespace hmi {

class ShortestPaths {

    public:

    ShortestPaths(Grid &grid);

    std::string getPath(int from, int to);

    private:

    std::vector<VectorString> paths_;

    private:

    void computePaths(Grid &grid);

};
}
}

#endif
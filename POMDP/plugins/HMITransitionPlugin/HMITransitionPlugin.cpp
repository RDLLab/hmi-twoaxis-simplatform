#include "oppt/plugin/Plugin.hpp"
#include "oppt/opptCore/Distribution.hpp"
#include "HMITransitionPluginOptions.hpp"
#include "plugins/HMIShared/HMIDataStructures.hpp"
#include "plugins/HMIShared/ShortestPaths.hpp"

#include <string>
#include <array>
#include <memory>
#include <stdio.h>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <map>
#include <unordered_map>
#include <random>
#include <utility>
#include <math.h>
#include <set>
#include <stdlib.h>

#include <iostream>


namespace oppt 
{
class HMITransitionPlugin: public TransitionPlugin 
{
public:

    HMITransitionPlugin():
        TransitionPlugin() {}

    virtual ~HMITransitionPlugin() = default;

    virtual bool load(const std::string &optionsFile) override {
        parseOptions_<HMITransitionPluginOptions>(optionsFile);
        std::string gridPath
          = static_cast<HMITransitionPluginOptions*>(options_.get())->gridPath;
        grid_ = hmi::instantiateGrid(gridPath);
        shortestPaths_ = hmi::ShortestPaths(grid_);
        std::string requestersPath
          = static_cast<HMITransitionPluginOptions*>(options_.get())->requestersPath;
        requesters_ = hmi::instantiateTypesAndIDs(requestersPath);
        std::string transitionMatricesPath
            = static_cast<HMITransitionPluginOptions*>(options_.get())->transitionMatrixPath;
        transitionMatrices_ = hmi::instantiateTransitionMatrices(transitionMatricesPath);
        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        PropagationResultSharedPtr propagationResult(new PropagationResult());
        VectorFloat actionVec = propagationRequest->action->as<VectorAction>()->asVector();
        VectorFloat stateVec(propagationRequest->currentState->as<VectorState>()->asVector());
        VectorFloat outState(stateVec.size(), -1);

        // Process actions and any requesters helped by those actions
        for (size_t i = 0; i != actionVec.size(); i += 2) {
            // The i-th robot's x-coordinate in s'
            outState[i] = actionVec[i];
            // The i-th robot's y-coordinate in s'
            outState[i+1] = actionVec[i+1];
            for (size_t j = actionVec.size(); j != stateVec.size(); j += 3) {
                bool sameX = stateVec[j] == actionVec[i];
                bool sameY = stateVec[j+1] == actionVec[i+1];
                // Helper robot is moving towards requester's location
                if (sameX && sameY) {
                    outState[j] = stateVec[j];
                    outState[j+1] = stateVec[j+1];
                    outState[j+2] = 0;
                }
            }
        }

        // Process all requesters who weren't helped by an action this time
        for (int j = actionVec.size(); j != outState.size(); j += 3) {
            if (outState[j] < 0) {
                FloatType x = stateVec[j];
                FloatType y = stateVec[j+1];
                FloatType c = stateVec[j+2];
                int idx = (j - actionVec.size()) / 3;
                std::string type = requesters_[idx].first;
                VectorFloat t = transition(x, y, c, type);
                for (size_t k = 0; k != 3; ++k) outState[j+k] = t[k];
            }
        }

        propagationResult->previousState = propagationRequest->currentState.get();
        propagationResult->action = propagationRequest->action;
        propagationResult->nextState = std::make_shared<oppt::VectorState>(outState);
        return propagationResult;
    }

private:

    std::vector<hmi::TypeAndId> requesters_;
    hmi::Grid grid_;
    std::unordered_map<std::string, hmi::TransitionMatrix> transitionMatrices_;
    hmi::ShortestPaths shortestPaths_;

    VectorFloat transition(FloatType x, FloatType y, FloatType c, std::string type) const {
        std::random_device rd;
        RandomEngine generator(rd());
        FloatType outX = x;
        FloatType outY = y;
        FloatType outC;
        if (c == 0) {
            VectorInt neighbours = getNeighbours(x, y);
            if (!neighbours.empty()) {
                int numN = neighbours.size() / 2;
                std::uniform_int_distribution<int> moveDist(0, numN - 1);
                int idx = moveDist(generator);
                outX = neighbours.at(2 * idx);
                outY = neighbours.at(2 * idx + 1);
            }
        }
        hmi::TransitionMatrix matrix = transitionMatrices_.at(type);
        std::uniform_real_distribution<float> transDist(0.0, 1.0);
        float changeVal = transDist(generator);
        int newC = -1;

        while (changeVal >= 0.0) {

            // Move through successive conditions and subtract the probability of transitioning
            // from the requester's current condition to the given condition from the given random
            // float, until the sign of the given random float is negative.
            ++newC;
            changeVal -= matrix.matrix_[c][newC];
        }
        outC = newC;
        return {outX, outY, outC};
    }

    VectorInt getNeighbours(FloatType x, FloatType y) const {
        VectorInt neighbours = VectorInt();
        for (int xIdx = -1; xIdx != 2; ++xIdx) {
            for (int yIdx = -1; yIdx != 2; ++yIdx) {
                bool validMove = false;
                if (abs(xIdx - yIdx) == 1) {
                    int neighX = (int) (x + xIdx);
                    int neighY = (int) (y + yIdx);
                    bool validX = neighX > -1 && neighX < grid_.getWidth();
                    bool validY = neighY > -1 && neighY < grid_.getHeight();
                    if (validX && validY) {
                        hmi::Coordinate coord = hmi::Coordinate(neighX, neighY);
                        bool validCell = grid_.getGrid()[coord.toPosition(grid_)];
                        if (validCell) {
                            neighbours.push_back(neighX);
                            neighbours.push_back(neighY);
                            validMove = true;
                        }
                    }
                }
            }
        }
        return neighbours;
    }
    
};

OPPT_REGISTER_TRANSITION_PLUGIN(HMITransitionPlugin)

}

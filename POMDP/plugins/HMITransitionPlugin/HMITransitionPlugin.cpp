#include "oppt/plugin/Plugin.hpp"
#include "oppt/opptCore/Distribution.hpp"
#include "HMITransitionPluginOptions.hpp"
#include "plugins/HMIShared/HMIObservation.hpp"
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
        // std::cout << "Running method load() in class HMITransitionPlugin...\n";
        parseOptions_<HMITransitionPluginOptions>(optionsFile);
        std::string gridPath
          = static_cast<HMITransitionPluginOptions*>(options_.get())->gridPath;
        grid_ = hmi::instantiateGrid(gridPath);
        shortestPaths_ = hmi::ShortestPaths(grid_);
        std::string randomAgentsPath
          = static_cast<HMITransitionPluginOptions*>(options_.get())->randomAgentsPath;
        randomAgents_ = hmi::instantiateTypesAndIDs(randomAgentsPath);
        std::string transitionMatricesPath
            = static_cast<HMITransitionPluginOptions*>(options_.get())->transitionMatrixPath;
        transitionMatrices_ = hmi::instantiateTransitionMatrices(transitionMatricesPath);
        // std::cout << "Completed method load() in class HMITransitionPlugin...\n";

        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        // std::cout << "Running HMITransitionPlugin.propagateState()..." << std::endl;
        PropagationResultSharedPtr propagationResult(new PropagationResult());
        // std::cout << "Setting up action vector" << std::endl;
        VectorFloat actionVec = propagationRequest->action->as<VectorAction>()->asVector();
        //// std::cout << "Action is (" << actionVec[0] << "," << actionVec[1] << ")" << std::endl;
        VectorFloat stateVec(propagationRequest->currentState->as<VectorState>()->asVector());
        // std::cout << "Instantiating outState..." << std::endl;
        VectorFloat outState(stateVec.size(), -1);

        // Process actions and any random agents helped by those actions
        // std::cout << "Start action loop" << std::endl;
        for (size_t i = 0; i != actionVec.size(); i += 2) {
            // The i-th robot's x-coordinate in s'
            // std::cout << "Copy x-coordinate" << std::endl;
            outState[i] = actionVec[i];
            // The i-th robot's y-coordinate in s'
            // std::cout << "Copy y-coordinate" << std::endl;
            outState[i+1] = actionVec[i+1];
            // std::cout << "Start agent loop" << std::endl;
            for (size_t j = actionVec.size(); j != stateVec.size(); j += 3) {
                // std::cout << "Check x-coordinate" << std::endl;
                bool sameX = stateVec[j] == actionVec[i];
                // std::cout << "Check y-coordinate" << std::endl;
                bool sameY = stateVec[j+1] == actionVec[i+1];
                // Helper robot is moving towards random agent's location
                // std::cout << "Check matching coordinates" << std::endl;
                if (sameX && sameY) {
                    // std::cout << "Same x-coordinate" << std::endl;
                    outState[j] = stateVec[j];
                    // std::cout << "Same y-coordinate" << std::endl;
                    outState[j+1] = stateVec[j+1];
                    // std::cout << "Agent is happy" << std::endl;
                    outState[j+2] = 0;
                }
            }
        }

        // Process all random agents who weren't helped by an action this time
        for (int j = actionVec.size(); j != outState.size(); j += 3) {
            if (outState[j] < 0) {
                FloatType x = stateVec[j];
                FloatType y = stateVec[j+1];
                FloatType c = stateVec[j+2];
                int idx = (j - actionVec.size()) / 3;
                // std::cout << "j is " << j << std::endl;
                // std::cout << "actionVec.size() is " << actionVec.size() << std::endl;
                // std::cout << "Index is " << idx << std::endl;
                std::string type = randomAgents_[idx].first;
                VectorFloat t = transition(x, y, c, type);
                for (size_t k = 0; k != 3; ++k) outState[j+k] = t[k];
            }
        }

        std::ostringstream oss;
        std::copy(stateVec.begin(), stateVec.end()-1, std::ostream_iterator<FloatType>(oss, ","));

        // Now add the last element with no delimiter
        oss << stateVec.back() << std::endl;
        std::copy(actionVec.begin(), actionVec.end()-1, std::ostream_iterator<FloatType>(oss, ","));

        // Now add the last element with no delimiter
        oss << actionVec.back() << std::endl;
        std::copy(outState.begin(), outState.end()-1, std::ostream_iterator<FloatType>(oss, ","));

        // Now add the last element with no delimiter
        oss << outState.back() << std::endl;

        std::cout << oss.str() << std::endl;

        propagationResult->previousState = propagationRequest->currentState.get();
        propagationResult->action = propagationRequest->action;
        propagationResult->nextState = std::make_shared<oppt::VectorState>(outState);
        // std::cout << "Completed HMITransitionPlugin.propagateState()..." << std::endl;
        return propagationResult;
    }

private:

    std::vector<hmi::TypeAndId> randomAgents_;
    hmi::Grid grid_;
    std::unordered_map<std::string, hmi::TransitionMatrix> transitionMatrices_;
    hmi::ShortestPaths shortestPaths_;

    VectorFloat transition(FloatType x, FloatType y, FloatType c, std::string type) const {
        // std::cout << "Starting transition()..." << std::endl;
        std::random_device rd;
        RandomEngine generator(rd());
        FloatType outX = x;
        FloatType outY = y;
        FloatType outC;
        if (c == 0) {
            VectorInt neighbours = getNeighbours(x, y);
            if (!neighbours.empty()) {
                // std::cout << "Making a move..." << std::endl;
                int numN = neighbours.size() / 2;
                std::uniform_int_distribution<int> moveDist(0, numN - 1);
                int idx = moveDist(generator);
                outX = neighbours.at(2 * idx);
                outY = neighbours.at(2 * idx + 1);
            }
        }
        // std::cout << "Instantiating a transition matrix..." << std::endl;
        hmi::TransitionMatrix matrix = transitionMatrices_.at(type);
        std::uniform_real_distribution<float> transDist(0.0, 1.0);
        float changeVal = transDist(generator);
        int newC = -1;

        while (changeVal >= 0.0) {

            // Move through successive conditions and subtract the probability of transitioning
            // from the agent's current condition to the given condition from the given random
            // float, until the sign of the given random float is not positive.
            ++newC;
            // std::cout << "New condition is currently " << newC << std::endl;
            changeVal -= matrix.matrix_[c][newC];
        }
        outC = newC;
        // std::cout << "Completed transition()..." << std::endl;
        return {outX, outY, outC};
    }

    VectorInt getNeighbours(FloatType x, FloatType y) const {
        // std::cout << "Running getNeighbours()..." << std::endl;
        VectorInt neighbours = VectorInt();
        for (int xIdx = -1; xIdx != 2; ++xIdx) {
            for (int yIdx = -1; yIdx != 2; ++yIdx) {
                bool validMove = false;
                std::cout << "For x == " << std::to_string(x) << ", y == " << std::to_string(y) << ", " << std::endl;
                std::cout << "Testing xIdx == " << std::to_string(xIdx) << ", yIdx == " << std::to_string(yIdx) << std::endl;
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
                if (validMove) std::cout << "Valid move!" << std::endl;
                else           std::cout << "Invalid move..." << std::endl;
            }
        }
        // std::cout << "Completed getNeighbours()..." << std::endl;
        return neighbours;
    }
    
};

OPPT_REGISTER_TRANSITION_PLUGIN(HMITransitionPlugin)

}

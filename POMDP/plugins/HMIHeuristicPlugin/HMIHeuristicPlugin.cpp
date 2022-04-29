#ifndef _HMI_HEURISTIC_PLUGIN_HPP_
#define _HMI_HEURISTIC_PLUGIN_HPP_
#include "oppt/plugin/Plugin.hpp"
#include "HMIHeuristicOptions.hpp"
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
#include <set>
#include <random>

namespace oppt
{
class HMIHeuristicPlugin: public HeuristicPlugin
{
public:

    HMIHeuristicPlugin(): HeuristicPlugin() { }

    virtual ~HMIHeuristicPlugin() {}

    virtual bool load(const std::string& optionsFile) override {
        // Set up the dependent agents as per robot environment.
        // std::cout << "Running method load() in class HMIHeuristicPlugin...\n";
        parseOptions_<HMIHeuristicOptions>(optionsFile);
        std::string gridPath
            = static_cast<HMIHeuristicOptions*>(options_.get())->gridPath;
        grid_ = hmi::instantiateGrid(gridPath);
        paths_ = hmi::ShortestPaths(grid_);
        helpReward = paths_.getLongestPath() + 1;
        std::string randomAgentsPath
            = static_cast<HMIHeuristicOptions*>(options_.get())->randomAgentsPath;
        randomAgents_ = hmi::instantiateTypesAndIDs(randomAgentsPath);
        std::string transitionMatricesPath
            = static_cast<HMIHeuristicOptions*>(options_.get())->transitionMatrixPath;
        transitionMatrices_ = hmi::instantiateTransitionMatrices(transitionMatricesPath);
        // std::cout << "Completed method load() in class HMIHeuristicPlugin...\n";
        return true;
    }

    virtual FloatType getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {
        // std::cout << "Running method getHeuristicValue() in class HMIHeuristicPlugin...\n";
        VectorFloat stateVec =
            heuristicInfo->currentState->as<VectorState>()->asVector();
        VectorFloat actionVec =
            heuristicInfo->action->as<VectorAction>()->asVector();
        FloatType currentDiscount = 1.0;
        FloatType totalDiscountedReward = 0.0;

        int numRobots = actionVec.size() / 2;
        int numRandAgents = (stateVec.size() - actionVec.size()) / 3;
        std::vector<bool> agentsVisited = std::vector<bool>(numRandAgents, false);
        bool allAgentsHelped = false;
        while (!allAgentsHelped) {
            currentDiscount *= heuristicInfo->discountFactor;
            VectorFloat gRewards = VectorFloat(numRobots, 0);
            VectorInt gIndices = VectorInt(numRobots, -1);
            VectorFloat bestRewards = VectorFloat();
            std::vector<hmi::Coordinate> bestRobotAgentCombinations = std::vector<hmi::Coordinate>();
            for (int i = 0; i != actionVec.size(); i += 2) {
                hmi::Coordinate start = hmi::Coordinate(stateVec[i], stateVec[i + 1]);
                for (int j = actionVec.size(); j != stateVec.size(); j += 3) {
                    hmi::Coordinate action = hmi::Coordinate(stateVec[j], stateVec[j + 1]);
                    FloatType possReward = stateVec[j + 2] > 0 ? helpReward : 0.0;
                    possReward -= paths_.getPath(start.toPosition(grid_), action.toPosition(grid_)).size();
                    bool inserted = false;
                    VectorFloat::iterator it;
                    std::vector<hmi::Coordinate>::iterator cit = bestRobotAgentCombinations.begin();
                    for (it = bestRewards.begin(); it != bestRewards.end(); ++it) {
                        if (possReward > *it) {
                            bestRewards.insert(it, possReward);
                            bestRobotAgentCombinations.insert(cit, hmi::Coordinate(i / 2, (j - actionVec.size()) / 3));
                            inserted = true;
                            break;
                        }
                        ++cit;
                    }
                    if (!inserted) {
                        bestRewards.push_back(possReward);
                        bestRobotAgentCombinations.push_back(hmi::Coordinate(i / 2, (j - actionVec.size()) / 3));
                    }
                }
            }
            for (int i = 0; i != bestRewards.size(); ++i) {
                int robotIdx = bestRobotAgentCombinations[i].getX();
                int agentIdx = bestRobotAgentCombinations[i].getY();
                bool robotFree = gIndices[robotIdx] < 0;
                bool agentFree = std::find(gIndices.begin(), gIndices.end(), agentIdx) == gIndices.end();
                if (robotFree && agentFree && !agentsVisited[agentIdx]) {
                    gIndices[robotIdx] = agentIdx;
                    gRewards[robotIdx] = bestRewards[i];
                    agentsVisited[agentIdx] = true;
                    stateVec[robotIdx * 2] = stateVec[agentIdx * 3 + actionVec.size()];
                    stateVec[robotIdx * 2 + 1] = stateVec[agentIdx * 3 + actionVec.size() + 1];
                    stateVec[agentIdx * 3 + actionVec.size() + 2] = 0;
                }
                if (std::find(gIndices.begin(), gIndices.end(), -1) == gIndices.end()) break;
            }

            for (FloatType reward : gRewards) totalDiscountedReward += currentDiscount * reward;

            for (int j = actionVec.size(); j != stateVec.size(); j += 3) {
                int idx = (j - actionVec.size()) / 3;
                if (std::find(gIndices.begin(), gIndices.end(), idx) == gIndices.end()) {
                    FloatType x = stateVec[j];
                    FloatType y = stateVec[j+1];
                    FloatType c = stateVec[j+2];
                    std::string type = randomAgents_[idx].first;
                    VectorFloat t = transition(x, y, c, type);
                    for (size_t k = 0; k != 3; ++k) stateVec[j+k] = t[k];
                }
            }

            allAgentsHelped = std::find(agentsVisited.begin(), agentsVisited.end(), false) == agentsVisited.end();
        }

        // std::cout << "Completed method getHeuristicValue() in class HMIHeuristicPlugin...\n";
        return totalDiscountedReward;
    }

private:
    std::vector<hmi::TypeAndId> randomAgents_;
    hmi::Grid grid_;
    std::unordered_map<std::string, hmi::TransitionMatrix> transitionMatrices_;
    hmi::ShortestPaths paths_;
    FloatType helpReward;

    VectorFloat transition(FloatType x, FloatType y, FloatType c, std::string type) const {
        RandomEngine generator;
        FloatType outX = x;
        FloatType outY = y;
        FloatType outC;
        if (c == 0) {
            VectorInt neighbours = getNeighbours(x, y);
            if (!neighbours.empty()) {
                int numN = neighbours.size() / 2;
                std::uniform_int_distribution<int> moveDist(0, numN);
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
            // from the agent's current condition to the given condition from the given random
            // float, until the sign of the given random float is not positive.
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
                        }
                    }
                }
            }
        }
        return neighbours;
    }

};

OPPT_REGISTER_HEURISTIC_PLUGIN(HMIHeuristicPlugin)

}

#endif

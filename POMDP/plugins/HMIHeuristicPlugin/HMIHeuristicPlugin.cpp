#ifndef _HMI_HEURISTIC_PLUGIN_HPP_
#define _HMI_HEURISTIC_PLUGIN_HPP_
#include "oppt/plugin/Plugin.hpp"
#include "HMIHeuristicOptions.hpp"
#include "plugins/HMIShared/HMIObservation.hpp"

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
        VectorFloat stateVec
            = heuristicInfo->currentState->as<VectorState>()->asVector();
        hmi::HMIState hmiState(stateVec, randomAgents_, transitionMatrices_, grid_);
        hmi::HMIObservation hmiObservation(hmiState);
        FloatType currentDiscount = std::pow(heuristicInfo->discountFactor, heuristicInfo->currentStep);        
        FloatType val = 0;

        // Collect all the dependent agents that need help
        std::set<hmi::HMIRandomAgent*> unhappyRandomAgents;
        for (hmi::HMIRandomAgent randomAgent : hmiState.getRandomAgents()) {
            if (randomAgent.getCondition() > 0) {
                unhappyRandomAgents.insert(&randomAgent);
            }
        }

        // Visit them from closest first
        hmi::Coordinate robotCoords = hmiState.getRobotCoordinates();
        while (!unhappyRandomAgents.empty()) {
            std::set<hmi::HMIRandomAgent*>::iterator unhappyRandomAgentsIterator = unhappyRandomAgents.begin();
            int robX = hmiState.getRobotX();
            int robY = hmiState.getRobotY();
            int shortestDistance = -1;
            std::string shortestPath = "";
            hmi::HMIRandomAgent *closestRandomAgent = *unhappyRandomAgentsIterator;
            for (; unhappyRandomAgentsIterator != unhappyRandomAgents.end(); ++unhappyRandomAgentsIterator) {
                int randAgX = (* unhappyRandomAgentsIterator)->getX();
                int randAgY = (* unhappyRandomAgentsIterator)->getY();
                std::pair<int, std::string> path = hmi::getShortestPath(grid_, robX, robY, randAgX, randAgY);
                if (path.first < shortestDistance || shortestDistance < 0) {
                    closestRandomAgent = *unhappyRandomAgentsIterator;
                    shortestDistance = path.first;
                    shortestPath = path.second;
                }
            }
            currentDiscount *= std::pow(heuristicInfo->discountFactor, shortestDistance);
            for (int i = 0; i < shortestDistance; ++i) {
                if (shortestPath.at(i) == 'N')      hmiState.setRobotY(hmiState.getRobotY() - 1);
                else if (shortestPath.at(i) == 'S') hmiState.setRobotY(hmiState.getRobotY() + 1);
                else if (shortestPath.at(i) == 'E') hmiState.setRobotX(hmiState.getRobotX() + 1);
                else                                hmiState.setRobotX(hmiState.getRobotX() - 1);
                for (hmi::HMIRandomAgent randAg : hmiState.getRandomAgents()) {
                    if (closestRandomAgent->getType() != randAg.getType() && closestRandomAgent->getID() != randAg.getID()) {
                        randAg.sampleMovement(grid_);
                    }
                }
                hmiObservation.makeObservations();
                VectorInt obsVector = hmiObservation.toVector();
                for (size_t j = 0; j != hmiState.getRandomAgents().size(); ++j) {
                    if (obsVector[j] > 0) {
                        unhappyRandomAgents.insert(&hmiState.getRandomAgents()[j]);
                    }
                }
            }
            unhappyRandomAgents.erase(closestRandomAgent);

            int numHappy = 0;
            bool allHappy = true;

            for (hmi::HMIRandomAgent randAg : hmiState.getRandomAgents()) {
                if (randAg.getCondition() == 0) ++numHappy;
                else                            allHappy = false;
            }

            if (allHappy) {
                val += hmi::MAX_REWARD;
            }
            else {
                for (hmi::HMIRandomAgent randAg : hmiState.getRandomAgents()) {
                    if (randAg.getCondition() != 0) {
                        val += numHappy * hmi::BASE_REWARD / std::max(2, hmi::getShortestPath(grid_, hmiState.getRobotX(), hmiState.getRobotY(), randAg.getX(), randAg.getY()).first);
                    }
                    else {
                        val += numHappy * hmi::BASE_REWARD;
                    }
                }
            }
        }
        // std::cout << "Completed method getHeuristicValue() in class HMIHeuristicPlugin...\n";
        return val;
    }

private:
    std::vector<hmi::TypeAndId> randomAgents_;
    hmi::Grid grid_;
    std::unordered_map<std::string, hmi::TransitionMatrix> transitionMatrices_;

};

OPPT_REGISTER_HEURISTIC_PLUGIN(HMIHeuristicPlugin)

}

#endif

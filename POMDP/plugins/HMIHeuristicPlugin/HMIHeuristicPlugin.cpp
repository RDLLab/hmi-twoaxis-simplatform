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
        FloatType currentDiscount = 1.0;     
        FloatType val = 0.0;

        // Collect all the dependent agents that need help
        std::set<hmi::HMIRandomAgent*> unhappyRandomAgents;
        for (hmi::HMIRandomAgent randomAgent : hmiState.getRandomAgents()) {
            if (randomAgent.getCondition() > 0) {
                unhappyRandomAgents.insert(&randomAgent);
            }
            else {
                val += hmi::BASE_REWARD;
            }
        }

        // Visit them from closest first
        while (!unhappyRandomAgents.empty()) {
            std::set<hmi::HMIRandomAgent*>::iterator unhappyRandomAgentsIterator = unhappyRandomAgents.begin();
            int shortestDistance = -1;
            hmi::HMIRandomAgent* closestRandomAgent = *unhappyRandomAgentsIterator;
            hmi::HMIRobot* closestRobot = &hmiState.getRobots()[0];
            for (; unhappyRandomAgentsIterator != unhappyRandomAgents.end(); ++unhappyRandomAgentsIterator) {
                int randAgX = (* unhappyRandomAgentsIterator)->getX();
                int randAgY = (* unhappyRandomAgentsIterator)->getY();
                for (hmi::HMIRobot robot : hmiState.getRobots()) {
                    int robX = robot.getCoordinates().getX();
                    int robY = robot.getCoordinates().getY();
                    std::pair<int, std::string> path = hmi::getShortestPath(grid_, robX, robY, randAgX, randAgY);
                    if (path.first < shortestDistance || shortestDistance < 0) {
                        closestRandomAgent = *unhappyRandomAgentsIterator;
                        closestRobot = &robot;
                        shortestDistance = path.first;
                    }
                }
            }
            currentDiscount *= std::pow(heuristicInfo->discountFactor, shortestDistance);
            val += hmi::BASE_REWARD * currentDiscount;
            closestRobot->setCoordinates(closestRandomAgent->getCoords());
            unhappyRandomAgents.erase(closestRandomAgent);
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

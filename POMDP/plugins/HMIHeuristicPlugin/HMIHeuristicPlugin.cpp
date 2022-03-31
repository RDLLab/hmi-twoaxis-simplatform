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
        FloatType discount = 1.0;     
        FloatType val = 0.0;
        size_t numRobots = hmiState.getRobots().size();
        size_t numRandoms = hmiState.getRandomAgents().size();

        // Collect all the dependent agents that need help
        std::set<hmi::HMIRandomAgent*> needHelp;
        for (hmi::HMIRandomAgent randAgent : hmiState.getRandomAgents()) {
            if (randAgent.getCondition() > 0) {
                needHelp.insert(&randAgent);
            }
        }

        std::vector<hmi::Coordinate> actionVec 
            = std::vector<hmi::Coordinate>(2 * numRobots);

        // Visit them from closest first
        while (!needHelp.empty()) {
            // The path from each robot to each random agent
            // std::vector<VectorString> pathsToRandomAgents;
            // // Instantiate it for each robot
            // pathsToRandomAgents = std::vector<VectorString>(numRobots);

            // // Iterate through and populate path vector
            // for (size_t i = 0; i != numRobots; ++i) {
            //     // Instantiate the vector for robot[i] -> every random agent
            //     pathsToRandomAgents[i] = VectorString(numRandomAgents);
            //     // Get the relevant robot
            //     hmi::HMIRobot robot = hmiState.getRobots()[i];
            //     // Find its coordinates
            //     hmi::Coordinate robotCoord = robot.getCoordinates();
            //     // Convert them to the appropriate index
            //     int robotPos = robotCoord.toPosition(grid_);
            //     // Populate vector for every random agent
            //     for (size_t j = 0; j != numRandomAgents; ++j) {
            //         // Get the relevant random agent
            //         hmi::HMIRandomAgent randag = hmiState.getRandomAgents()[j];
            //         // Get its coordinates
            //         hmi::Coordinate randCoord = randag.getCoords();
            //         // Convert them to the appropriate index
            //         int randPos = randCoord.toPosition(grid_);
            //         // Find the path between the robot and the random agent
            //         std::string path = shortestPaths_.getPath(robotPos, randPos);
            //         // Store it in our vector
            //         pathsToRandomAgents[i][j] = path;
            //     }
            // }

            std::set<std::string> targetAgs;

            int longest = -1;

            for (size_t i = 0; i != numRobots; ++i) {
                hmi::HMIRobot robot = hmiState.getRobots()[i];
                hmi::Coordinate robCoords = robot.getCoordinates();
                int robotPos = robCoords.toPosition(grid_);
                std::set<hmi::HMIRandomAgent*>::iterator needHelpIt
                    = needHelp.begin();
                int shortest = paths_.getLongestPath();
                hmi::HMIRandomAgent* closest = *needHelpIt;
                for (; needHelpIt != needHelp.end(); ++needHelpIt) {
                    hmi::HMIRandomAgent* randAgent = *needHelpIt;
                    hmi::Coordinate randCoords = (* needHelpIt)->getCoords();
                    int randPos = randCoords.toPosition(grid_);
                    std::string path = paths_.getPath(robotPos, randPos);
                    if (path.size() < shortest) {
                        shortest = path.size();
                        closest = *needHelpIt;
                    }
                }
                targetAgs.insert(closest->getIdentifier());
                longest = std::max(longest, (int) shortest);
                actionVec[i] = closest->getCoords();
            }

            hmiState.sampleMovement(longest, targetAgs);

            discount *= std::pow(heuristicInfo->discountFactor, longest);
            val += discount * ((int) targetAgs.size() * hmi::BASE_REWARD);

            for (size_t i = 0; i != numRobots; ++i) {
                hmi::HMIRobot robot = hmiState.getRobots()[i];
                hmi::Coordinate robCoords = robot.getCoordinates();
                int robotPos = robCoords.toPosition(grid_);
                int actionPos = actionVec[i].toPosition(grid_);
                val -= (int) (targetAgs.size() * paths_.getPath(robotPos, actionPos).size());
                robot.setCoordinates(actionVec[i]);
            }

            for (hmi::HMIRandomAgent randAgent : hmiState.getRandomAgents()) {
                std::string id = randAgent.getIdentifier();
                if (targetAgs.find(id) != targetAgs.end()) {
                    randAgent.setCondition(0);
                }
            }
            needHelp.clear();
            for (hmi::HMIRandomAgent randAgent : hmiState.getRandomAgents()) {
                if (randAgent.getCondition() > 0) {
                    needHelp.insert(&randAgent);
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
    hmi::ShortestPaths paths_;

};

OPPT_REGISTER_HEURISTIC_PLUGIN(HMIHeuristicPlugin)

}

#endif

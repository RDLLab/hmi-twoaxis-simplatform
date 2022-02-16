#include "oppt/plugin/Plugin.hpp"
#include "oppt/opptCore/Distribution.hpp"
#include "HMITransitionPluginOptions.hpp"
#include "plugins/HMIShared/HMIState.hpp"

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
        // std::cout << "Running method propagateState() in class HMITransitionPlugin...\n";
        PropagationResultSharedPtr propagationResult(new PropagationResult());
        VectorFloat actionVec = propagationRequest->action->as<VectorAction>()->asVector();
        VectorFloat resultingState(propagationRequest->currentState->as<VectorState>()->asVector());
        hmi::HMIState currentState(resultingState, randomAgents_, transitionMatrices_, grid_);

        std::set<hmi::HMIRandomAgent*> targetAgents;
        for (size_t i = 0; i != actionVec.size(); i += 2) {
            int actionX = actionVec[i];
            int actionY = actionVec[i + 1];
            for (hmi::HMIRandomAgent randomAgent : currentState.getRandomAgents()) {
                if (randomAgent.getCoords().getX() == actionX && randomAgent.getCoords().getY() == actionY) {
                    targetAgents.insert(&randomAgent);
                    randomAgent.setCondition(0);
                }
            }
        }

        std::vector<std::pair<int, std::string>> shortestPaths(currentState.getRobots().size());
        int maxShortestPath = -1;
        for (size_t i = 0; i != currentState.getRobots().size(); ++i) {
            hmi::HMIRobot robot = currentState.getRobots()[i];
            int robotX = robot.getCoordinates().getX();
            int robotY = robot.getCoordinates().getY();
            int actionX = (int) actionVec[2*i];
            int actionY = (int) actionVec[2*i + 1];
            shortestPaths[i] = hmi::getShortestPath(grid_, robotX, robotY, actionX, actionY);
            maxShortestPath = std::max(shortestPaths[i].first, maxShortestPath);
        }

        for (size_t i = 0; i != maxShortestPath; ++i) {
            for (size_t j = 0; j != currentState.getRobots().size(); ++j) {
                if (i < shortestPaths[j].size()) {
                    hmi::HMIRobot robot = currentState.getRobots()[j];
                    hmi::Coordinate robotCoords = robot.getCoordinates();
                    std::string path = shortestPaths[j].second;
                    if (path.at(i) == 'N')      robot.setCoordinates(hmi::Coordinate(robot.getCoordinates().getX(), robot.getCoordinates.getY() - 1));
                    else if (path.at(i) == 'S') robot.setCoordinates(hmi::Coordinate(robot.getCoordinates().getX(), robot.getCoordinates.getY() + 1));
                    else if (path.at(i) == 'E') robot.setCoordinates(hmi::Coordinate(robot.getCoordinates().getX() + 1, robot.getCoordinates.getY()));
                    else                        robot.setCoordinates(hmi::Coordinate(robot.getCoordinates().getX() - 1, robot.getCoordinates.getY()));
                    for (hmi::HMIRandomAgent randomAgent : currentState.getRandomAgents()) {
                        if (randomAgent.getCoords() == robot.getCoordinates()) {
                            randomAgent.setCondition(0);
                        }
                    }   
                }
            }
            currentState.sampleMovement(1, targetAgents);
        }

        propagationResult->previousState = propagationRequest->currentState.get();
        VectorInt outState = currentState.toVector();
        VectorFloat floatOutState(outState.begin(), outState.end());
        propagationResult->nextState = std::make_shared<oppt::VectorState>(floatOutState);
        // std::cout << "Completed method propagateState() in class HMITransitionPlugin...\n";
        return propagationResult;
    }

private:

    std::vector<hmi::TypeAndId> randomAgents_;
    hmi::Grid grid_;
    std::unordered_map<std::string, hmi::TransitionMatrix> transitionMatrices_;
    
};

OPPT_REGISTER_TRANSITION_PLUGIN(HMITransitionPlugin)

}

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

        int actionX = round(actionVec[0]);
        int actionY = round(actionVec[1]);

        hmi::HMIRandomAgent* targetAgent = NULL;
        for (hmi::HMIRandomAgent randAg : currentState.getRandomAgents()) {
            if (randAg.getX() == actionX && randAg.getY() == actionY) {
                targetAgent = &randAg;
                randAg.setCondition(0);
                break;
            }
        }

        int robotX = currentState.getRobotX();
        int robotY = currentState.getRobotY();

        std::pair<int, std::string> shortestPath = hmi::getShortestPath(grid_, robotX, robotY, actionX, actionY);
        int shortestPathLength = shortestPath.first;
        std::string path = shortestPath.second;
        for (int i = 0; i < shortestPathLength; ++i) {
            hmi::Coordinate coords = currentState.getRobotCoordinates();
            if (path.at(i) == 'N')      currentState.setRobotY(coords.getY() - 1);
            else if (path.at(i) == 'S') currentState.setRobotY(coords.getY() + 1);
            else if (path.at(i) == 'E') currentState.setRobotX(coords.getX() + 1);
            else                        currentState.setRobotX(coords.getX() - 1);
            for (hmi::HMIRandomAgent randomAgent : currentState.getRandomAgents()) {
                if (randomAgent.getCoords() == currentState.getRobotCoordinates()) {
                    randomAgent.setCondition(0);
                }
            }
            currentState.sampleMovement(1, targetAgent);
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

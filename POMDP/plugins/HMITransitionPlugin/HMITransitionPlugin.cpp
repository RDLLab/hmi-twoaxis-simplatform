#include "oppt/plugin/Plugin.hpp"
#include "oppt/opptCore/Distribution.hpp"
#include "HMITransitionPluginOptions.hpp"
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
        // // std::cout << "Running method load() in class HMITransitionPlugin...\n";
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
        // // std::cout << "Completed method load() in class HMITransitionPlugin...\n";

        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        // // std::cout << "Running method propagateState() in class HMITransitionPlugin...\n";
        PropagationResultSharedPtr propagationResult(new PropagationResult());
        VectorFloat actionVec = propagationRequest->action->as<VectorAction>()->asVector();
        VectorFloat resultingState(propagationRequest->currentState->as<VectorState>()->asVector());
        hmi::HMIState currentState(resultingState, randomAgents_, transitionMatrices_, grid_);
        hmi::HMIObservation observation(currentState);

        std::set<std::string> targetAgents = observation.getUnderlyingState().getTargetAgents(actionVec);

        std::vector<std::string> shortestPaths(currentState.getRobots().size());
        int maxShortestPath = 0;
        for (size_t i = 0; i != currentState.getRobots().size(); ++i) {
            hmi::HMIRobot robot = currentState.getRobots()[i];
            int robotX = robot.getCoordinates().getX();
            int robotY = robot.getCoordinates().getY();
            int actionX = (int) actionVec[2*i];
            int actionY = (int) actionVec[2*i + 1];
            std::pair<int, std::string> path = hmi::getShortestPath(grid_, robotX, robotY, actionX, actionY);
            // std::cout << "From (" << robotX << "," << robotY << ") to (" << actionX << "," << actionY << "):" << std::endl;
            // std::cout << "Shortest distance is " << path.first << std::endl;
            shortestPaths[i] = path.second;
            maxShortestPath = std::max(path.first, maxShortestPath);
        }

        observation.sampleMovement(maxShortestPath, shortestPaths, targetAgents);

        VectorInt outState = observation.toStateVector();
        propagationResult->previousState = propagationRequest->currentState.get();

        VectorFloat floatOutState(outState.begin(), outState.end());
        propagationResult->nextState = std::make_shared<oppt::VectorState>(floatOutState);
        // // std::cout << "Completed method propagateState() in class HMITransitionPlugin...\n";
        return propagationResult;
    }

private:

    std::vector<hmi::TypeAndId> randomAgents_;
    hmi::Grid grid_;
    std::unordered_map<std::string, hmi::TransitionMatrix> transitionMatrices_;
    
};

OPPT_REGISTER_TRANSITION_PLUGIN(HMITransitionPlugin)

}

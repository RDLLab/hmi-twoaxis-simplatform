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
        VectorFloat actionVec = propagationRequest->action->as<VectorAction>()->asVector();
        //std::cout << "Action is (" << actionVec[0] << "," << actionVec[1] << ")" << std::endl;
        VectorFloat resultingState(propagationRequest->currentState->as<VectorState>()->asVector());
        hmi::HMIState currentState(resultingState, randomAgents_, transitionMatrices_, grid_);

        std::set<std::string> targetAgents = currentState.getTargetAgents(actionVec);

        std::vector<std::string> shortestPaths(currentState.getRobots().size());
        int maxShortestPath = 0;
        for (size_t i = 0; i != currentState.getRobots().size(); ++i) {
            hmi::HMIRobot robot = currentState.getRobots()[i];
            hmi::Coordinate robotCoords = robot.getCoordinates();
            hmi::Coordinate actionCoords((int) actionVec[2*i], (int) actionVec[2*i + 1]);
            std::string path = shortestPaths_.getPath(robotCoords.toPosition(grid_), actionCoords.toPosition(grid_));
            // // std::cout << "From (" << robotX << "," << robotY << ") to (" << actionX << "," << actionY << "):" << std::endl;
            // // std::cout << "Shortest distance is " << path.first << std::endl;
            shortestPaths[i] = path;
            maxShortestPath = std::max((int) path.size(), maxShortestPath);
        }
        for (size_t i = 0; i != maxShortestPath; ++i) {
            currentState.sampleMovement(1, targetAgents);
            for (size_t j = 0; j != currentState.getRobots().size(); ++j) {
                if (!shortestPaths[j].empty()) {
                    currentState.getRobots()[j].makeMove(shortestPaths[j].at(0));
                    shortestPaths[j] = shortestPaths[j].substr(1);
                }
            }
        }
        VectorInt outState = currentState.toVector();
        propagationResult->previousState = propagationRequest->currentState.get();

        VectorFloat floatOutState(outState.begin(), outState.end());
        propagationResult->nextState = std::make_shared<oppt::VectorState>(floatOutState);
        // std::cout << "Completed HMITransitionPlugin.propagateState()..." << std::endl;
        return propagationResult;
    }

private:

    std::vector<hmi::TypeAndId> randomAgents_;
    hmi::Grid grid_;
    std::unordered_map<std::string, hmi::TransitionMatrix> transitionMatrices_;
    hmi::ShortestPaths shortestPaths_;
    
};

OPPT_REGISTER_TRANSITION_PLUGIN(HMITransitionPlugin)

}

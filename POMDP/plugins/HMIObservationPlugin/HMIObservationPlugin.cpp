#include "oppt/plugin/Plugin.hpp"
#include "oppt/opptCore/Distribution.hpp"
#include "HMIObservationPluginOptions.hpp"
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
#include <algorithm>
#include <utility>

namespace oppt
{
class HMIObservationPlugin: public ObservationPlugin
{
public:

    HMIObservationPlugin(): ObservationPlugin() {}

    virtual ~HMIObservationPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {

        // std::cout << "Running method load() in class HMIObservationPlugin...\n";
        
        // Parse the options for this plugin.
        parseOptions_<HMIObservationPluginOptions>(optionsFile);

        // Extract data for the current grid being used and enrich it into a
        // Grid data structure.
        std::string gridPath
            = static_cast<HMIObservationPluginOptions*>(options_.get())->gridPath;
        grid_ = hmi::instantiateGrid(gridPath);

        // Extract random agent data for the current problem and enrich it into
        // a richer data structure.
        std::string randomAgentsPath
            = static_cast<HMIObservationPluginOptions*>(options_.get())->randomAgentsPath;
        randomAgents_ = hmi::instantiateTypesAndIDs(randomAgentsPath);

        // Extract transition matrix data for the current problem and enrich it into a
        // data structure that is easier to use.
        std::string transitionMatricesPath
            = static_cast<HMIObservationPluginOptions*>(options_.get())->transitionMatrixPath;
        transitionMatrices_ = hmi::instantiateTransitionMatrices(transitionMatricesPath);
        
        // std::cout << "Completed method load() in class HMIObservationPlugin...\n";

        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        
        // std::cout << "Running method getObservation() in class HMIObservationPlugin...\n";
        // Create the pointer that will store the result of the observation to be made.
        ObservationResultSharedPtr observationResult = std::make_shared<ObservationResult>();

        // Convert the current state of the problem into a richer data format.
        VectorFloat stateVec = observationRequest->currentState->as<VectorState>()->asVector();
        hmi::HMIState hmiState(stateVec, randomAgents_, transitionMatrices_, grid_);

        // Initialise the observation data.
        hmi::HMIObservation hmiObservation(hmiState);

        // Determine what action will be made from the given data.
        VectorFloat actionVec = observationRequest->action->as<VectorAction>()->asVector();
        std::set<std::string> targetAgents = hmiObservation.getUnderlyingState().getTargetAgents(actionVec);

        std::vector<std::string> shortestPaths(hmiState.getRobots().size());
        int maxShortestPath = 0;
        for (size_t i = 0; i != hmiState.getRobots().size(); ++i) {
            hmi::HMIRobot robot = hmiState.getRobots()[i];
            int robotX = robot.getCoordinates().getX();
            int robotY = robot.getCoordinates().getY();
            int actionX = (int) actionVec[2*i];
            int actionY = (int) actionVec[2*i + 1];
            std::pair<int, std::string> path = hmi::getShortestPath(grid_, robotX, robotY, actionX, actionY);
            shortestPaths[i] = path.second;
            maxShortestPath = std::max(path.first, maxShortestPath);
        }

        hmiObservation.sampleMovement(maxShortestPath, shortestPaths, targetAgents);

        // Populate the resulting observation and return it.
        VectorInt outVector = hmiObservation.toVector();
        VectorFloat floatOutVector(outVector.begin(), outVector.end());
        ObservationSharedPtr observation = std::make_shared<DiscreteVectorObservation>(floatOutVector);
        observationResult->observation = observation;
        observationResult->errorVector = observationRequest->errorVector;

        // // std::cout << "Completed method getObservation() in class HMIObservationPlugin...\n";

        return observationResult;
    }

private:
    std::vector<hmi::TypeAndId> randomAgents_;
    hmi::Grid grid_;
    std::unordered_map<std::string, hmi::TransitionMatrix> transitionMatrices_;
};

OPPT_REGISTER_OBSERVATION_PLUGIN(HMIObservationPlugin)

}
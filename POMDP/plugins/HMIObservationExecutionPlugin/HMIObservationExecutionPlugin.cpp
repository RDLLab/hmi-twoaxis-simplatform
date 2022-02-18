#include "oppt/plugin/Plugin.hpp"
#include "oppt/opptCore/Distribution.hpp"
#include "HMIObservationExecutionPluginOptions.hpp"
#include "plugins/HMIShared/HMIDataStructures.hpp"

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
class HMIObservationExecutionPlugin : public ObservationPlugin {

public:

    HMIObservationExecutionPlugin() : ObservationPlugin() { }

    virtual ~HMIObservationExecutionPlugin() = default;

    virtual bool load(const std::string &optionsFile) override {
        // std::cout << "Running method load() in HMIObservationExecutionPlugin..." << std::endl;
        parseOptions_<HMIObservationExecutionPluginOptions>(optionsFile);
        pipePathToGama_ = static_cast<HMIObservationExecutionPluginOptions*>(options_.get())->pipePathToGama;
        pipePathToSolver_ = static_cast<HMIObservationExecutionPluginOptions*>(options_.get())->pipePathToSolver;
        std::string randomAgentsPath
          = static_cast<HMIObservationExecutionPluginOptions*>(options_.get())->randomAgentsPath;
        randomAgents_ = hmi::instantiateTypesAndIDs(randomAgentsPath);
        // std::cout << "Completed method load() in HMIObservationExecutionPlugin..." << std::endl;
        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest * observationRequest) const override {
        // std::cout << "Running method getObservation() in HMIObservationExecutionPlugin..." << std::endl;

        // Create the pointer that will store the result of the observation to be made.
        ObservationResultSharedPtr observationResult = std::make_shared<ObservationResult>();

        std::string command = "cat < " + pipePathToSolver_;
        std::string observations = hmi::execute(command.c_str());
        // std::cout << "The observations are " << observations << std::endl;

        VectorFloat currentState = observationRequest->currentState->as<VectorState>()->asVector();
        int numObservations = robotEnvironment_->getRobot()->getObservationSpace()->getNumDimensions();
        VectorFloat observationVec(numObservations);

        for (size_t i = 0; i != numObservations; ++i) {
            // std::cout << "Remaining observations are " << observations << std::endl;
            observationVec[i] = (float) std::stoi(observations);
            observations = observations.substr(observations.find(",") + 1);
        }

        ObservationSharedPtr observation = std::make_shared<DiscreteVectorObservation>(observationVec);
        observationResult->observation = observation;
        observationResult->errorVector = observationRequest->errorVector;
        // std::cout << "Completed method getObservation() in HMIObservationExecutionPlugin..." << std::endl;
        return observationResult;
    }

private:

    std::string pipePathToGama_;
    std::string pipePathToSolver_;
    std::vector<hmi::TypeAndId> randomAgents_;

};

OPPT_REGISTER_OBSERVATION_PLUGIN(HMIObservationExecutionPlugin)

}
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
        parseOptions_<HMIObservationExecutionPluginOptions>(optionsFile);
        pipePathToSolver_ = static_cast<HMIObservationExecutionPluginOptions*>(options_.get())->pipePathToSolver;
        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest * observationRequest) const override {

        // Create the pointer that will store the result of the observation to be made.
        ObservationResultSharedPtr observationResult = std::make_shared<ObservationResult>();

        std::string command = "cat < " + pipePathToSolver_;
        std::string observations = hmi::execute(command.c_str());

        VectorFloat currentState = observationRequest->currentState->as<VectorState>()->asVector();
        int numObservations = robotEnvironment_->getRobot()->getObservationSpace()->getNumDimensions();
        VectorFloat observationVec(numObservations);

        for (size_t i = 0; i != numObservations; ++i) {
            observationVec[i] = (float) std::stoi(observations);
            observations = observations.substr(observations.find(",") + 1);
        }

        ObservationSharedPtr observation = std::make_shared<DiscreteVectorObservation>(observationVec);
        observationResult->state = observationRequest->currentState.get();
        observationResult->action = observationRequest->action;
        observationResult->observation = observation;
        observationResult->errorVector = observationRequest->errorVector;
        return observationResult;
    }

private:

    std::string pipePathToSolver_;

};

OPPT_REGISTER_OBSERVATION_PLUGIN(HMIObservationExecutionPlugin)

}
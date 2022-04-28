#include "oppt/plugin/Plugin.hpp"
#include "oppt/opptCore/Distribution.hpp"
#include "HMIObservationPluginOptions.hpp"
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

        std::cout << "Running method load() in class HMIObservationPlugin...\n";
        
        // Parse the options for this plugin.
        parseOptions_<HMIObservationPluginOptions>(optionsFile);

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

        std::string randId = randomAgents_[0].first;
        numConditions_ = transitionMatrices_.at(randId).numConditions_;
        
        std::cout << "Completed method load() in class HMIObservationPlugin...\n";

        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        
        std::cout << "Running method getObservation() in class HMIObservationPlugin...\n";
        // Create the pointer that will store the result of the observation to be made.
        ObservationResultSharedPtr obsResult = std::make_shared<ObservationResult>();

        // Convert the current state of the problem into a richer data format.
        VectorFloat stateVec = observationRequest->currentState->as<VectorState>()->asVector();
        VectorFloat actionVec = observationRequest->action->as<VectorAction>()->asVector();
        size_t robOffset = actionVec.size();

        VectorFloat obsVec = VectorFloat(randomAgents_.size());
        RandomEngine generator;
        std::uniform_real_distribution<float> obsDist(0, 1.0);

        for (size_t i = 0; i != obsVec.size(); ++i) {
            FloatType obs = obsDist(generator);
            int trueC = stateVec[3 * i + robOffset];
            if (obs < 0.8) obsVec[i] = trueC;
            else {
                int obsIdx = (int) ((obs - 0.8) * 5 * (numConditions_ - 1));
                obsVec[i] = obsIdx >= trueC ? obsIdx + 1 : obsIdx;
            }
        }

        ObservationSharedPtr observation = std::make_shared<DiscreteVectorObservation>(obsVec);
        obsResult->state = observationRequest->currentState.get();
        obsResult->action = observationRequest->action;
        obsResult->observation = observation;
        obsResult->errorVector = observationRequest->errorVector;

        std::cout << "Completed method getObservation() in class HMIObservationPlugin...\n";

        return obsResult;
    }

private:
    std::vector<hmi::TypeAndId> randomAgents_;
    std::unordered_map<std::string, hmi::TransitionMatrix> transitionMatrices_;
    int numConditions_;
};

OPPT_REGISTER_OBSERVATION_PLUGIN(HMIObservationPlugin)

}

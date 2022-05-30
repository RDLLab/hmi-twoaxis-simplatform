#include "oppt/plugin/Plugin.hpp"
#include "oppt/opptCore/Distribution.hpp"
#include "HMIObservationPluginOptions.hpp"
#include "plugins/HMIShared/HMIDataStructures.hpp"
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
        
        // Parse the options for this plugin.
        parseOptions_<HMIObservationPluginOptions>(optionsFile);

        // Extract random agent data for the current problem and enrich it into
        // a richer data structure.
        std::string requestersPath
            = static_cast<HMIObservationPluginOptions*>(options_.get())->requestersPath;
        requesters_ = hmi::instantiateTypesAndIDs(requestersPath);

        zeta_ = static_cast<HMIObservationPluginOptions*>(options_.get())->zeta;

        // Extract transition matrix data for the current problem and enrich it into a
        // data structure that is easier to use.
        std::string transitionMatricesPath
            = static_cast<HMIObservationPluginOptions*>(options_.get())->transitionMatrixPath;
        transitionMatrices_ = hmi::instantiateTransitionMatrices(transitionMatricesPath);

        std::string randId = requesters_[0].first;
        numConditions_ = transitionMatrices_.at(randId).numConditions_;

        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        // Create the pointer that will store the result of the observation to be made.
        ObservationResultSharedPtr obsResult = std::make_shared<ObservationResult>();

        // Convert the current state of the problem into a richer data format.
        VectorFloat stateVec = observationRequest->currentState->as<VectorState>()->asVector();
        VectorFloat actionVec = observationRequest->action->as<VectorAction>()->asVector();
        size_t robOffset = actionVec.size();

        VectorFloat obsVec = VectorFloat(requesters_.size());
        std::random_device rd;
        RandomEngine generator(rd());
        std::uniform_real_distribution<float> obsDist(0, 1.0);

        for (size_t i = 0; i != obsVec.size(); ++i) {
            FloatType obs = obsDist(generator);
            int trueC = stateVec[3 * i + robOffset + 2];
            if (obs < zeta_) obsVec[i] = trueC;
            else {
                int obsIdx = (int) ((obs - zeta_) * 5 * (numConditions_ - 1));
                int observation = obsIdx >= trueC ? obsIdx + 1 : obsIdx;
                obsVec[i] = observation;
            }
        }

        ObservationSharedPtr observation = std::make_shared<DiscreteVectorObservation>(obsVec);
        obsResult->state = observationRequest->currentState.get();
        obsResult->action = observationRequest->action;
        obsResult->observation = observation;
        obsResult->errorVector = observationRequest->errorVector;

        return obsResult;
    }

private:
    std::vector<hmi::TypeAndId> requesters_;
    std::unordered_map<std::string, hmi::TransitionMatrix> transitionMatrices_;
    int numConditions_;
    FloatType zeta_;
};

OPPT_REGISTER_OBSERVATION_PLUGIN(HMIObservationPlugin)

}

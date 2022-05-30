#include "oppt/plugin/Plugin.hpp"
#include "oppt/opptCore/Distribution.hpp"
#include "HMITransitionExecutionPluginOptions.hpp"
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
#include <utility>
#include <math.h>

namespace oppt
{

class HMITransitionExecutionPlugin : public TransitionPlugin {

public:

    HMITransitionExecutionPlugin() : TransitionPlugin() { }

    virtual ~HMITransitionExecutionPlugin() = default;

    virtual bool load(const std::string &optionsFile) override {
        parseOptions_<HMITransitionExecutionPluginOptions>(optionsFile);
        pipePathToGama_ = static_cast<HMITransitionExecutionPluginOptions*>(options_.get())->pipePathToGama;
        pipePathToSolver_ = static_cast<HMITransitionExecutionPluginOptions*>(options_.get())->pipePathToSolver;
        std::string requestersPath
          = static_cast<HMITransitionExecutionPluginOptions*>(options_.get())->requestersPath;
        requesters_ = hmi::instantiateTypesAndIDs(requestersPath);
        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        PropagationResultSharedPtr propagationResult(new PropagationResult());

        VectorFloat currentState(propagationRequest->currentState->as<VectorState>()->asVector());
        VectorFloat actionVec = propagationRequest->action->as<VectorAction>()->asVector();

        std::string cmd = "echo -n ";

        for (size_t i = 0; i != actionVec.size() / 2; ++i) {
            int xAction = (int) actionVec[2*i];
            int yAction = (int) actionVec[2*i + 1];

            cmd += std::to_string(xAction) + "," + std::to_string(yAction) + ",";
        }

        cmd = cmd.substr(0, cmd.size() - 1) + " > " + pipePathToGama_;
        FILE * pipeToGama(popen(cmd.c_str(), "w"));
        pclose(pipeToGama);

        std::string resultingCmd = "cat < " + pipePathToSolver_;
        std::string resultingState = hmi::execute(resultingCmd.c_str());
        VectorFloat nextState(currentState.size());
        for (size_t i = 0; i != currentState.size(); ++i) {
            nextState[i] = (float) std::stof(resultingState);
            resultingState = resultingState.substr(resultingState.find(",") + 1);
        }

        propagationResult->previousState = propagationRequest->currentState.get();
        propagationResult->action = propagationRequest->action;
        propagationResult->nextState = std::make_shared<oppt::VectorState>(nextState);

        return propagationResult;
    }

private:

    std::string pipePathToGama_;
    std::string pipePathToSolver_;

    std::vector<hmi::TypeAndId> requesters_;

};

OPPT_REGISTER_TRANSITION_PLUGIN(HMITransitionExecutionPlugin)

}
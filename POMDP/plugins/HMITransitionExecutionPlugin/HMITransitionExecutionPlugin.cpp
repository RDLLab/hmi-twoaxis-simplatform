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
        // std::cout << "Running method load() in HMITransitionExecutionPlugin..." << std::endl;
        parseOptions_<HMITransitionExecutionPluginOptions>(optionsFile);
        pipePathToGama_ = static_cast<HMITransitionExecutionPluginOptions*>(options_.get())->pipePathToGama;
        pipePathToSolver_ = static_cast<HMITransitionExecutionPluginOptions*>(options_.get())->pipePathToSolver;
        std::string randomAgentsPath
          = static_cast<HMITransitionExecutionPluginOptions*>(options_.get())->randomAgentsPath;
        randomAgents_ = hmi::instantiateTypesAndIDs(randomAgentsPath);
        // std::cout << "Completed method load() in HMITransitionExecutionPlugin..." << std::endl;
        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        // std::cout << "Running method propagateState() in HMITransitionExecutionPlugin..." << std::endl;
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
        // std::cout << "About to send action to solver..." << std::endl;
        FILE * pipeToGama(popen(cmd.c_str(), "w"));
        pclose(pipeToGama);

        std::string resultingCmd = "cat < " + pipePathToSolver_;
        std::string resultingState = hmi::execute(resultingCmd.c_str());
        // std::cout << "Resulting state is " << resultingState << std::endl;
        VectorFloat nextState(currentState.size());
        for (size_t i = 0; i != currentState.size(); ++i) {
            // std::cout << "Remaining state is " << resultingState << std::endl;
            nextState[i] = (float) std::stof(resultingState);
            resultingState = resultingState.substr(resultingState.find(",") + 1);
        }

        propagationResult->previousState = propagationRequest->currentState.get();
        propagationResult->action = propagationRequest->action;
        propagationResult->nextState = std::make_shared<oppt::VectorState>(nextState);

        // std::cout << "Completed method propagateState() in HMITransitionExecutionPlugin..." << std::endl;

        return propagationResult;
    }

private:

    std::string pipePathToGama_;
    std::string pipePathToSolver_;

    std::vector<hmi::TypeAndId> randomAgents_;

};

OPPT_REGISTER_TRANSITION_PLUGIN(HMITransitionExecutionPlugin)

}
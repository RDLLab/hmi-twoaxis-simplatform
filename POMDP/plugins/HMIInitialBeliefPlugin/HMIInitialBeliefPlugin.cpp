#ifndef _HMI_INITIAL_STATE_SAMPLER_PLUGIN_HPP_
#define _HMI_INITIAL_STATE_SAMPLER_PLUGIN_HPP_
#include "oppt/plugin/Plugin.hpp"
#include "HMIInitialBeliefOptions.hpp"
#include "oppt/gazeboInterface/GazeboInterface.hpp"

#include <string>
#include <array>
#include <memory>
#include <stdio.h>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <map>
#include <unordered_map>

namespace oppt
{
class HMIInitialBeliefPlugin: public InitialBeliefPlugin
{
public:

    HMIInitialBeliefPlugin():
        InitialBeliefPlugin(), initialStateVec_() {

    }

    virtual ~HMIInitialBeliefPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        std::cout << "Running method load() in class HMIInitialBeliefPlugin...\n";
        parseOptions_<HMIInitialBeliefOptions>(optionsFile);
        initialStateVec_ = static_cast<HMIInitialBeliefOptions*>(options_.get())->initialRobotStateVec;
        VectorFloat initialRandomAgentState = static_cast<HMIInitialBeliefOptions*>(options_.get())->initialRandomAgentStateVec;
        initialStateVec_.insert(std::end(initialStateVec_), std::begin(initialRandomAgentState), std::end(initialRandomAgentState));
        std::cout << "Completed method load() in class HMIInitialBeliefPlugin...\n";
        return true;
    }

    virtual RobotStateSharedPtr sampleAnInitState() override {
        std::cout << "Running method sampleAnInitState() in class HMIInitialBeliefPlugin...\n";
        RobotStateSharedPtr initState(new VectorState(initialStateVec_));
        std::cout << "Completed method sampleAnInitState() in class HMIInitialBeliefPlugin...\n";
        return initState;
    }

private:

    VectorFloat initialStateVec_;
};

OPPT_REGISTER_INITIAL_BELIEF_PLUGIN(HMIInitialBeliefPlugin)

}

#endif
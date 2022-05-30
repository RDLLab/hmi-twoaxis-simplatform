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
        parseOptions_<HMIInitialBeliefOptions>(optionsFile);
        initialStateVec_ = static_cast<HMIInitialBeliefOptions*>(options_.get())->initialRobotStateVec;
        VectorFloat initialRequesterState = static_cast<HMIInitialBeliefOptions*>(options_.get())->initialRequesterStateVec;
        initialStateVec_.insert(std::end(initialStateVec_), std::begin(initialRequesterState), std::end(initialRequesterState));
        return true;
    }

    virtual RobotStateSharedPtr sampleAnInitState() override {
        RobotStateSharedPtr initState(new VectorState(initialStateVec_));
        return initState;
    }

private:

    VectorFloat initialStateVec_;
};

OPPT_REGISTER_INITIAL_BELIEF_PLUGIN(HMIInitialBeliefPlugin)

}

#endif
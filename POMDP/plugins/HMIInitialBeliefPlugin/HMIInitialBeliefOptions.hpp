#ifndef _HMI_INITIAL_STATE_SAMPLER_OPTIONS_HPP_
#define _HMI_INITIAL_STATE_SAMPLER_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class HMIInitialBeliefOptions: public PluginOptions
{
public:
    HMIInitialBeliefOptions() = default;

    virtual ~HMIInitialBeliefOptions() = default;

    VectorFloat initialRobotStateVec;
    VectorFloat initialRandomAgentStateVec;

    static std::unique_ptr<options::OptionParser> makeParser() {
        // std::cout << "Running method makeParser() in class HMIInitialBeliefOptions...\n";
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addHMIInitialBeliefOptions(parser.get());
        // std::cout << "Completed method makeParser() in class HMIInitialBeliefOptions...\n";
        return std::move(parser);
    }

    static void addHMIInitialBeliefOptions(options::OptionParser* parser) {    
        // std::cout << "Running method addHMIInitialBeliefOptions() in class HMIInitialBeliefOptions...\n";
        parser->addOption<VectorFloat>("initialBeliefOptions",
                                       "initialRobotState",
                                       &HMIInitialBeliefOptions::initialRobotStateVec);	
        parser->addOption<VectorFloat>("initialBeliefOptions",
                                       "initialRandomAgentState",
                                       &HMIInitialBeliefOptions::initialRandomAgentStateVec);
        // std::cout << "Completed method addHMIInitialBeliefOptions() in class HMIInitialBeliefOptions...\n";
    }

    const std::string BASE_PATH = "../oppt_install/oppt/";

};
}

#endif

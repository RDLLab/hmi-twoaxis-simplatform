#ifndef _HMI_TRANSITION_EXECUTION_PLUGIN_OPTIONS_HPP_
#define _HMI_TRANSITION_EXECUTION_PLUGIN_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class HMITransitionExecutionPluginOptions: public PluginOptions
{
public:
    HMITransitionExecutionPluginOptions() = default;

    virtual ~HMITransitionExecutionPluginOptions() = default;

    std::string requestersPath = BASE_PATH + "models/HMIModel/HMIRequesters.txt";

    std::string pipePathToGama = BASE_PATH + "pipes/pipeToGama";

    std::string pipePathToSolver = BASE_PATH + "pipes/statePipeToSolver";

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser = PluginOptions::makeParser();
        addHMITransitionExecutionPluginOptions(parser.get());
        return std::move(parser);
    }

    static void addHMITransitionExecutionPluginOptions(options::OptionParser* parser) {
        parser->addOption<std::string>("transitionExecutionPluginOptions",
                                       "requestersPath",
                                       &HMITransitionExecutionPluginOptions::requestersPath);
        parser->addOption<std::string>("transitionExecutionPluginOptions",
                                       "pipePathToGama",
                                       &HMITransitionExecutionPluginOptions::pipePathToGama);
        parser->addOption<std::string>("transitionExecutionPluginOptions",
                                       "pipePathToSolver",
                                       &HMITransitionExecutionPluginOptions::pipePathToSolver);
    }

private:

    const std::string BASE_PATH = "../oppt_install/oppt/";

};
}

#endif

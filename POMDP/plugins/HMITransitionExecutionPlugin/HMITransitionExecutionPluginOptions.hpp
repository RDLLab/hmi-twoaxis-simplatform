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

    std::string randomAgentsPath = BASE_PATH + "models/HMIModel/HMIRandomAgents.txt";

    std::string pipePathToGama = BASE_PATH + "pipes/pipeToGama";

    std::string pipePathToSolver = BASE_PATH + "pipes/statePipeToSolver";

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::cout << "Running method makeParser() in HMITransitionExecutionPluginOptions..." << std::endl;
        std::unique_ptr<options::OptionParser> parser = PluginOptions::makeParser();
        addHMITransitionExecutionPluginOptions(parser.get());
        std::cout << "Completed method makeParser() in HMITransitionExecutionPluginOptions..." << std::endl;
        return std::move(parser);
    }

    static void addHMITransitionExecutionPluginOptions(options::OptionParser* parser) {
        std::cout << "Running method addHMITransitionExecutionPluginOptions() in HMITransitionExecutionPluginOptions..." << std::endl;
        parser->addOption<std::string>("transitionExecutionPluginOptions",
                                       "randomAgentsPath",
                                       &HMITransitionExecutionPluginOptions::randomAgentsPath);
        parser->addOption<std::string>("transitionExecutionPluginOptions",
                                       "pipePathToGama",
                                       &HMITransitionExecutionPluginOptions::pipePathToGama);
        parser->addOption<std::string>("transitionExecutionPluginOptions",
                                       "pipePathToSolver",
                                       &HMITransitionExecutionPluginOptions::pipePathToSolver);
        std::cout << "Completed method addHMITransitionExecutionPluginOptions() in HMITransitionExecutionPluginOptions..." << std::endl;
    }

private:

    const std::string BASE_PATH = "../oppt_install/oppt/";

};
}

#endif

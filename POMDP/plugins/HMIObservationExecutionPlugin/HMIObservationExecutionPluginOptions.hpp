#ifndef _HMI_OBSERVATION_EXECUTION_PLUGIN_OPTIONS_HPP_
#define _HMI_OBSERVATION_EXECUTION_PLUGIN_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class HMIObservationExecutionPluginOptions : public PluginOptions {

public:

    HMIObservationExecutionPluginOptions() = default;

    virtual ~HMIObservationExecutionPluginOptions() = default;

    std::string randomAgentsPath = BASE_PATH + "models/HMIModel/HMIRandomAgents.txt";

    std::string pipePathToGama = BASE_PATH + "pipes/pipeToGama";

    std::string pipePathToSolver = BASE_PATH + "pipes/observationPipeToSolver";

    static std::unique_ptr<options::OptionParser> makeParser() {
        // std::cout << "Running method load() in HMIObservationExecutionPluginOptions..." << std::endl;
        std::unique_ptr<options::OptionParser> parser = PluginOptions::makeParser();
        addHMIObservationExecutionPluginOptions(parser.get());
        // std::cout << "Completed method load() in HMIObservationExecutionPluginOptions..." << std::endl;
        return std::move(parser);
    }

    static void addHMIObservationExecutionPluginOptions(options::OptionParser* parser) {
        // std::cout << "Running method addHMIObservationExecutionPluginOptions() in HMIObservationExecutionPluginOptions..." << std::endl;
        parser->addOption<std::string>("observationExecutionPluginOptions",
                                       "randomAgentsPath",
                                       &HMIObservationExecutionPluginOptions::randomAgentsPath);
        parser->addOption<std::string>("observationExecutionPluginOptions",
                                       "pipePathToGama",
                                       &HMIObservationExecutionPluginOptions::pipePathToGama);
        parser->addOption<std::string>("observationExecutionPluginOptions",
                                       "pipePathToSolver",
                                       &HMIObservationExecutionPluginOptions::pipePathToSolver);
        // std::cout << "Completed method addHMIObservationExecutionPluginOptions() in HMIObservationExecutionPluginOptions..." << std::endl;
    }

private:

    const std::string BASE_PATH = "../oppt_install/oppt/";

};
}

#endif
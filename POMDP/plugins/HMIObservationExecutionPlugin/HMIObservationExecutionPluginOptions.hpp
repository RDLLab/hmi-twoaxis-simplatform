#ifndef _HMI_OBSERVATION_EXECUTION_PLUGIN_OPTIONS_HPP_
#define _HMI_OBSERVATION_EXECUTION_PLUGIN_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class HMIObservationExecutionPluginOptions : public PluginOptions {

public:

    HMIObservationExecutionPluginOptions() = default;

    virtual ~HMIObservationExecutionPluginOptions() = default;

    std::string requestersPath = BASE_PATH + "models/HMIModel/HMIRequesters.txt";

    std::string pipePathToGama = BASE_PATH + "pipes/pipeToGama";

    std::string pipePathToSolver = BASE_PATH + "pipes/observationPipeToSolver";

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser = PluginOptions::makeParser();
        addHMIObservationExecutionPluginOptions(parser.get());
        return std::move(parser);
    }

    static void addHMIObservationExecutionPluginOptions(options::OptionParser* parser) {
        parser->addOption<std::string>("observationExecutionPluginOptions",
                                       "requestersPath",
                                       &HMIObservationExecutionPluginOptions::requestersPath);
        parser->addOption<std::string>("observationExecutionPluginOptions",
                                       "pipePathToGama",
                                       &HMIObservationExecutionPluginOptions::pipePathToGama);
        parser->addOption<std::string>("observationExecutionPluginOptions",
                                       "pipePathToSolver",
                                       &HMIObservationExecutionPluginOptions::pipePathToSolver);
    }

private:

    const std::string BASE_PATH = "../oppt_install/oppt/";

};
}

#endif
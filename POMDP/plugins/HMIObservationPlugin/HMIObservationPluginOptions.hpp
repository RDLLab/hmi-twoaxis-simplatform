#ifndef _HMI_OBSERVATION_PLUGIN_OPTIONS_HPP_
#define _HMI_OBSERVATION_PLUGIN_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class HMIObservationPluginOptions: public PluginOptions
{
public:
    HMIObservationPluginOptions() = default;

    virtual ~HMIObservationPluginOptions() = default;

    /** @brief The filepath to load the problem's grid environment. */
    std::string gridPath = BASE_PATH + "models/HMIModel/HMIGrid.txt";

    std::string randomAgentsPath = BASE_PATH + "models/HMIModel/HMIRandomAgents.txt";

    /** @brief The filepath to load the state transition matrices for each
     *  dependent agent. */
    std::string transitionMatrixPath = BASE_PATH + "models/HMIModel/HMITransitionMatrices.txt";    

    static std::unique_ptr<options::OptionParser> makeParser() {
        // std::cout << "Running method makeParser() in class HMIObservationPluginOptions...\n";
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addHMIObservationPluginOptions(parser.get());
        // std::cout << "Completed method makeParser() in class HMIObservationPluginOptions...\n";
        return std::move(parser);
    }

    static void addHMIObservationPluginOptions(options::OptionParser* parser) {
        // std::cout << "Running method addHMIObservationPluginOptions() in class HMIObservationPluginOptions...\n";
        parser->addOption<std::string>("observationPluginOptions",
                                       "gridPath",
                                       &HMIObservationPluginOptions::gridPath);
        parser->addOption<std::string>("observationPluginOptions",
                                       "randomAgentsPath",
                                       &HMIObservationPluginOptions::randomAgentsPath);
        parser->addOption<std::string>("observationPluginOptions",
                                       "transitionMatrixPath",
                                       &HMIObservationPluginOptions::transitionMatrixPath);
        // std::cout << "Completed method addHMIObservationPluginOptions() in class HMIObservationPluginOptions...\n";
    }

    const std::string BASE_PATH = "../oppt_install/oppt/";

};
}

#endif
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

    std::string requestersPath = BASE_PATH + "models/HMIModel/HMIRequesters.txt";

    /** @brief The filepath to load the state transition matrices for each
     *  requester. */
    std::string transitionMatrixPath = BASE_PATH + "models/HMIModel/HMITransitionMatrices.txt";

    FloatType zeta = 0.8;

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addHMIObservationPluginOptions(parser.get());
        return std::move(parser);
    }

    static void addHMIObservationPluginOptions(options::OptionParser* parser) {
        parser->addOption<std::string>("observationPluginOptions",
                                       "gridPath",
                                       &HMIObservationPluginOptions::gridPath);
        parser->addOption<std::string>("observationPluginOptions",
                                       "requestersPath",
                                       &HMIObservationPluginOptions::requestersPath);
        parser->addOption<std::string>("observationPluginOptions",
                                       "transitionMatrixPath",
                                       &HMIObservationPluginOptions::transitionMatrixPath);
        parser->addOption<FloatType>("observationPluginOptions",
                                     "zeta",
                                     &HMIObservationPluginOptions::zeta);
    }

    const std::string BASE_PATH = "../oppt_install/oppt/";

};
}

#endif
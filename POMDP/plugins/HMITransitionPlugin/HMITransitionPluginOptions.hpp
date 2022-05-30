#ifndef _HMI_TRANSITION_PLUGIN_OPTIONS_HPP_
#define _HMI_TRANSITION_PLUGIN_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class HMITransitionPluginOptions: public PluginOptions
{
public:
    HMITransitionPluginOptions() = default;

    virtual ~HMITransitionPluginOptions() = default;

    /** @brief The filepath to load the problem's grid environment. */
    std::string gridPath = BASE_PATH + "models/HMIModel/HMIGrid.txt";

    std::string requestersPath = BASE_PATH + "models/HMIModel/HMIRequesters.txt";

    /** @brief The filepath to load the state transition matrices for each
     *  requester. */
    std::string transitionMatrixPath = BASE_PATH + "models/HMIModel/HMITransitionMatrices.txt";

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser = PluginOptions::makeParser();
        addHMITransitionPluginOptions(parser.get());
        return std::move(parser);
    }

    static void addHMITransitionPluginOptions(options::OptionParser* parser) {
        parser->addOption<std::string>("transitionPluginOptions",
                                       "gridPath",
                                       &HMITransitionPluginOptions::gridPath);
        parser->addOption<std::string>("transitionPluginOptions",
                                       "requestersPath",
                                       &HMITransitionPluginOptions::requestersPath);
        parser->addOption<std::string>("transitionPluginOptions",
                                       "transitionMatrixPath",
                                       &HMITransitionPluginOptions::transitionMatrixPath);
    }

private:

    const std::string BASE_PATH = "../oppt_install/oppt/";

};
}

#endif

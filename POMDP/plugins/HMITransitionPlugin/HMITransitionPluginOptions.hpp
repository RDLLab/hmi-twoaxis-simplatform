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

    std::string randomAgentsPath = BASE_PATH + "models/HMIModel/HMIRandomAgents.txt";

    /** @brief The filepath to load the state transition matrices for each
     *  dependent agent. */
    std::string transitionMatrixPath = BASE_PATH + "models/HMIModel/HMITransitionMatrices.txt";

    static std::unique_ptr<options::OptionParser> makeParser() {
        // std::cout << "Running method makeParser() in HMITransitionPluginOptions..." << std::endl;
        std::unique_ptr<options::OptionParser> parser = PluginOptions::makeParser();
        addHMITransitionPluginOptions(parser.get());
        // std::cout << "Completed method makeParser() in HMITransitionPluginOptions..." << std::endl;
        return std::move(parser);
    }

    static void addHMITransitionPluginOptions(options::OptionParser* parser) {
        // std::cout << "Running method addHMITransitionPluginOptions() in HMITransitionPluginOptions..." << std::endl;
        parser->addOption<std::string>("transitionPluginOptions",
                                       "gridPath",
                                       &HMITransitionPluginOptions::gridPath);
        parser->addOption<std::string>("transitionPluginOptions",
                                       "randomAgentsPath",
                                       &HMITransitionPluginOptions::randomAgentsPath);
        parser->addOption<std::string>("transitionPluginOptions",
                                       "transitionMatrixPath",
                                       &HMITransitionPluginOptions::transitionMatrixPath);
        // std::cout << "Completed method addHMITransitionPluginOptions() in HMITransitionPluginOptions..." << std::endl;
    }

private:

    const std::string BASE_PATH = "../oppt_install/oppt/";

};
}

#endif

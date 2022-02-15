#ifndef _HMI_HEURISTIC_OPTIONS_HPP_
#define _HMI_HEURISTIC_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class HMIHeuristicOptions: public PluginOptions
{
public:
    HMIHeuristicOptions() = default;
    virtual ~HMIHeuristicOptions() = default;

    /** @brief The filepath to load the problem's grid environment. */
    std::string gridPath = BASE_PATH + "models/HMIModel/HMIGrid.txt";

    std::string randomAgentsPath = BASE_PATH + "models/HMIModel/HMIRandomAgents.txt";

    /** @brief The filepath to load the state transition matrices for each
     *  dependent agent. */
    std::string transitionMatrixPath = BASE_PATH + "models/HMIModel/HMITransitionMatrices.txt";

    static std::unique_ptr<options::OptionParser> makeParser() {
        // std::cout << "Running method makeParser() in class HMIHeuristicOptions...\n";
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addHMIHeuristicPluginOptions(parser.get());
        // std::cout << "Completed method makeParser() in class HMIHeuristicOptions...\n";
        return std::move(parser);
    }

    static void addHMIHeuristicPluginOptions(options::OptionParser* parser) {
        // std::cout << "Running method addHMIHeuristicPluginOptions() in class HMIHeuristicOptions...\n";
        parser->addOption<std::string>("heuristicPluginOptions",
                                       "gridPath",
                                       &HMIHeuristicOptions::gridPath);
        parser->addOption<std::string>("heuristicPluginOptions",
                                       "randomAgentsPath",
                                       &HMIHeuristicOptions::randomAgentsPath);
        parser->addOption<std::string>("heuristicPluginOptions",
                                       "transitionMatrixPath",
                                       &HMIHeuristicOptions::transitionMatrixPath);
        // std::cout << "Completed method addHMIHeuristicPluginOptions() in class HMIHeuristicOptions...\n";
    }

    const std::string BASE_PATH = "../oppt_install/oppt/";

};
}

#endif

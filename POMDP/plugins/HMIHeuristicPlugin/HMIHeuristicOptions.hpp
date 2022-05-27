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

    std::string requestersPath = BASE_PATH + "models/HMIModel/HMIRequesters.txt";

    /** @brief The filepath to load the state transition matrices for each
     *  requester. */
    std::string transitionMatrixPath = BASE_PATH + "models/HMIModel/HMITransitionMatrices.txt";

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addHMIHeuristicPluginOptions(parser.get());
        return std::move(parser);
    }

    static void addHMIHeuristicPluginOptions(options::OptionParser* parser) {
        parser->addOption<std::string>("heuristicPluginOptions",
                                       "gridPath",
                                       &HMIHeuristicOptions::gridPath);
        parser->addOption<std::string>("heuristicPluginOptions",
                                       "requestersPath",
                                       &HMIHeuristicOptions::requestersPath);
        parser->addOption<std::string>("heuristicPluginOptions",
                                       "transitionMatrixPath",
                                       &HMIHeuristicOptions::transitionMatrixPath);
    }

    const std::string BASE_PATH = "../oppt_install/oppt/";

};
}

#endif

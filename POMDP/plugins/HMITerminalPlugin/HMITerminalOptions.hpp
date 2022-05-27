#ifndef _HMI_TERMINAL_OPTIONS_HPP_
#define _HMI_TERMINAL_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{
class HMITerminalOptions: public PluginOptions
{
public:
    HMITerminalOptions() = default;

    virtual ~HMITerminalOptions() = default;

    /** @brief The filepath to load the problem's grid environment. */
    std::string gridPath = BASE_PATH + "models/HMIModel/HMIGrid.txt";

    std::string requestersPath = BASE_PATH + "models/HMIModel/HMIRequesters.txt";

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addHMITerminalOptions(parser.get());
        return std::move(parser);
    }

    static void addHMITerminalOptions(options::OptionParser* parser) {
        parser->addOption<std::string>("terminalPluginOptions",
                                       "gridPath",
                                       &HMITerminalOptions::gridPath);
        parser->addOption<std::string>("terminalPluginOptions",
                                       "requestersPath",
                                       &HMITerminalOptions::requestersPath);
    }

    const std::string BASE_PATH = "../oppt_install/oppt/";

};
}

#endif

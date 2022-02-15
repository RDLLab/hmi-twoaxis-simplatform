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

    static std::unique_ptr<options::OptionParser> makeParser() {
        // std::cout << "Running method makeParser() in class HMITerminalOptions...\n";
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addHMITerminalOptions(parser.get());
        // std::cout << "Completed method makeParser() in class HMITerminalOptions...\n";
        return std::move(parser);
    }

    static void addHMITerminalOptions(options::OptionParser* parser) {
        // std::cout << "Running method addHMITerminalOptions() in class HMITerminalOptions...\n";
        parser->addOption<std::string>("terminalPluginOptions",
                                       "gridPath",
                                       &HMITerminalOptions::gridPath);
        // std::cout << "Completed method addHMITerminalOptions() in class HMITerminalOptions...\n";
    }

    const std::string BASE_PATH = "../oppt_install/oppt/";

};
}

#endif

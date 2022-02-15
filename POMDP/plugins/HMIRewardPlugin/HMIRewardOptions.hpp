#ifndef _HMI_REWARD_OPTIONS_HPP_
#define _HMI_REWARD_OPTIONS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{

class HMIRewardOptions: public PluginOptions
{
public:
    HMIRewardOptions() = default;
    virtual ~HMIRewardOptions() = default;

    std::string gridPath = BASE_PATH + "models/HMIModel/HMIGrid.txt";

    std::string randomAgentsPath = BASE_PATH + "models/HMIModel/HMIRandomAgents.txt";

    static std::unique_ptr<options::OptionParser> makeParser() {
        // std::cout << "Running method makeParser() in class HMIRewardOptions...\n";
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addHMIRewardOptions(parser.get());
        // std::cout << "Completed method makeParser() in class HMIRewardOptions...\n";
        return std::move(parser);
    }

    static void addHMIRewardOptions(options::OptionParser* parser) {
        // std::cout << "Running method addHMIRewardOptions() in class HMIRewardOptions...\n";
        parser->addOption<std::string>("rewardPluginOptions",
                                       "gridPath",
                                       &HMIRewardOptions::gridPath);
        parser->addOption<std::string>("rewardPluginOptions",
                                       "randomAgentsPath",
                                       &HMIRewardOptions::randomAgentsPath);
        // std::cout << "Completed method addHMIRewardOptions() in class HMIRewardOptions...\n";
    }

    const std::string BASE_PATH = "../oppt_install/oppt/";

};

}

#endif

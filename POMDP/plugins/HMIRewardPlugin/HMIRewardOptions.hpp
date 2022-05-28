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

    std::string requestersPath = BASE_PATH + "models/HMIModel/HMIRequesters.txt";

    FloatType rho = 100.0;

    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addHMIRewardOptions(parser.get());
        return std::move(parser);
    }

    static void addHMIRewardOptions(options::OptionParser* parser) {
        parser->addOption<std::string>("rewardPluginOptions",
                                       "gridPath",
                                       &HMIRewardOptions::gridPath);
        parser->addOption<std::string>("rewardPluginOptions",
                                       "requestersPath",
                                       &HMIRewardOptions::requestersPath);
        parser->addOption<FloatType>("rewardPluginOptions",
                                     "rho",
                                     &HMIRewardOptions::rho);
    }

    const std::string BASE_PATH = "../oppt_install/oppt/";

};

}

#endif

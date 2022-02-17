#include "oppt/plugin/Plugin.hpp"
#include "HMIRewardOptions.hpp"
#include "plugins/HMIShared/HMIState.hpp"

#include <string>
#include <array>
#include <memory>
#include <stdio.h>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <map>
#include <unordered_map>

namespace oppt
{
class HMIRewardPlugin: public RewardPlugin {

public:
    HMIRewardPlugin(): RewardPlugin() {}

    virtual ~HMIRewardPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        std::cout << "Running method load() in class HMIRewardPlugin...\n";
        parseOptions_<HMIRewardOptions>(optionsFile);
        std::string gridPath
            = static_cast<HMIRewardOptions*>(options_.get())->gridPath;
        grid_ = hmi::instantiateGrid(gridPath);
        std::string randomAgentsPath
            = static_cast<HMIRewardOptions*>(options_.get())->randomAgentsPath;
        randomAgents_ = hmi::instantiateTypesAndIDs(randomAgentsPath);
        std::cout << "Completed method load() in class HMIRewardPlugin...\n";
        return true;
    }

    virtual FloatType getReward(const PropagationResultSharedPtr& propagationResult) const override {
        std::cout << "Running method getReward() in class HMIRewardPlugin...\n";

        // Extract data from propagated state.
        VectorFloat currentStateVector = propagationResult->nextState->as<VectorState>()->asVector();
        VectorFloat actionVec = propagationResult->action->as<VectorAction>()->asVector();

        // Convert the state into an easier-to-use data structure.
        hmi::HMIState currentState(currentStateVector, randomAgents_, grid_);

        // Initialise resulting reward value.
        FloatType reward = 0.0;

        for (size_t i = 0; i != actionVec.size(); i += 2) {
            hmi::Coordinate action((int) actionVec[i], (int) actionVec[i+1]);
            bool invalidCell = !currentState.getGrid().getGrid()[action.toPosition(currentState.getGrid())];
            if (invalidCell) return hmi::MIN_REWARD;
        }

        for (hmi::HMIRandomAgent randomAgent : currentState.getRandomAgents()) {
            reward += randomAgent.getCondition() == 0 ? hmi::BASE_REWARD : 0.0;
        }
        
        std::cout << "Completed method getReward() in class HMIRewardPlugin...\n";
        return reward;
    }

    virtual std::pair<FloatType, FloatType> getMinMaxReward() const override {
        std::cout << "Running and completing method getMinMaxReward() in class HMIRewardPlugin...\n";
        return std::make_pair(hmi::MIN_REWARD, randomAgents_.size() * hmi::BASE_REWARD);
    }


private:

    hmi::Grid grid_;
    std::vector<hmi::TypeAndId> randomAgents_;

};

OPPT_REGISTER_REWARD_PLUGIN(HMIRewardPlugin)

}

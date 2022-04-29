#include "oppt/plugin/Plugin.hpp"
#include "HMIRewardOptions.hpp"
#include "plugins/HMIShared/HMIState.hpp"
#include "plugins/HMIShared/ShortestPaths.hpp"

#include <string>
#include <array>
#include <memory>
#include <stdio.h>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <map>
#include <unordered_map>
#include <algorithm>

namespace oppt
{
class HMIRewardPlugin: public RewardPlugin {

public:
    HMIRewardPlugin(): RewardPlugin() {}

    virtual ~HMIRewardPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        // std::cout << "Running method load() in class HMIRewardPlugin...\n";
        parseOptions_<HMIRewardOptions>(optionsFile);
        std::string gridPath
            = static_cast<HMIRewardOptions*>(options_.get())->gridPath;
        grid_ = hmi::instantiateGrid(gridPath);
        shortestPaths_ = hmi::ShortestPaths(grid_);
        helpReward = shortestPaths_.getLongestPath() + 1;
        illegalMovePenalty = -100.0 * helpReward;
        // std::cout << "Completed method load() in class HMIRewardPlugin...\n";
        return true;
    }

    virtual FloatType getReward(const PropagationResultSharedPtr& propagationResult) const override {

        // std::cout << "Running getReward() in HMIRewardPlugin..." << std::endl;

        // Extract data from propagated state.
        VectorFloat previousStateVector = propagationResult->previousState->as<VectorState>()->asVector();
        VectorFloat actionVec = propagationResult->action->as<VectorAction>()->asVector();

        FloatType reward = 0.0;

        for (size_t i = 0; i != actionVec.size(); i += 2) {
            hmi::Coordinate start = hmi::Coordinate(previousStateVector[i], previousStateVector[i + 1]);
            hmi::Coordinate action = hmi::Coordinate(actionVec[i], actionVec[i + 1]);
            if (!grid_.getGrid()[action.toPosition(grid_)]) return illegalMovePenalty;
            bool helpingAgent = false;
            FloatType helpCond = -1.0;
            for (size_t j = actionVec.size(); j != previousStateVector.size(); j += 3) {
                if (previousStateVector[j] == action.getX() && previousStateVector[j + 1] == action.getY()) {
                    helpingAgent = true;
                    helpCond = std::max(helpCond, previousStateVector[j + 2]);
                }
            }
            FloatType stepPenalty = shortestPaths_.getPath(start.toPosition(grid_), action.toPosition(grid_)).size();
            if (helpingAgent && (int) helpCond > 0) reward += helpReward;
            reward -= stepPenalty;
        }
        return reward;
    }

    virtual std::pair<FloatType, FloatType> getMinMaxReward() const override {
        // std::cout << "Running and completing method getMinMaxReward() in class HMIRewardPlugin...\n";
        FloatType numRobots = robotEnvironment_->getRobot()->getActionSpace()->getNumDimensions() / 2.0;
        return std::make_pair(illegalMovePenalty, helpReward * numRobots);
    }


private:

    hmi::Grid grid_;
    hmi::ShortestPaths shortestPaths_;
    FloatType helpReward;
    FloatType illegalMovePenalty;

};

OPPT_REGISTER_REWARD_PLUGIN(HMIRewardPlugin)

}

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
        VectorFloat previousStateVector = propagationResult->previousState->as<VectorState>()->asVector();
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

        std::cout << "Setting happiness variables..." << std::endl;
        int numHappy = 0;
        bool allHappy = true;
        VectorInt distances(currentState.getRandomAgents().size());

        std::cout << "Checking happiness of each random agent..." << std::endl;
        for (size_t i = 0; i != currentState.getRandomAgents().size(); ++i) {
            hmi::HMIRandomAgent randomAgent = currentState.getRandomAgents()[i];
            if (randomAgent.getCondition() == 0) ++numHappy;
            else {
                allHappy = false;
                int randomX = randomAgent.getCoords().getX();
                int randomY = randomAgent.getCoords().getY();
                distances[i] = std::max(2, hmi::getShortestPath(grid_, actionVec[0], actionVec[1], randomX, randomY).first);
                for (size_t j = 2; j != actionVec.size(); j += 2) {
                    int dist = std::max(2, hmi::getShortestPath(grid_, actionVec[j], actionVec[j+1], randomX, randomY).first);
                    distances[i] = std::min(distances[i], dist);
                }
            }
        }

        if (allHappy) {
            std::cout << "Completed method getReward() in class HMIRewardPlugin...\n";
            return hmi::MAX_REWARD;
        }

        for (size_t i = 0; i != currentState.getRandomAgents().size(); ++i) {
            if (currentState.getRandomAgents()[i].getCondition() != 0)  {
                reward += numHappy * hmi::BASE_REWARD / distances[i];
            }
            else {
                reward += numHappy * hmi::BASE_REWARD;
            }
        }
        
        std::cout << "Completed method getReward() in class HMIRewardPlugin...\n";
        return reward;
    }

    virtual std::pair<FloatType, FloatType> getMinMaxReward() const override {
        std::cout << "Running and completing method getMinMaxReward() in class HMIRewardPlugin...\n";
        return std::make_pair(hmi::MIN_REWARD, hmi::MAX_REWARD);
    }


private:

    hmi::Grid grid_;
    std::vector<hmi::TypeAndId> randomAgents_;

};

OPPT_REGISTER_REWARD_PLUGIN(HMIRewardPlugin)

}

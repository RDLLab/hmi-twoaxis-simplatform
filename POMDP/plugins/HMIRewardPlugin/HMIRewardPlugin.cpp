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

        std::cout << "Getting action coordinates..." << std::endl;
        int actionX = (int) actionVec[0];
        int actionY = (int) actionVec[1];

        std::cout << "Setting invalid grid cell conditions..." << std::endl;
        bool xOutOfGrid = actionX < 0 || actionX >= currentState.getGrid().getWidth();
        bool yOutOfGrid = actionY < 0 || actionY >= currentState.getGrid().getHeight();
        bool invalidCell = !currentState.getGrid().getGrid()[hmi::Coordinate(actionX, actionY).toPosition(currentState.getGrid())];

        // Get robot coordinates.
        //int robotX = currentState.getRobotCoordinates().first;
        //int robotY = currentState.getRobotCoordinates().second;

        //bool xOutOfGrid = robotX < 0 || robotX >= currentState.getGrid().width_;
        //bool yOutOfGrid = robotY < 0 || robotY >= currentState.getGrid().height_;
        //bool invalidCell = !currentState.getGrid().grid_[robotX][robotY];

        if (xOutOfGrid || yOutOfGrid || invalidCell) {
            std::cout << "Action was " << std::to_string(actionVec[0]) << " " << std::to_string(actionVec[1]) << std::endl;
            std::cout << "Returned min reward!\n";
            return hmi::MIN_REWARD;
        }

        std::cout << "Setting happiness variables..." << std::endl;
        int numHappy = 0;
        bool allHappy = true;

        std::cout << "Checking happiness of each random agent..." << std::endl;
        for (hmi::HMIRandomAgent randomAgent : currentState.getRandomAgents()) {
            if (randomAgent.getCondition() == 0) ++numHappy;
            else                                 allHappy = false;
        }

        if (allHappy) {
            std::cout << "Completed method getReward() in class HMIRewardPlugin...\n";
            return hmi::MAX_REWARD;
        }

        for (hmi::HMIRandomAgent randomAgent : currentState.getRandomAgents()) {
            if (randomAgent.getCondition() != 0) {
                int randomX = randomAgent.getCoords().getX();
                int randomY = randomAgent.getCoords().getY();
                std::cout << "Calculating reward for actionX = " + std::to_string(actionX) + ", actionY = " + std::to_string(actionY) + ", randomX = " + std::to_string(randomX) + ", randomY = " + std::to_string(randomY) << std::endl;
                std::cout << "Distance between two agents is " << std::to_string(hmi::getShortestPath(grid_, actionX, actionY, randomX, randomY).first) << std::endl;
                reward += numHappy * hmi::BASE_REWARD / std::max(2, hmi::getShortestPath(grid_, actionX, actionY, randomX, randomY).first);
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

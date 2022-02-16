#include "HMIState.hpp"

namespace oppt
{
namespace hmi
{
    
HMIState::HMIState(VectorFloat stateVec, std::vector<TypeAndId> typesAndIDs, std::unordered_map<std::string, TransitionMatrix> typesToMatrices, Grid grid) {

    std::cout << "Running constructor of HMIState..." << std::endl;
    
    VectorInt state(stateVec.begin(), stateVec.end());

    // Initialise robot coordinates and grid structure.
    grid_ = grid;

    size_t numRobots = 2 * (state.size() - typesAndIDs.size());

    for (size_t i = 0; i != numRobots; i += ROBOT_ELEMENTS) {
        HMIRobot hmiRobot(state[i], state[i + 1]);
        robots_.push_back(hmiRobot);
    }

    for (size_t i = numRobots; i != state.size(); i += RANDOM_AGENT_ELEMENTS) {

        // Store the x- and y-coordinates and condition of this random agent into variables.
        int x = state[i];
        int y = state[i + 1];
        int condition = state[i + 2];

        // Determine where this random agent sits among the order of all the random agents.
        int randomAgentIndex = (i - numRobots) / RANDOM_AGENT_ELEMENTS;

        // Store the type and ID of this random agent into variables.
        std::pair<std::string, int> typeAndID = typesAndIDs[randomAgentIndex];
        std::string type = typeAndID.first;
        int id = typeAndID.second;

        // Determine the transition matrix that corresponds to this random agent's type.
        TransitionMatrix tm = typesToMatrices.at(type);

        // Initialise the random agent with these values and append it to this state's
        // vector of random agents.
        HMIRandomAgent randomAgent(x, y, type, id, condition, tm);
        randomAgents_.push_back(randomAgent);
    }

    std::cout << "Completed constructor of HMIState..." << std::endl;

}

HMIState::HMIState(VectorFloat stateVec, std::vector<TypeAndId> typesAndIDs, Grid grid) {

    std::cout << "Running constructor of HMIState..." << std::endl;
    
    VectorInt state(stateVec.begin(), stateVec.end());
    grid_ = grid;
    size_t numRobots = 2 * (state.size() - typesAndIDs.size());

    for (size_t i = 0; i != numRobots; i += ROBOT_ELEMENTS) {
        HMIRobot hmiRobot(state[i], state[i + 1]);
        robots_.push_back(hmiRobot);
    }

    for (size_t i = numRobots; i != state.size(); i += RANDOM_AGENT_ELEMENTS) {

        // Store the x- and y-coordinates and condition of this random agent into variables.
        int x = state[i];
        int y = state[i + 1];
        int condition = state[i + 2];

        // Determine where this random agent sits among the order of all the random agents.
        int randomAgentIndex = (i - numRobots) / RANDOM_AGENT_ELEMENTS;

        // Store the type and ID of this random agent into variables.
        std::pair<std::string, int> typeAndID = typesAndIDs[randomAgentIndex];
        std::string type = typeAndID.first;
        int id = typeAndID.second;

        // Initialise the random agent with these values and append it to this state's
        // vector of random agents.
        HMIRandomAgent randomAgent(x, y, type, id, condition);
        randomAgents_.push_back(randomAgent);
    }

    std::cout << "Completed constructor of HMIState..." << std::endl;
}

HMIState::HMIState(VectorFloat stateVec, Grid grid) {

    std::cout << "Running constructor of HMIState..." << std::endl;

    VectorInt state(stateVec.begin(), stateVec.end());
    grid_ = grid;
    size_t numRobots = 2 * (state.size() - typesAndIDs.size());

    for (size_t i = 0; i != numRobots; i += ROBOT_ELEMENTS) {
        HMIRobot hmiRobot(state[i], state[i + 1]);
        robots_.push_back(hmiRobot);
    }

    for (size_t i = numRobots; i != state.size(); i += RANDOM_AGENT_ELEMENTS) {

        // Store the x- and y-coordinates and condition of this random agent into variables.
        int x = state[i];
        int y = state[i + 1];
        int condition = state[i + 2];

        HMIRandomAgent randomAgent(x, y, condition);
        randomAgents_.push_back(randomAgent);
    }

    std::cout << "Completed constructor of HMIState..." << std::endl;
}

std::vector<HMIRobot> HMIState::getRobots() {
    return robots_;
}

Grid HMIState::getGrid() {
    return grid_;
}

std::vector<HMIRandomAgent> HMIState::getRandomAgents() {
    return randomAgents_;
}

void HMIState::sampleMovement(int numberOfTurns, std::set<HMIRandomAgent*> targetAgents) {
    std::cout << "Running method sampleMovement() of HMIState..." << std::endl;
    for (int t = 0; t < numberOfTurns; ++t) {
        for (HMIRandomAgent randAg : getRandomAgents()) {
            if (targetAgents.empty())
                randAg.sampleMovement(grid_);
            else {
                std::set<HMIRandomAgent*>::iterator randagIterator;
                for (randagIterator = targetAgents.begin(); randagIterator != targetAgents.end(); ++randagIterator) {
                    if (*randagIterator == &randAg) {
                        randAg.sampleMovement(grid_);
                    }
                }
            }
        }
    }
    std::cout << "Running method sampleMovement() of HMIState..." << std::endl;
}

VectorInt HMIState::toVector() {
    std::cout << "Running method toVector() of HMIState..." << std::endl;

    // Initialise the resulting vector of integers.
    int numRobots = ROBOT_ELEMENTS * robots_.size();
    int numRandomAgents = RANDOM_AGENT_ELEMENTS * randomAgents_.size();
    VectorInt res(numRobots + numRandomAgents);

    for (size_t i = 0; i != numRobots; i += ROBOT_ELEMENTS) {
        res[i] = robots_[i / ROBOT_ELEMENTS].getCoordinates().getX();
        res[i + 1] = robots_[i / ROBOT_ELEMENTS].getCoordinates().getY();
    }

    for (size_t i = numRobots; i != numRobots + numRandomAgents; i += RANDOM_AGENT_ELEMENTS) {
        int idx = (i - numRobots) / RANDOM_AGENT_ELEMENTS;
        res[i] = randomAgents_[idx].getCoords().getX();
        res[i + 1] = randomAgents_[idx].getCoords().getY();
        res[i + 2] = randomAgents_[idx].getCondition();
    }

    std::cout << "Completed method toVector() of HMIState..." << std::endl;

    // Return the resulting vector, filled with all of the necessary information.
    return res;
}


}
}
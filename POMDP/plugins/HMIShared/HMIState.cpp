#include "HMIState.hpp"

namespace oppt
{
namespace hmi
{
    
HMIState::HMIState(VectorFloat stateVec, std::vector<TypeAndId> typesAndIDs, std::unordered_map<std::string, TransitionMatrix> typesToMatrices, Grid grid) {

    std::cout << "Running constructor of HMIState..." << std::endl;
    
    VectorInt state(stateVec.begin(), stateVec.end());

    // Initialise robot coordinates and grid structure.
    robotX_ = state[DEFAULT_X_INDEX];
    robotY_ = state[DEFAULT_Y_INDEX];
    grid_ = grid;

    for (size_t i = DEFAULT_RANDOM_AGENT_START; i != state.size(); i += RANDOM_AGENT_ELEMENTS) {

        // Store the x- and y-coordinates and condition of this random agent into variables.
        int x = state[i];
        int y = state[i + 1];
        int condition = state[i + 2];

        // Determine where this random agent sits among the order of all the random agents.
        int randomAgentIndex = (i - DEFAULT_RANDOM_AGENT_START) / RANDOM_AGENT_ELEMENTS;

        // Store the type and ID of this random agent into variables.
        std::pair<std::string, int> typeAndID = typesAndIDs[randomAgentIndex];
        std::string type = typeAndID.first;
        int id = typeAndID.second;

        // Determine the transition matrix that corresponds to this random agent's type.
        TransitionMatrix transitionMatrix = typesToMatrices.at(type);

        // Initialise the random agent with these values and append it to this state's
        // vector of random agents.
        HMIRandomAgent randomAgent(x, y, type, id, condition, transitionMatrix);
        randomAgents_.push_back(randomAgent);
    }

    std::cout << "Completed constructor of HMIState..." << std::endl;

}

HMIState::HMIState(VectorFloat stateVec, std::vector<TypeAndId> typesAndIDs, Grid grid) {

    std::cout << "Running constructor of HMIState..." << std::endl;
    
    VectorInt state(stateVec.begin(), stateVec.end());
    
    // Initialise robot coordinates and grid structure.
    robotX_ = state[DEFAULT_X_INDEX];
    robotY_ = state[DEFAULT_Y_INDEX];
    grid_ = grid;

    for (size_t i = DEFAULT_RANDOM_AGENT_START; i != state.size(); i += RANDOM_AGENT_ELEMENTS) {

        // Store the x- and y-coordinates and condition of this random agent into variables.
        int x = state[i];
        int y = state[i + 1];
        int condition = state[i + 2];

        // Determine where this random agent sits among the order of all the random agents.
        int randomAgentIndex = (i - DEFAULT_RANDOM_AGENT_START) / RANDOM_AGENT_ELEMENTS;

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

    robotX_ = state[DEFAULT_X_INDEX];
    robotY_ = state[DEFAULT_Y_INDEX];
    grid_ = grid;

    for (size_t i = DEFAULT_RANDOM_AGENT_START; i != state.size(); i += RANDOM_AGENT_ELEMENTS) {

        // Store the x- and y-coordinates and condition of this random agent into variables.
        int x = state[i];
        int y = state[i + 1];
        int condition = state[i + 2];

        HMIRandomAgent randomAgent(x, y, condition);
        randomAgents_.push_back(randomAgent);
    }

    std::cout << "Completed constructor of HMIState..." << std::endl;
}

Coordinate HMIState::getRobotCoordinates() {
    return Coordinate(robotX_, robotY_);
}

int HMIState::getRobotX() {
    return robotX_;
}

int HMIState::getRobotY() {
    return robotY_;
}

void HMIState::setRobotX(int x) {
    robotX_ = x;
}

void HMIState::setRobotY(int y) {
    robotY_ = y;
}

Grid HMIState::getGrid() {
    return grid_;
}

std::vector<HMIRandomAgent> HMIState::getRandomAgents() {
    return randomAgents_;
}

void HMIState::sampleMovement(int numberOfTurns, HMIRandomAgent *targetAgent) {
    std::cout << "Running method sampleMovement() of HMIState..." << std::endl;
    for (int t = 0; t < numberOfTurns; ++t) {
        for (HMIRandomAgent randAg : getRandomAgents()) {
            if (targetAgent == nullptr || targetAgent != &randAg)
                randAg.sampleMovement(grid_);
        }
    }
    std::cout << "Running method sampleMovement() of HMIState..." << std::endl;
}

VectorInt HMIState::toVector() {
    std::cout << "Running method toVector() of HMIState..." << std::endl;

    // Initialise the resulting vector of integers.
    VectorInt res(DEFAULT_RANDOM_AGENT_START + (getRandomAgents().size() * RANDOM_AGENT_ELEMENTS));

    // Put the x- and y-coordinates of the robot into the first two indices of this vector.
    res[0] = robotX_;
    res[1] = robotY_;

    // Initialise the index variable that places information into the resulting vector.
    size_t idx = DEFAULT_RANDOM_AGENT_START;

    for (HMIRandomAgent randAg : getRandomAgents()) {

        // Add the random agent's x- and y-coordinates, as well as its condition.
        res[idx] = randAg.getX();
        res[idx + 1] = randAg.getY();
        res[idx + 2] = randAg.getCondition();

        // Increase the index variable by the number of variable elements for a random agent.
        idx += RANDOM_AGENT_ELEMENTS;
    }
    std::cout << "Completed method toVector() of HMIState..." << std::endl;

    // Return the resulting vector, filled with all of the necessary information.
    return res;
}

}
}
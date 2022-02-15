#include "HMIObservation.hpp"

namespace oppt
{

namespace hmi
{

HMIObservation::HMIObservation(HMIState &currentState) {
    underlyingState_ = &currentState;
    for (HMIRandomAgent randomAgent : currentState.getRandomAgents()) {
        observations_.insert(std::make_pair(&randomAgent, false));
        originalConditions_.push_back(randomAgent.getCondition());
    }
}

HMIObservation::HMIObservation(HMIState &currentState, HMIRandomAgent* targetAgent) {
    underlyingState_ = &currentState;
    for (HMIRandomAgent randomAgent : currentState.getRandomAgents()) {
        if (targetAgent && targetAgent == &randomAgent) {
            observations_.insert(std::make_pair(&randomAgent, true));
            randomAgent.setCondition(0);
        }
        else {
            observations_.insert(std::make_pair(&randomAgent, false));
        }
        originalConditions_.push_back(randomAgent.getCondition());
    }
}

HMIState HMIObservation::getUnderlyingState() {
    return *underlyingState_;
}

std::unordered_map<HMIRandomAgent*, bool> HMIObservation::getObservations() {
    return observations_;
}

void HMIObservation::makeObservations() {

    // Store the underlying state's robot coordinates.
    Coordinate robotCoords = getUnderlyingState().getRobotCoordinates();

    for (HMIRandomAgent randomAgent : getUnderlyingState().getRandomAgents()) {

        // If the agent is calling for help or the robot can see its condition, we add
        // its condition being observed to the observation vector.
        Coordinate randomAgentCoords = randomAgent.getCoords();
        if (testConditionProbability())
            observations_.at(&randomAgent) = true;
    }
}

VectorInt HMIObservation::toVector() {
    size_t numRandomAgents = getUnderlyingState().getRandomAgents().size();
    VectorInt res(numRandomAgents);
    for (size_t i = 0; i < numRandomAgents; ++i) {
        HMIRandomAgent randomAgent = getUnderlyingState().getRandomAgents()[i];
        if (observations_.at(&randomAgent)) res[i] = randomAgent.getCondition();
        else                                res[i] = originalConditions_[i];
    }
    return res;
}

bool HMIObservation::canSeeCondition(Coordinate start, Coordinate dest) {

    // Instantiate necessary elements to create a random float in the range [0.0, 1.0].
    RandomEngine randEng;
    std::uniform_real_distribution<float> sightDistribution(0.0, 1.0);

    // Determine the distance between the two given coordinates.
    int diffX = dest.getX() - start.getX();
    int diffY = dest.getY() - start.getY();

    // If the start and end coordinate are the same, the robot trivially has the potential
    // to see the agent's condition.
    if (diffX == 0 && diffY == 0) return sightDistribution(randEng) <= SEE_CONDITION_PROBABILITY;

    // Determine the longest distance. If the x-distance is larger, the "line of sight" will 
    // move by a magnitude of 1 along the x-axis at each iteration of the below loop, and vice 
    // versa if the y-distance is larger. The smaller distance will move by a magnitude of less 
    // than 1 along its axis at each iteration of the below loop.
    int longestDistance = std::max(abs(diffX), abs(diffY));

    // Set pointers to the line of sight in both coordinates.
    double x = (double) start.getX();
    double y = (double) start.getY();

    for (int i = 0; i < longestDistance; ++i) {

        // Move both line of sight pointers towards the destination.
        x += ((double) diffX) / ((double) longestDistance);
        y += ((double) diffY) / ((double) longestDistance);
        Coordinate coordinate(x, y);

        // Determine conditions where line of sight might be broken.
        bool xOutOfGrid = x < 0 || x >= getUnderlyingState().getGrid().getWidth();
        bool yOutOfGrid = y < 0 || y >= getUnderlyingState().getGrid().getHeight();
        bool invalidCell = !getUnderlyingState().getGrid().getGrid()[coordinate.toPosition(getUnderlyingState().getGrid())];

        // If the line of sight is broken, the robot did not see the random agent.
        if (xOutOfGrid || yOutOfGrid || invalidCell) return false;
    }

    // The robot can see the random agent, so it has a `SEE_CONDITION_PROBABILITY` chance
    // of seeing the random agent's condition.
    return sightDistribution(randEng) <= SEE_CONDITION_PROBABILITY;
}

bool HMIObservation::testConditionProbability() {
    RandomEngine randEng;
    std::uniform_real_distribution<float> sightDistribution(0.0, 1.0);
    return sightDistribution(randEng) <= 0.8;
}

}
}
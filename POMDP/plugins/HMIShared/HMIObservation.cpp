#include "HMIObservation.hpp"

namespace oppt
{

namespace hmi
{

HMIObservation::HMIObservation(HMIState &currentState) {
    underlyingState_ = &currentState;
    for (HMIRandomAgent randomAgent : currentState.getRandomAgents()) {
        observations_.insert(std::make_pair(&randomAgent, -1));
        originalConditions_.push_back(randomAgent.getCondition());
    }
}

HMIState HMIObservation::getUnderlyingState() {
    return *underlyingState_;
}

std::unordered_map<HMIRandomAgent*, int> HMIObservation::getObservations() {
    return observations_;
}

void HMIObservation::makeObservations() {

    for (HMIRandomAgent randomAgent : getUnderlyingState().getRandomAgents()) {

        // If the agent is calling for help or the robot can see its condition, we add
        // its condition being observed to the observation vector.
        if (testConditionProbability())
            observations_.at(&randomAgent) = randomAgent.getCondition();
    }
}

VectorInt HMIObservation::toVector() {
    size_t numRandomAgents = getUnderlyingState().getRandomAgents().size();
    VectorInt res(numRandomAgents);
    for (size_t i = 0; i < numRandomAgents; ++i) {
        res[i] = observations_.at(&getUnderlyingState().getRandomAgents()[i]);
    }
    return res;
}

VectorInt HMIObservation::toStateVector() {
    VectorInt res(underlyingState_->toVector());
    size_t numRobots = underlyingState_->getRobots().size();
    for (size_t i = numRobots + 2; i != res.size(); i += 3) {
        HMIRandomAgent* randomAgent = &underlyingState_->getRandomAgents()[i];
        res[i] = observations_.at(randomAgent) == -1 ? originalConditions_[(i - numRobots) / 3] : observations_.at(randomAgent);
    }
    return res;
}

void HMIObservation::sampleMovement(int numTurns, std::vector<std::string> robotMoves, std::set<HMIRandomAgent*> targetAgents) {
    for (size_t i = 0; i != numTurns; ++i) {
        for (size_t j = 0; j != underlyingState_->getRobots().size(); ++j) {
            if (!robotMoves[j].empty()) {
                hmi::HMIRobot robot = underlyingState_->getRobots()[j];
                hmi::Coordinate robotCoords = robot.getCoordinates();
                std::string path = robotMoves[j];
                if (path.at(0) == 'N')      robot.setCoordinates(hmi::Coordinate(robotCoords.getX(), robotCoords.getY() - 1));
                else if (path.at(0) == 'S') robot.setCoordinates(hmi::Coordinate(robotCoords.getX(), robotCoords.getY() + 1));
                else if (path.at(0) == 'E') robot.setCoordinates(hmi::Coordinate(robotCoords.getX() + 1, robotCoords.getY()));
                else                        robot.setCoordinates(hmi::Coordinate(robotCoords.getX() - 1, robotCoords.getY()));
                robotMoves[j] = robotMoves[j].substr(1);
                for (hmi::HMIRandomAgent randomAgent : underlyingState_->getRandomAgents()) {
                    if (randomAgent.getCoords() == robotCoords) {
                        randomAgent.setCondition(0);
                        observations_.at(&randomAgent) = 0;
                    }
                }   
            }
        }
        underlyingState_->sampleMovement(1, targetAgents);
        makeObservations();
    }
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
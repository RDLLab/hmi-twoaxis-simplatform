#include "HMIObservation.hpp"

namespace oppt
{

namespace hmi
{

HMIObservation::HMIObservation(HMIState &currentState) {
    underlyingState_ = &currentState;
    for (HMIRandomAgent randomAgent : underlyingState_->getRandomAgents()) {
        observations_.insert(std::make_pair(randomAgent.getIdentifier(), -1));
        originalConditions_.push_back(randomAgent.getCondition());
    }
}

HMIState HMIObservation::getUnderlyingState() {
    return *underlyingState_;
}

std::unordered_map<std::string, int> HMIObservation::getObservations() {
    return observations_;
}

void HMIObservation::makeObservations() {

    // std::cout << "Running method makeObservations() in HMIObservation..." << std::endl;

    for (HMIRandomAgent randomAgent : underlyingState_->getRandomAgents()) {

        // If the agent is calling for help or the robot can see its condition, we add
        // its condition being observed to the observation vector.
        if (testConditionProbability())
            observations_.at(randomAgent.getIdentifier()) = randomAgent.getCondition();
    }
    // std::cout << "Completed method makeObservations() in HMIObservation..." << std::endl;
}

VectorInt HMIObservation::toVector() {
    // std::cout << "Running method toVector() in HMIObservation..." << std::endl;
    size_t numRandomAgents = getUnderlyingState().getRandomAgents().size();
    VectorInt res(numRandomAgents);
    for (size_t i = 0; i < numRandomAgents; ++i) {
        res[i] = observations_.at(getUnderlyingState().getRandomAgents()[i].getIdentifier());
    }
    // std::cout << "Completed method toVector() in HMIObservation..." << std::endl;
    return res;
}

VectorInt HMIObservation::toStateVector() {
    // std::cout << "Running method toStateVector() in HMIObservation..." << std::endl;
    VectorInt res(underlyingState_->toVector());
    size_t numRobots = underlyingState_->getRobots().size() * HMIState::ROBOT_ELEMENTS;
    size_t numRandags = underlyingState_->getRandomAgents().size();
    for (size_t i = 0; i != numRandags; ++i) {
        HMIRandomAgent randomAgent = underlyingState_->getRandomAgents()[i];
        int idx = HMIState::RANDOM_AGENT_ELEMENTS * i + numRobots;
        if (observations_.at(randomAgent.getIdentifier()) == -1) {
            res[idx + 2] = originalConditions_[i];
        }
        else {
            res[idx + 2] = observations_.at(randomAgent.getIdentifier());
        }
    }
    // std::cout << "Completed method toStateVector() in HMIObservation..." << std::endl;
    return res;
}

void HMIObservation::sampleMovement(int numTurns, std::vector<std::string> robotMoves, std::set<std::string> targetAgents) {
    // std::cout << "Running method sampleMovement() in HMIObservation..." << std::endl;
    for (size_t i = 0; i < numTurns; ++i) {
        for (size_t j = 0; j < underlyingState_->getRobots().size(); ++j) {
            hmi::HMIRobot robot = underlyingState_->getRobots()[j];
            hmi::Coordinate robotCoords = robot.getCoordinates();
            std::string path = robotMoves[j];
            if (!path.empty()) {
                if (path.at(0) == 'N')      robot.setCoordinates(hmi::Coordinate(robotCoords.getX(), robotCoords.getY() - 1));
                else if (path.at(0) == 'S') robot.setCoordinates(hmi::Coordinate(robotCoords.getX(), robotCoords.getY() + 1));
                else if (path.at(0) == 'E') robot.setCoordinates(hmi::Coordinate(robotCoords.getX() + 1, robotCoords.getY()));
                else                        robot.setCoordinates(hmi::Coordinate(robotCoords.getX() - 1, robotCoords.getY()));
                robotMoves[j] = robotMoves[j].substr(1);
                if (robotMoves[j].empty()) {
                    for (HMIRandomAgent agent : underlyingState_->getRandomAgents()) {
                        if (agent.getCoords().getX() == robot.getCoordinates().getX() &&
                              agent.getCoords().getY() == robot.getCoordinates().getY())
                            agent.setCondition(0);
                            observations_.at(agent.getIdentifier()) = 0;
                            if (targetAgents.find(agent.getIdentifier()) != targetAgents.end()) {
                                targetAgents.erase(agent.getIdentifier());
                            }
                    }
                }
            }
            // for (hmi::HMIRandomAgent randomAgent : underlyingState_->getRandomAgents()) {
            //     if (randomAgent.getCoords() == robotCoords) {
            //         randomAgent.setCondition(0);
            //         observations_.at(randomAgent.getIdentifier()) = 0;
            //     }
            // }   
        }
        underlyingState_->sampleMovement(1, targetAgents);
        makeObservations();
    }
    // std::cout << "Completed method sampleMovement() in HMIObservation..." << std::endl;
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
    // std::cout << "Running method testConditionProbability() in HMIObservation..." << std::endl;
    RandomEngine randEng;
    std::uniform_real_distribution<float> sightDistribution(0.0, 1.0);
    // std::cout << "Completed method testConditionProbability() in HMIObservation..." << std::endl;
    return sightDistribution(randEng) <= 0.8;
}

}
}
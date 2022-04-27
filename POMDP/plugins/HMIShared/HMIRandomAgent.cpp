#include "HMIRandomAgent.hpp"

namespace oppt
{
namespace hmi
{

int DEFAULT_CONDITION = 0;

HMIRandomAgent::HMIRandomAgent(int x, int y, std::string type, int id, int condition, TransitionMatrix tm) :
    coords_(Coordinate(x, y)), type_(type), id_(id), condition_(condition), transitionMatrix_(tm), leftAlone_(true) { }

HMIRandomAgent::HMIRandomAgent(int x, int y, std::string type, int id, TransitionMatrix tm) :
    coords_(Coordinate(x, y)), type_(type), id_(id), condition_(DEFAULT_CONDITION), transitionMatrix_(tm), leftAlone_(true) { }

HMIRandomAgent::HMIRandomAgent(int x, int y, std::string type, int id, int condition) :
    coords_(Coordinate(x, y)), type_(type), id_(id), condition_(condition), leftAlone_(true) { }

HMIRandomAgent::HMIRandomAgent(int x, int y, int condition) :
    coords_(Coordinate(x, y)), condition_(condition), leftAlone_(true) { }

int HMIRandomAgent::getX() {
    return coords_.getX();
}

void HMIRandomAgent::setX(int x) {
    coords_ = Coordinate(x, coords_.getY());
}

int HMIRandomAgent::getY() {
    return coords_.getY();
}

void HMIRandomAgent::setY(int y) {
    coords_ = Coordinate(coords_.getX(), y);
}

Coordinate HMIRandomAgent::getCoords() {
    return coords_;
}

void HMIRandomAgent::setCoords(Coordinate &coords) {
    coords_ = coords;
}

std::string HMIRandomAgent::getType() {
    return type_;
}

int HMIRandomAgent::getID() {
    return id_;
}

int HMIRandomAgent::getCondition() {
    return condition_;
}

void HMIRandomAgent::setCondition(int condition) {
    condition_ = condition;
}

std::string HMIRandomAgent::getIdentifier() {
    return getType() + std::to_string(getID());
}

TransitionMatrix HMIRandomAgent::getTransitionMatrix() {
    return transitionMatrix_;
}

bool HMIRandomAgent::isLeftAlone() {
    return leftAlone_;
}

void HMIRandomAgent::setLeftAlone(bool leftAlone) {
    leftAlone_ = leftAlone;
}

void HMIRandomAgent::sampleMovement(const Grid &grid) {

    // Initialise random engine to use for movement and condition transition.
    RandomEngine randEng;

    // Create a distribution to decide whether this random agent will move or not.
    std::uniform_int_distribution<> moveDistribution(0, 1);
    bool willMove = moveDistribution(randEng);

    if (condition_ == 0 && willMove) {

        // Create a vector storing all of the traversing neighbouring squares to
        // this random agent (diagonals allowed).
        std::vector<Coordinate> possibleMoves;

        // Store how many possible moves this random agent can make. Useful for
        // randomly choosing a move from all of the possible moves.
        int numberOfMoves = 0;

        for (Coordinate coord : DIRECTIONS) {
            int newX = getX() + coord.getX();
            int newY = getY() + coord.getY();
            Coordinate coordinate(newX, newY);

            bool xInBounds = newX > -1 && newX < grid.getWidth();
            bool yInBounds = newY > -1 && newY < grid.getHeight();
            bool validCell = grid.getGrid()[coordinate.toPosition(grid)];

            if (xInBounds && yInBounds && validCell) {
                Coordinate possibleMove(newX, newY);
                possibleMoves.push_back(possibleMove);
                ++numberOfMoves;
            }
        }

        // Randomly obtain a move from the pool of valid moves this agent can make.
        std::uniform_int_distribution<> newPositionDistribution(0, numberOfMoves);
        Coordinate newMove = possibleMoves[newPositionDistribution(randEng)];

        // Set this agent's x- and y-coordinates according to the move that was sampled.
        setX(newMove.getX());
        setY(newMove.getY());
    }

    // Create a distribution for making a random number to determine condition transition.
    std::uniform_real_distribution<float> transitionDistribution(0.0, 1.0);

    // Initialise this random float.
    float changeValue;

    do {

        // Ensure that this float does not equal 0.0, since that would result in the
        // agent instantly transitioning to condition 0 (whatever that condition is).
        changeValue = transitionDistribution(randEng);
    } while (changeValue == 0.0);

    // Set the condition to change to as 0, and subtract the probability of transitioning
    // from the agent's current condition to condition 0 from the given random float.
    int conditionToChangeTo = 0;
    changeValue -= getTransitionMatrix().matrix_[getCondition()][conditionToChangeTo];

    while (changeValue > 0.0) {

        // Move through successive conditions and subtract the probability of transitioning
        // from the agent's current condition to the given condition from the given random
        // float, until the sign of the given random float is not positive.
        ++conditionToChangeTo;
        changeValue -= getTransitionMatrix().matrix_[getCondition()][conditionToChangeTo];
    }

    // Given random float is no longer positive, so we stop and assign the condition that
    // we stopped at to be the agent's new condition.
    setCondition(conditionToChangeTo);
}

bool HMIRandomAgent::operator==(const HMIRandomAgent& other) {
    return this->getType() == other.type_ && this->getID() == other.id_;
}

}
}
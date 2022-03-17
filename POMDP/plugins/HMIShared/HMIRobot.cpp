#include "HMIRobot.hpp"

namespace oppt 
{
namespace hmi
{

HMIRobot::HMIRobot(int x, int y) : coordinates_(x, y) { }

Coordinate HMIRobot::getCoordinates() {
    return coordinates_;
}

void HMIRobot::setCoordinates(Coordinate coordinate) {
    coordinates_ = Coordinate(coordinate.getX(), coordinate.getY());
}

void HMIRobot::sampleMovement(Coordinate move) {
    setCoordinates(Coordinate(coordinates_.getX() + move.getX(), coordinates_.getY() + move.getY()));
}

void HMIRobot::makeMove(char direction) {
    if (direction == 'N')      sampleMovement(Coordinate(0, -1));
    else if (direction == 'E') sampleMovement(Coordinate(1, 0));
    else if (direction == 'S') sampleMovement(Coordinate(0, 1));
    else if (direction == 'W') sampleMovement(Coordinate(-1, 0));
}

}
}
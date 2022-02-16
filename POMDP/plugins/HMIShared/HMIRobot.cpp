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
    coordinates_(coordinate);
}

}
}
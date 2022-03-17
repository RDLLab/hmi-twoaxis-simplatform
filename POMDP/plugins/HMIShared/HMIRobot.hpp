#ifndef _HMI_ROBOT_HPP_
#define _HMI_ROBOT_HPP_

#include "HMIRandomAgent.hpp"

namespace oppt
{
namespace hmi
{

class HMIRobot {

public:
    
    HMIRobot(int x, int y);

    Coordinate getCoordinates();

    void setCoordinates(Coordinate coordinate);

    void sampleMovement(Coordinate move);

    void makeMove(char direction);

private:
    Coordinate coordinates_;

};

}
}

#endif
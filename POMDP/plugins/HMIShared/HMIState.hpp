#ifndef _HMI_STATE_HPP_
#define _HMI_STATE_HPP_

#include "HMIRobot.hpp"
#include "oppt/opptCore/core.hpp"
#include <vector>
#include <unordered_map>
#include <utility>
#include <set>
#include <iterator>

namespace oppt
{
namespace hmi
{

class HMIState
{
public:

    // The default index whose element represents the robot's x-coordinate.
    const int DEFAULT_X_INDEX = 0;

    // The default index whose element represents the robot's y-coordinate
    const int DEFAULT_Y_INDEX = 1;

    // The default index whose element represents the start of all information
    // relating to random agents.
    const int DEFAULT_RANDOM_AGENT_START = 2;

    // The number of variable fields belonging to each random agent.
    static const int RANDOM_AGENT_ELEMENTS = 3;

    static const int ROBOT_ELEMENTS = 2;

    /**
     * Constructor for HMIState. Takes a vector of integers and enriches
     * it into an easier-to-use data structure.
     * 
     * @param stateVec        the variable information needed to populate the state, that
     *                        is locations and conditions
     * 
     * @param typesAndIDs     the type and ID of every dependent agent, maintained in a
     *                        specific order
     * 
     * @param typesToMatrices a map linking different agent types to their corresponding
     *                        transition matrices
     * 
     * @param grid            the grid on which all agents move
    **/
    HMIState(VectorFloat stateVec, std::vector<TypeAndId> typesAndIDs, std::unordered_map<std::string, TransitionMatrix> typesToMatrices, Grid grid);

    HMIState(VectorFloat stateVec, std::vector<TypeAndId> typesAndIDs, Grid grid);

    HMIState(VectorFloat stateVec, Grid grid);

    std::vector<HMIRobot> getRobots();

    Grid getGrid();

    std::vector<HMIRandomAgent> getRandomAgents();
    
    /**
     * Samples movement for every random agent for a given number of turns.
     * 
     * @param numberOfTurns the number of turns for which to sample agent movement
    **/
    void sampleMovement(int numberOfTurns, std::set<HMIRandomAgent*> targetAgents);

    /**
     * Returns a vector of integers representing the variable elements of this
     * state.
     * 
     * @return a vector representing the variable elements of this state.
    **/
    VectorInt toVector();

private:

    // The robots in the state
    std::vector<HMIRobot> robots_;

    // All of the random agents in the given problem.
    std::vector<HMIRandomAgent> randomAgents_;

    // The grid on which all of the agents in this problem move.
    Grid grid_;
    
};

}
}

#endif
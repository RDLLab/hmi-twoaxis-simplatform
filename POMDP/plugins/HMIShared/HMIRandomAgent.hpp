#ifndef _HMI_RANDOM_AGENT_HPP_
#define _HMI_RANDOM_AGENT_HPP_

#include "HMIDataStructures.hpp"
#include <iostream>
#include <random>
#include <string>
#include <vector>
#include <utility>

namespace oppt
{
namespace hmi
{

class HMIRandomAgent {
public:

    /**
     * Constructor for HMIRandomAgent.
     * 
     * @param x          the starting x-coordinate of this agent
     * 
     * @param y          the starting y-coordinate of this agent
     * 
     * @param type       the type to which this agent belongs, eg. toddler, elderly person etc.
     * 
     * @param id         the numerical ID of this agent. Every random agent has a unique type
     *                   and ID combination.
     * 
     * @param condition  the starting condition of this agent
     * 
     * @param tm         the transition matrix according to which this agent behaves
    **/
    HMIRandomAgent(int x, int y, std::string type, int id, int condition, TransitionMatrix tm);
    
    /**
     * Constructor for HMIRandomAgent. The only difference between this constructor and the one
     * above is that this constructor assigns a default condition to the agent.
     * 
     * @param x          the starting x-coordinate of this agent
     * 
     * @param y          the starting y-coordinate of this agent
     * 
     * @param type       the type to which this agent belongs, eg. toddler, elderly person etc.
     * 
     * @param id         the numerical ID of this agent. Every random agent has a unique type
     *                   and ID combination.
     * 
     * @param tm         the transition matrix according to which this agent behaves
    **/
    HMIRandomAgent(int x, int y, std::string type, int id, TransitionMatrix tm);

    HMIRandomAgent(int x, int y, std::string type, int id, int condition);

    HMIRandomAgent(int x, int y, int condition);

    /**
     * Destructor for HMIRandomAgent.
    **/
    ~HMIRandomAgent() = default;

    int getX();

    void setX(int x);

    int getY();

    void setY(int y);

    Coordinate getCoords();

    void setCoords(Coordinate &coords);

    std::string getType();

    int getID();

    int getCondition();

    void setCondition(int condition);

    std::string getIdentifier();

    TransitionMatrix getTransitionMatrix();

    bool isLeftAlone();

    void setLeftAlone(bool leftAlone);

    /**
     * Samples a single move and condition change for this agent. This involves
     * using the grid to move to a suitable location and transitioning to a certain
     * condition based on this random agent's transition matrix.
     * 
     * @param grid the grid on which the agent will move
    **/
    void sampleMovement(const Grid &grid);

    bool operator==(const HMIRandomAgent& other);

private:

    // The agent's coordinates
    Coordinate coords_;

    // The type to which this agent belongs, eg. toddler, elderly person etc.
    std::string type_;

    // The agent's numerical ID. Each random agent has a unique type and ID
    // combination.
    int id_;

    // The agent's current condition.
    int condition_;

    // Whether the agent is currently receiving help or not.
    bool leftAlone_;

    // The transition matrix according to which the agent behaves.
    TransitionMatrix transitionMatrix_;
};

}
}

#endif
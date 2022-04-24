#ifndef _HMI_OBSERVATION_HPP_
#define _HMI_OBSERVATION_HPP_

#include "HMIState.hpp"
#include "oppt/opptCore/core.hpp"
#include <vector>
#include <utility>
#include <unordered_map>
#include <random>
#include <algorithm>
#include <set>
#include <string>

namespace oppt
{
namespace hmi
{
class HMIObservation 
{

public:

    HMIObservation(HMIState &currentState, int numConditions);

    HMIState getUnderlyingState();

    std::unordered_map<std::string, int> getObservations();

    void makeObservations();

    VectorInt toVector();

    VectorInt toStateVector();

    void sampleMovement(int numTurns, std::vector<std::string> robotMoves, std::set<std::string> targetAgents);

private:

    HMIState *underlyingState_;
    std::unordered_map<std::string, int> observations_;
    VectorInt originalConditions_;
    const double SEE_CONDITION_PROBABILITY = 0.8;

private:

    bool canSeeCondition(Coordinate start, Coordinate dest);

    bool testConditionProbability();

};
}
}

#endif
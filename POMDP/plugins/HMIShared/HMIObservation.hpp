#ifndef _HMI_OBSERVATION_HPP_
#define _HMI_OBSERVATION_HPP_

#include "HMIState.hpp"
#include "oppt/opptCore/core.hpp"
#include <vector>
#include <utility>
#include <unordered_map>
#include <random>
#include <algorithm>

namespace oppt
{
namespace hmi
{
class HMIObservation 
{

public:

    HMIObservation(HMIState &currentState);

    HMIObservation(HMIState &currentState, HMIRandomAgent* targetAgent);

    HMIState getUnderlyingState();

    std::unordered_map<HMIRandomAgent*, bool> getObservations();

    void makeObservations();

    VectorInt toVector();

private:

    HMIState *underlyingState_;
    std::unordered_map<HMIRandomAgent*, bool> observations_;
    VectorInt originalConditions_;
    const double SEE_CONDITION_PROBABILITY = 0.8;

private:

    bool canSeeCondition(Coordinate start, Coordinate dest);

    bool testConditionProbability();

};
}
}

#endif
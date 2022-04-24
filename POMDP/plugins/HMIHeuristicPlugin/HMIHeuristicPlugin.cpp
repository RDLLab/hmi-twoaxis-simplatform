#ifndef _HMI_HEURISTIC_PLUGIN_HPP_
#define _HMI_HEURISTIC_PLUGIN_HPP_
#include "oppt/plugin/Plugin.hpp"
#include "HMIHeuristicOptions.hpp"
#include "plugins/HMIShared/HMIObservation.hpp"
#include "plugins/HMIShared/ShortestPaths.hpp"

#include <string>
#include <array>
#include <memory>
#include <stdio.h>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <map>
#include <unordered_map>
#include <set>
#include <random>

namespace oppt
{
class HMIHeuristicPlugin: public HeuristicPlugin
{
public:

    HMIHeuristicPlugin(): HeuristicPlugin() { }

    virtual ~HMIHeuristicPlugin() {}

    virtual bool load(const std::string& optionsFile) override {
        // Set up the dependent agents as per robot environment.
        // std::cout << "Running method load() in class HMIHeuristicPlugin...\n";
        parseOptions_<HMIHeuristicOptions>(optionsFile);
        std::string gridPath
            = static_cast<HMIHeuristicOptions*>(options_.get())->gridPath;
        grid_ = hmi::instantiateGrid(gridPath);
        paths_ = hmi::ShortestPaths(grid_);
        std::string randomAgentsPath
            = static_cast<HMIHeuristicOptions*>(options_.get())->randomAgentsPath;
        randomAgents_ = hmi::instantiateTypesAndIDs(randomAgentsPath);
        std::string transitionMatricesPath
            = static_cast<HMIHeuristicOptions*>(options_.get())->transitionMatrixPath;
        transitionMatrices_ = hmi::instantiateTransitionMatrices(transitionMatricesPath);
        // std::cout << "Completed method load() in class HMIHeuristicPlugin...\n";
        return true;
    }

    virtual FloatType getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {
        // std::cout << "Running method getHeuristicValue() in class HMIHeuristicPlugin...\n";
        FloatType val = 0.9;
        // std::cout << "Completed method getHeuristicValue() in class HMIHeuristicPlugin...\n";
        return val;
    }

private:
    std::vector<hmi::TypeAndId> randomAgents_;
    hmi::Grid grid_;
    std::unordered_map<std::string, hmi::TransitionMatrix> transitionMatrices_;
    hmi::ShortestPaths paths_;

};

OPPT_REGISTER_HEURISTIC_PLUGIN(HMIHeuristicPlugin)

}

#endif

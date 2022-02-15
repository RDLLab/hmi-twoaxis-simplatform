#include "oppt/plugin/Plugin.hpp"
#include "HMITerminalOptions.hpp"
#include "plugins/HMIShared/HMIState.hpp"

#include <string>
#include <array>
#include <memory>
#include <stdio.h>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <map>
#include <unordered_map>

namespace oppt
{
class HMITerminalPlugin: public TerminalPlugin {
    public:

        HMITerminalPlugin(): TerminalPlugin() {}

        virtual ~HMITerminalPlugin() = default;

        virtual bool load(const std::string& optionsFile) override {
            // std::cout << "Running method load() in class HMITerminalPlugin...\n";
            parseOptions_<HMITerminalOptions>(optionsFile);
            std::string gridPath
                = static_cast<HMITerminalOptions*>(options_.get())->gridPath;
            grid_ = hmi::instantiateGrid(gridPath);
            // std::cout << "Completed method load() in class HMITerminalPlugin...\n";
            return true;
        }

        virtual ValidityReportSharedPtr isValid(const PropagationResultSharedPtr& propagationResult) override {
            // std::cout << "Running method isValid() in class HMITerminalPlugin...\n";
            ValidityReportSharedPtr vr(new ValidityReport(propagationResult->nextState));
            VectorFloat stateVec = propagationResult->nextState->as<VectorState>()->asVector();
            hmi::HMIState hmiState(stateVec, grid_);
            vr->isValid = true;

            bool xOutOfGrid = hmiState.getRobotX() < 0 || hmiState.getRobotX() >= grid_.getWidth();
            bool yOutOfGrid = hmiState.getRobotY() < 0 || hmiState.getRobotY() >= grid_.getHeight();
            bool invalidCell = !grid_.getGrid()[hmiState.getRobotCoordinates().toPosition(grid_)];
            if (xOutOfGrid || yOutOfGrid || invalidCell) {
                vr->isValid = false;
            }
            else {
                for (hmi::HMIRandomAgent randomAgent : hmiState.getRandomAgents()) {
                    xOutOfGrid = randomAgent.getX() < 0 || randomAgent.getX() >= grid_.getWidth();
                    yOutOfGrid = randomAgent.getY() < 0 || randomAgent.getY() >= grid_.getHeight();
                    invalidCell = !grid_.getGrid()[randomAgent.getCoords().toPosition(grid_)];
                    if (xOutOfGrid || yOutOfGrid || invalidCell) {
                        vr->isValid = false;
                        break;
                    }
                }
            }
            // std::cout << "Completed method isValid() in class HMITerminalPlugin...\n";
            return vr;
        }

        virtual bool isTerminal(const PropagationResultSharedPtr& propagationResult) override {
            // std::cout << "Running and completing method isTerminal() in class HMITerminalPlugin...\n";
            return false;
        }

    private:
        hmi::Grid grid_;

};

OPPT_REGISTER_TERMINAL_PLUGIN(HMITerminalPlugin)

}

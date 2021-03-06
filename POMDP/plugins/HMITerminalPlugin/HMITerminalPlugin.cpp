#include "oppt/plugin/Plugin.hpp"
#include "HMITerminalOptions.hpp"
#include "plugins/HMIShared/HMIDataStructures.hpp"

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
            parseOptions_<HMITerminalOptions>(optionsFile);
            std::string gridPath
                = static_cast<HMITerminalOptions*>(options_.get())->gridPath;
            grid_ = hmi::instantiateGrid(gridPath);

            std::string requestersPath
                = static_cast<HMITerminalOptions*>(options_.get())->requestersPath;
            requesters_ = hmi::instantiateTypesAndIDs(requestersPath);
            return true;
        }

        virtual ValidityReportSharedPtr isValid(const PropagationResultSharedPtr& propagationResult) override {
            ValidityReportSharedPtr vr(new ValidityReport(propagationResult->nextState));
            VectorFloat stateVec = propagationResult->nextState->as<VectorState>()->asVector();
            vr->isValid = true;

            VectorFloat actionVec = propagationResult->action->as<VectorAction>()->asVector();

            for (size_t i = 0; i != actionVec.size(); i += 2) {
                hmi::Coordinate pos(stateVec[i], stateVec[i + 1]);
                bool xOutOfGrid = pos.getX() < 0 || pos.getX() >= grid_.getWidth();
                bool yOutOfGrid = pos.getY() < 0 || pos.getY() >= grid_.getHeight();
                if (xOutOfGrid || yOutOfGrid) {
                    vr->isValid = false;
                    break;
                }
                bool isValidCell = grid_.getGrid()[pos.toPosition(grid_)];
                if (!isValidCell) {
                    vr->isValid = false;
                    break;
                }
            }

            if (vr->isValid) {
                for (size_t j = actionVec.size(); j != stateVec.size(); j += 3) {
                    hmi::Coordinate pos(stateVec[j], stateVec[j + 1]);
                    bool xOutOfGrid = pos.getX() < 0 || pos.getX() >= grid_.getWidth();
                    bool yOutOfGrid = pos.getY() < 0 || pos.getY() >= grid_.getHeight();
                    if (xOutOfGrid || yOutOfGrid) {
                        vr->isValid = false;
                        break;
                    }
                    bool isValidCell = grid_.getGrid()[pos.toPosition(grid_)];
                    if (!isValidCell) {
                        vr->isValid = false;
                        break;
                    }
                }
            }
            return vr;
        }

        virtual bool isTerminal(const PropagationResultSharedPtr& propagationResult) override {
            return false;
        }

    private:
        hmi::Grid grid_;
        std::vector<hmi::TypeAndId> requesters_;

};

OPPT_REGISTER_TERMINAL_PLUGIN(HMITerminalPlugin)

}

// #ifndef _HMI_SOLVER_HPP_
// #define _HMI_SOLVER_HPP_

// #include "oppt/solver/solver.hpp"
// #include "solvers/ABT/robotModel/changes/GenericModelChange.hpp"
// #include "solvers/ABT/robotModel/treeInspector/TreeInspector.hpp"
// #include "solvers/ABT/robotModel/abtSimulator/ABTSimulator.hpp"
// #include "solvers/ABT/robotModel/solver/Solver.hpp"
// #include "solvers/ABT/robotModel/RobotModel.hpp"
// #include "solvers/ABT/robotModel/solver/StatePool.hpp"
// #include "solvers/ABT/solver/Histories.hpp"
// #include "oppt/filter/particleFilter/ParticleFilter.hpp"

// #include "HMISolverOptions.hpp"
// #include "shared/structures/HMIObservation.hpp"

// using namespace oppt;

// namespace solvers
// {

// const bool SENDING = true;
// const bool RECEIVING = false;

// class HMISolver: public ABT
// {
// public:
//     HMISolver() : ABT(), grid_() {
//         this->solverName_ = "HMISolver";
//     }
    
//     virtual ~HMISolver() {
//         if (prevStream_)
//             delete prevStream_;
//     }

//     virtual void setup() override {
//         if (static_cast<const HMISolverOptions*>(problemEnvironmentOptions_)->savePolicy)
//             static_cast<HMISolverOptions*>(problemEnvironmentOptions_)->pruneEveryStep = false;
//         instantiatePipes();
//         grid_ = hmi::instantiateGrid(static_cast<HMISolverOptions*>(problemEnvironmentOptions_)->gridPath);

//         registerForEnvironmentChanges(robotPlanningEnvironment_);

//         particleFilter_ = std::make_unique<ParticleFilter>();
//     }

//     virtual bool reset() override {
//         HMISolverOptions* options = static_cast<HMISolverOptions*>(problemEnvironmentOptions_);
//         this->randGen_->discard(10000);
//         if (!options->baseConfigPath.empty()) {
//             auto baseConfigPath = options->baseConfigPath;
//             oppt::change_directory(baseConfigPath);
//         }

//         if (!solverModel_) {
//             solverModel_ =
//                 std::make_unique<robot::RobotModel>(this->randGen_, std::make_unique<HMISolverOptions*>(options));
//         }
//         solverModel_->setRobotEnvironment(robotPlanningEnvironment_);
//         if (heuristicPlugin_) {
//             solverModel_->setHeuristicPlugin(heuristicPlugin_.get());
//         }

//         solver_ = std::make_unique<ABTSolver>(std::move(solverModel_));

//         // NB This is taken directly from the ABT Solver source code, and includes functionalities that I don't
//         // know are necessary in our solver. The `hasChanges_` and `interactive_` fields are protected in
//         // the ABT Solver source. Leaving them as their default "false" for now, unless we need to add them in
//         // later.

//         // solver_->hasChanges_ = static_cast<HMISolverOptions const*>(problemEnvironmentOptions_)->hasChanges;
//         // solver_->interactive_ = static_cast<HMISolverOptions const*>(problemEnvironmentOptions_)->interactive;

//         if (options->loadInitialPolicy) {
//             std::ifstream inFile;
//             inFile.open(options->policyPath);
//             if (!inFile.is_open()) {
//                 std::ostringstream message;
//                 message << "Failed to open " << options->policyPath;
//                 debug::show_message(message.str());
//                 return 1;
//             }
//             solver_->getSerializer()->load(inFile);
//             inFile.close();
//         }
//         else {
//             solver_->initializeEmpty();
//         }

//         simulatorModel_ = 
//             std::make_unique<robot::RobotModel>(this->randGen_, std::make_unique<HMISolverOptions*>(*options));
//         simulatorModel_->setRobotEnvironment(robotPlanningEnvironment_);
//         if (heuristicPlugin_)
//             simulatorModel_->setHeuristicPlugin(heuristicPlugin_.get());
//         simulator_ = std::make_unique<abt::ABTSimulator>(std::move(simulatorModel_), solver_.get(), options->areDynamic);

//         if (!options->baseConfigPath.empty()) {
//             auto baseConfigPath = options->baseConfigPath;
//             oppt::change_directory(baseConfigPath);
//         }

//         simulator_->setMaxStepCount(options->nSimulationSteps);
//         return true;
//     }

//     virtual void handleEnvironmentChanges(const std::vector<EnvironmentChangeSharedPtr>& environmentChanges) override {
//         // FIXME Complete this method

//         if (simulator_) {
//             // Set the change sequence
//             abt::ChangeSequence changeSequence = getChangeSequence(environmentChanges);
//             abt::ChangeSequence::iterator it = changeSequence.find(simulator_->getStepCount());
//             if (it != changeSequence.end()) {
//                 if (problemEnvironmentOptions_->hasVerboseOutput) {
//                     cout << "Model changing." << endl;
//                 }
//                 FloatType changingTimeStart = oppt::clock_ms();
//                 // Apply all the changes!
//                 bool noError =
//                     simulator_->handleChanges(iter->second,
//                                               problemEnvironmentOptions_->areDynamic,
//                                               static_cast<HMISolverOptions*>(problemEnvironmentOptions_)->resetOnChanges);
//                 // Update the BeliefNode * in case there was a tree reset
//                 auto currentBelief = simulator_->getAgent()->getCurrentBelief();
//                 if (!noError) return;
                
//                 FloatType changingTime = oppt::clock_ms() - changingTimeStart;
//                 if (problemEnvironmentOptions_->hasVerboseOutput) {
//                     cout << "Changes complete" << endl;
//                     cout << "Total of " << changingTime << " ms used for changes." << endl;
//                 }
//             }
//         }
//     }

//     virtual bool improvePolicy(const FloatType& timeout) override {
//         return ABT::improvePolicy(timeout, -1);
//     }

//     // bool improvePolicy(const FloatType& timeout, const int& numHistories) {
//     //     auto currentBelief = simulator_->getAgent()->getCurrentBelief();
//     //     if (!currentBelief) {
//     //         WARNING("Current belief is null");
//     //         return false;
//     //     }
//     //     if (currentBelief == simulator_->getSolver()->getPolicy()->getRoot()) {
//     //         numHistories_ = simulator_->getSolver()->improvePolicy(nullptr, numHistories, -1, timeout);
//     //     } else {
//     //         numHistories_ = simulator_->getSolver()->improvePolicy(currentBelief, numHistories, -1, timeout);
//     //     }

//     //     totalNumHistories_ += numHistories_;
//     //     if (numHistories_ == 0)
//     //         return false;
//     //     return true;
//     // }

//     virtual ActionSharedPtr getNextAction() override {
//         // FIXME Verify this method with Hanna to confirm it makes sense
//         auto model = static_cast<robot::RobotModel*>(simulator_->getModel());
//         auto preferredAction = simulator_->getAgent()->getPreferredAction().get();
//         if (!preferredAction) {
//             ERROR("Preferred action is null");
//         }
//         return model->getBaseAction(*(preferredAction));
//     }

//     virtual bool updateBelief(const ActionSharedPtr& action,
//                               const ObservationSharedPtr& observation,
//                               const bool &allowTerminalStates = false) override {
//         abt::BeliefNode *currentBelief = simulator_->getAgent()->getCurrentBelief();
//         auto robotAction =
//             static_cast<robot::RobotModel*>(simulator_->getModel())->makeRobotAction(action);
//         shared::RobotObservation robotObservation(observation);
//         abt::BeliefNode *nextBelief = currentBelief->createOrGetChild(*(robotAction.get()), robotObservation);

//         // Update the agent's belief.
//         simulator_->getAgent()->updateBelief(*(robotAction.get()), robotObservation);
//         FilterResultPtr filterResult = generateParticles_(currentBelief,
//                                        nextBelief,
//                                        action,
//                                        observation,
//                                        allowTerminalStates);

//         if (filterResult)
//             return ABT::updateNextBelief_(std::move(filterResult), nextBelief);
//         return false;
//     }

//     virtual void serializeStep(std::ofstream& os) override {
//         os << "NUM HISTORIES: " << numHistories_ << endl;

//         // Log some statistics regarding the belief tree
//         auto treeInspector = static_cast<abt::ABTSimulator*>(simulator_.get())->getTreeInspector();
//         if (treeInspector) {
//             treeInspector->makeTreeStatistics(simulator_.get(), os);
//         }

//         if (problemEnvironmentOptions_->hasVerboseOutput) {
//             cout << "Belief #" << simulator_->getAgent()->getCurrentBelief()->getParentBelief()->getId() << endl;
//             std::stringstream newStream;
//             newStream << "After:" << endl;
//             solver_->printBelief(simulator_->getAgent()->getCurrentBelief()->getParentBelief(), newStream);
//             while (prevStream_->good() || newStream.good()) {
//                 std::string s1, s2;
//                 std::getline(*prevStream_, s1);
//                 std::getline(newStream, s2);
//                 cout << s1 << std::setw(40 - s1.size()) << "";
//                 cout << s2 << std::setw(40 - s2.size()) << "";
//                 cout << endl;
//             }

//             prevStream_ = new std::stringstream;
//             cout << endl;
//             auto currentBelief = simulator_->getAgent()->getCurrentBelief();

//             abt::HistoricalData* data = currentBelief->getHistoricalData();
//             if (data != nullptr) {
//                 cout << endl;
//                 cout << *data;
//                 cout << endl;
//             }
//             *prevStream_ << "Before:" << endl;
//             solver_->printBelief(currentBelief, *prevStream_);
//         }

//         if (static_cast<oppt::HMISolverOptions const*>(problemEnvironmentOptions_)->pruneEveryStep) {
//             auto currentBelief = simulator_->getAgent()->getCurrentBelief();
//             long nSequencesDeleted = solver_->pruneSiblings(currentBelief);
//             long nSequencesDeleted2 = static_cast<oppt::ABTSolver*>(solver_.get())->cleanHistories(currentBelief);
//             if ((!static_cast<oppt::HMISolverOptions const*>(problemEnvironmentOptions_)->hasChanges) &&
//                     !problemEnvironmentOptions_->interactive) {
//                 auto parentBeliefStates = currentBelief->getParentBelief()->getStates();
//                 for (auto & beliefState : parentBeliefStates) {
//                     static_cast<const shared::RobotState*>(beliefState)->getOpptState()->clearUserData();
//                 }
//                 static_cast<abt::OPPTStatePool*>(solver_->getStatePool())->cleanup();
//             }
//         }
//     } 

//     virtual VectorRobotStatePtr getBeliefParticles() override {
//         auto currentBelief = simulator_->getAgent()->getCurrentBelief();
//         std::vector<abt::State const*> currentParticles = currentBelief->getStates();
//         VectorRobotStatePtr beliefParticles(currentParticles.size());
//         for (size_t i = 0; i != beliefParticles.size(); ++i) {
//             oppt::RobotStateSharedPtr opptStateParticle =
//                 static_cast<shared::RobotState const*>(currentParticles[i])->getOpptState();
//             beliefParticles[i] = opptStateParticle;
//         }

//         return beliefParticles;
//     }

//     abt::Solver* const getSolver() const {
//         return simulator_->getSolver();
//     }

//     robot::RobotModel *const getRobotModel() const {
//         return static_cast<robot::RobotModel *>(simulator_->getModel());
//     }

//     abt::Simulator *const getSimulator() const {
//         return simulator_.get();
//     }

//     abt::BeliefNode *getCurrentBelief() const {
//         return simulator_->getAgent()->getCurrentBelief();
//     }

//     ParticleFilter *const getParticleFilter() const {
//         return particleFilter_.get();
//     }

// protected:

//     virtual FilterResultPtr generateParticles_(abt::BeliefNode *currentBelief,
//             abt::BeliefNode *nextBelief,
//             const ActionSharedPtr& action,
//             const ObservationSharedPtr& observation,
//             const bool &allowTerminalStates = false) {
//         const HMISolverOptions *hmiSolverOptions = static_cast<const HMISolverOptions*>(problemEnvironmentOptions_);

//         // Fill in the states of the current belief to the filter request
//         std::vector<abt::State const*> currentStates = currentBelief->getStates();

//         // New filter request
//         FilterRequestPtr filterRequest = std::make_unique<FilterRequest>();
//         filterRequest->robotEnvironment = robotPlanningEnvironment_;

//         for (size_t i = 0; i != currentStates.size(); ++i) {
//             auto st = static_cast<const shared::RobotState*>(currentStates[i]);
//             filterRequest->previousParticles.push_back(std::make_shared<Particle>(st->getOpptState(),
//                     st->getWeight()));
//         }

//         // Check if we need additional particles from the initial belief
//         if (currentBelief == solver_->getPolicy()->getRoot()) {
//             int diff = hmiSolverOptions->minParticleCount - filterRequest->previousParticles.size();
//             if (diff > 0) {
//                 LOGGING("Sampling additional particles from the initial belief");
//                 for (size_t i = 0; i != diff; ++i) {
//                     oppt::RobotStateSharedPtr state =
//                         robotPlanningEnvironment_->sampleInitialState();
//                     filterRequest->previousParticles.push_back(std::make_shared<Particle>(state, 
//                         state->as<oppt::WeightedRobotState>()->getWeight()));
//                 }
//             }
//         }

//         if (filterRequest->previousParticles.empty())
//             return nullptr;

//         // Fill in the particles of the next belief that were obtained during planning
//         /**std::vector<abt::State const*> nextStates = nextBelief->getStates();
//         for (size_t i = 0; i != nextStates.size(); ++i) {
//             auto st = static_cast<const shared::RobotState*>(nextStates[i]);
//             filterRequest->currentNextParticles.push_back(std::make_shared<Particle>(st->getOpptState(), st->getWeight()));
//         }*/

//         filterRequest->allowTerminalStates = allowTerminalStates;
//         filterRequest->allowZeroWeightParticles = false;
//         filterRequest->numParticles = hmiSolverOptions->minParticleCount;
//         filterRequest->action = action.get();
//         filterRequest->observation = observation.get();
//         filterRequest->allowCollisions = robotPlanningEnvironment_->getRobot()->getCollisionsAllowed();
//         filterRequest->randomEngine = robotPlanningEnvironment_->getRobot()->getRandomEngine();

//         FilterResultPtr filterResult = particleFilter_->filter(filterRequest);
//         if (filterResult->particles.empty())
//             return nullptr;
//         return std::move(filterResult);
//     }


// private:

//     std::unique_ptr<abt::Simulator> simulator_;
//     std::unique_ptr<robot::RobotModel> solverModel_;
//     std::unique_ptr<robot::RobotModel> simulatorModel_;
//     std::unique_ptr<ABTSolver> solver_;
//     long numHistories_ = 0;
//     long totalNumHistories_ = 0;

//     std::stringstream *prevStream_ = new std::stringstream;

//     hmi::Grid grid_;

//     std::string pipeToGama_;
//     std::string pipeToSolver_;

//     std::unique_ptr<ParticleFilter> particleFilter_ = nullptr;

// private:

//     void instantiatePipes() {
//         pipeToGama_ = static_cast<HMISolverOptions*>(problemEnvironmentOptions_)->pipePathToGama;
//         pipeToSolver_ = static_cast<HMISolverOptions*>(problemEnvironmentOptions_)->pipePathToSolver;
//     }

//     abt::ChangeSequence getChangeSequence(const std::vector<EnvironmentChangeSharedPtr>& environmentChanges) {
//         abt::ChangeSequence changeSequence;
//         for (auto & environmentChange : environmentChanges) {
//             auto changePtr = std::make_unique<oppt::GenericModelChange>(environmentChange);
//             changeSequence[simulator_->getStepCount()].push_back(std::move(changePtr));
//         }

//         return changeSequence;
//     }
    
// };

// }

// #endif

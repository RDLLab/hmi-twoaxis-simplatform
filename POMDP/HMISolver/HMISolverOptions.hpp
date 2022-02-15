// #ifndef _HMI_SOLVER_OPTIONS_HPP_
// #define _HMI_SOLVER_OPTIONS_HPP_

// #include "oppt/options/tclap/CmdLine.h"
// #include "oppt/options/option_parser.hpp"
// #include "oppt/problemEnvironment/ProblemEnvironmentOptions.hpp"
// #include "oppt/opptCore/typedefs.hpp"

// namespace oppt
// {
// struct HMISolverOptions: public ProblemEnvironmentOptions {
// public:
//     HMISolverOptions() = default;
//     virtual ~HMISolverOptions() = default;

//     /* The filepath to load the problem's grid environment. */
//     std::string gridPath = "oppt/models/HMIModel/HMIGrid.txt";

//     /* The filepaths to/from the FIFO pipes to communicate with GAMA. */
//     std::string pipePathToGama = "oppt/models/HMIModel/solver_to_gama_pipe";
//     std::string pipePathToSolver = "oppt/models/HMIModel/gama_to_solver_pipe";

//     std::string randomAgentsPath = "oppt/models/HMIModel/HMIRandomAgents.txt";
//     std::string transitionMatricesPath = "oppt/models/HMIModel/HMITransitionMatrices.txt";

//     std::string baseConfigPath = "";

//     /** The minimum number of particles to maintain in the active belief node. */
//     unsigned long minParticleCount = 1000;

//     /** Whether to completely re-build the tree from scratch if changes occur. */
//     bool resetOnChanges = false;

//     std::string policyPath = "";

//     bool savePolicy = false;
//     bool pruneEveryStep = false;
//     bool loadInitialPolicy = false;

//     static std::unique_ptr<options::OptionParser> makeParser(bool simulating) {
//         std::unique_ptr<options::OptionParser> parser =
//             ProblemEnvironmentOptions::makeParser(simulating);
//         addHMISolverOptions(parser.get());
//         return std::move(parser);
//     }

//     static void addHMISolverOptions(options::OptionParser* parser) {
//         parser->addOption<std::string>("HMISolver", "gridPath", &HMISolverOptions::gridPath);
//         parser->addOption<std::string>("HMISolver", "pipePathToGama", &HMISolverOptions::pipePathToGama);
//         parser->addOption<std::string>("HMISolver", "pipePathToSolver", &HMISolverOptions::pipePathToSolver);
//         parser->addOption<std::string>("HMISolver", "randomAgentsPath", &HMISolverOptions::randomAgentsPath);
//         parser->addOption<std::string>("HMISolver", "transitionMatricesPath", &HMISolverOptions::transitionMatricesPath);
//         parser->addOptionWithDefault<bool>("HMISolver", "savePolicy", &HMISolverOptions::savePolicy, false);
//         parser->addOptionWithDefault<bool>("HMISolver", "pruneEveryStep", &HMISolverOptions::pruneEveryStep, false);
//         parser->addOptionWithDefault<bool>("HMISolver", "loadInitialPolicy", &HMISolverOptions::loadInitialPolicy, false);
//         parser->addOptionWithDefault<unsigned long>("HMISolver", "minParticleCount",
//                 &HMISolverOptions::minParticleCount, 1000);
//         parser->addOptionWithDefault<std::string>("HMISolver",
//                 "policyPath",
//                 &HMISolverOptions::policyPath,
//                 "pol.pol");
//         parser->addOptionWithDefault<bool>("HMISolver", "resetOnChanges",
//                                            &HMISolverOptions::resetOnChanges, false);
//     }
// };
// }

// #endif

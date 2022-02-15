#include "oppt/problemEnvironment/ProblemEnvironment.hpp"
#include "HMISolver.hpp"
#include "HMISolverOptions.hpp"

int main(int argc, char const* argv[])
{
    oppt:ProblemEnvironment problemEnvironment;
    int ret = problemEnvironment.setup<solvers::HMISolver, oppt:HMISolverOptions>(argc, argv);
    if (ret != 0)
        return ret;
    return problemEnvironment.runEnvironment(argc, argv);
}

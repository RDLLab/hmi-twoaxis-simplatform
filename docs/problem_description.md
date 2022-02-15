# Problem Description

## Research Question

Can we incorporate ethical algorithms into existing POMDP-solving algorithms? Does this work both algorithmically and computationally? Is it actually effective, and does an “ethical” robot do what we want it to do?

## Relevant Reading

[Marcus Hoerger; Hanna Kurniawati, Alberto Elfes “A Software Framework for Planning under Partial Observability” in IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Madrid, Spain, October, 2018.](readings/IROS18_Hoerger_Final.pdf)

Marcus Hoerger; Hanna Kurniawati, Alberto Elfes “A Software Framework for Planning under Partial Observability” in IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Madrid, Spain, October, 2018.

## Implementation

### State space

The state space is made up of repeating *{x, y}* pairs for each robot in the problem, and then repeating *{x, y, condition}* tuples for each random agent in the problem.

### Action space

Each action is an *{x, y}* pair which takes the robot to cell *{x, y}* on the problem grid. If a random agent resides on that cell when the robot takes this action, the random agent stays on that cell for the robot to come help them.

### Observation space

With each action, the robot moves a certain number of steps towards its destination cell. At each step, the robot ‘looks around’ to see if it can deduce the condition of any random agent it can see. ‘Sight’ is this problem is naively defined by there existing a line of the form *y = mx + c* such that for every *x* in the domain *[robotX, randAgentX]* or the domain *[randAgentX, robotX]* (whichever one is a valid domain), the cell *{x, y}* is a valid cell on the problem grid. If the robot *can* see a certain random agent, the probably that it deduces its condition is **0.8** (although this can be subject to change).

### Communicating with simulation environment

The simulation part of the solver is run by GAMA. GAMA and the solver communicate via two single-direction FIFO pipes. Both can be found in the `oppt/pipes` directory.

Initially, GAMA will write several files in the `oppt/models/HMIModel` directory. These files contain the data for the grid, random agent types and random agent transition matrices for the given problem. These files are of the following forms:

**Grid:** Consists, in this order, of `grid_width`, `grid_height` and the status of each individual cell in left to right, top to bottom order. The grid width, grid height and cell details are each separated by a comma. If a cell is traversable (that is, it is not a wall, pit, etc.), it is represented by a `_`. If it is not traversable, it is represented by a `*`. So, for example, a 5x5 grid with only walls at {2,2}, {2,4}, {4,2} and {4,4} would be represented as `5,5,______*_*_______*_*______`. This would create the grid:
`_____`
`_*_*_`
`_____`
`_*_*_`
`_____`

**Transition matrices:** Represented by a text file with multiple lines. The first line contains the number of conditions each random agent has in the given problem. Each subsequent line contains the type of random agent to which the transition matrix applies, and then the details of the transition matrix. For example, a problem with four conditions, as well as a toddler type and an elderly type might have a transition matrix file that looks like this:

`4
toddler,0.9,0.1,0.0,0.0,0.5,0.33,0.0,0.17,0.8,0.0,0.1,0.1,0.0,0.0,0.0,1.0`
`elderly,0.1,0.2,0.3,0.5,0.6,0.3,0.05,0.05,0.3,0.2,0.3,0.2,0.9,0.1,0.0,0.0`

**Random agents:** Contains the order of the random agents in the state space of the problem. It also determines what types of random agents are in the problem. So, in a problem with two toddlers and two elderly people, the random agent text file might look like this:
`toddler,0`
`toddler,1`
`elderly,0`
`elderly,1`

These three files are created before run-time. During run-time, the solver will determine the optimal action given the current state, and send this action over to GAMA. This action is represented by a pair of coordinates, which are the coordinates the robot should move to on the grid. If there is a random agent at these coordinates, it is implied that the robot tells that random agent to wait there while it comes over to help it. Therefore, that random agent will stay where it is until the robot moves to its coordinates, at which point it will reset its condition to 0 (’happy’).

GAMA then runs the simulation with the given action, and returns the resulting state and any observations made along the way. The observation space is defined as a list of conditions, with one condition corresponding to each agent. If the robot has not observed a given random agent’s condition, it assumes that it has not changed. If the robot **has** observed a given random agent’s condition, it then updates that agent’s known condition via the observation function.

When GAMA sends this over to the solver, it sends it over two messages. The first one is sent to the solver’s transition plugin, and it is the resulting state with all observations taken into account. The second one is sent to the solver’s observation plugin, and it is the list of all observations made for each agent. If a given observation is a non-negative number, that means an observation was actually made for the corresponding random agent. If it is -1, on the other hand, that means that an observation was not made.

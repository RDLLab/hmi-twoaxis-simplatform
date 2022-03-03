/**
* Name: HMIGamaInterface
* Based on the internal empty template. 
* Author: Vikram Sondergaard
* Tags: 
*/

model HMIGamaInterface

/* This model is intended to be placed in the directory that contains the root folder
 * for OPPT. That is, you should have a directory structure as such:
 * 
 * `path/to/oppt/<oppt_root_folder>`
 * `path/to/oppt/HMIGamaInterface`
 */

global {
	
	/** Width of the grid. */
	int grid_size <- 5;
	
	/** Sprite size. */
	float size <- 10.0;
	float reduction_factor <- 1.5;
	
	/** Layout of the grid. */
	string grid_layout <- "_____" +
	                      "_*_*_" +
	                      "_____" +
	                      "_*_*_" +
	                      "_____";
	                      
	string working_directory <- copy_between(command("pwd"), 0, length(command("pwd")) - 1) + "/";
	
	string base_config_file <- working_directory + "../includes/HMISolver.cfg";
	
	/** Root directory of OPPT. */
	string base_directory <- working_directory + "../../oppt_install/oppt/";
	
	/** Directory to all information related to the model. */
	string model_directory <- base_directory + "models/HMIModel/";
	
	/** Path to a text file containing details about the problem grid. */
	string grid_path <- model_directory + "HMIGrid.txt";
	
	/** Path to a text file containing details about each random agent's 
	 *  condition transition matrix. */
	string matrices_path <- model_directory + "HMITransitionMatrix.txt";
	
	/** Path to a text file containing details about the problem's initial 
	 * state. */
	string initial_state_path <- model_directory + "HMIInitialState.txt";
	
	/** Path to a text file containing details about the random agents in this
	 *  problem. */
	string randag_path <- model_directory + "HMIRandomAgents.txt";
	
	/** Path to a named pipe to send data to the solver about the problem's
	 *  current belief state. */
	string state_to_solver <- base_directory + "pipes/statePipeToSolver";
	
	/** Path to a named pipe to send data to the solver about the model's most
	 *  recent observation. */
	string obs_to_solver <- base_directory + "pipes/observationPipeToSolver";
	
	/** Path to a named pipe to send files to this GAMA model. */
	string path_to_gama <- base_directory + "pipes/pipeToGama";

    string config_file_path <- base_directory + "cfg/HMISolver.cfg";
    
    list<string> randag_types <- ["ELDERLY", "TODDLER"];
	
	map<string, int> randag_map <- create_map(randag_types, [1, 1]);
	
	list<string> random_agents;
	
	list<point> robot_locations <- [{2,2}];
	
	map<string, list<point>> randag_locations <- create_map(randag_types, [[{0,0}], [{4,4}]]);
	map<string, list<int>> randag_conditions <- create_map(randag_types, [[0], [0]]);
	
	//list<point> randag_locations <- [{0,0}, {4,4}];
	
	//list<int> randag_conditions <- [0,0];
	
	/** A map mapping each random agent type to its corresponding condition 
	 * transition matrix.
	 * 
	 * Rows represent the given state, columns represent the next state.
	 * 
	 * 0 - happy
	 * 1 - agitated
	 * 2 - sad
	 * 3 - angry
	 * 
	 * For example, the values in column 0 denote the following probabilities:
	 * 
	 * - Row 0: P{s_t+1 = happy | s_t = happy};
	 * - Row 1: P{s_t+1 = agitated | s_t = happy};
	 * - Row 2: P{s_t+1 = sad | s_t = happy}; and
	 * - Row 3: P{s_t+1 = angry | s_t = happy}.
	 * */
	map<string, matrix<float>> matrices <- create_map(randag_map.keys, [
		/* Elderly state transition matrix */
		(matrix([[0.9, 0.0, 0.0, 0.5],
                [0.1, 0.8, 0.0, 0.0],
                [0.0, 0.1, 1.0, 0.5],
                [0.0, 0.1, 0.0, 0.0]])),
        /* Toddler state transition matrix */
        (matrix([[0.9, 0.0, 0.0, 0.5],
                [0.1, 0.8, 0.0, 0.0],
                [0.0, 0.1, 1.0, 0.5],
                [0.0, 0.1, 0.0, 0.0]]))
	]);
	
	/** Number of conditions in the problem. */
	int num_conditions <- 4;
	
	int num_robots;
	
	/** The base filepath for any images used in this simulation. */
	string uri_base <- "../includes/";
	
	/**
	 * Initialise all global facets of the model.
	 */
	init {
		write "Running method init() of species world...";
		string cmd <- command("mkdir -p " + model_directory);
		do refresh_pipes();
		do initialise_grid();
		write "Completed method init() of species world...";
	}
	
	reflex init_agents when: world.cycle = 0 {
		write "Running method init_agents() of species world...";
		random_agents <- generate_randag_strings();
		string random_agent_string <- initialise_random_agents();
		string robot_string <- initialise_robots();
		do edit_config_file(robot_string, random_agent_string);
		do send_transition_matrices();
		do send_grid();
		write "Completed method init_agents() of species world...";
	}
	
	list<string> generate_randag_strings {
		write "Running method generate_randag_strings() of species world...";
		list<string> res <- [];
		loop r over: randag_map.keys {
			loop i from: 0 to: (randag_map at r) - 1 {
				res <- res + (r + "," + string(i));
			}
		}
		write "Completed method generate_randag_strings() of species world...";
		return res;
	}
	
	action edit_config_file(string robot_state, string random_agent_state) {
		write "Running method edit_config_file() of species world...";
		file cfg_file <- file(base_config_file) writable true;
		map<string, string> to_replace;
		string res <- "";
		put robot_state key: "initialRobotState" in: to_replace;
		put random_agent_state key: "initialRandomAgentState" in: to_replace;
		put get_state_string() key: "[state]" in: to_replace;
		put get_action_string() key: "[action]" in: to_replace;
		put get_obs_string() key: "[observation]" in: to_replace;
		list<string> facets <- ["[state]", "[action]", "[observation]"];
		put string(grid_size) key: "numInputStepsActions" in: to_replace;
		put string(num_conditions) key: "numInputStepsObservations" in: to_replace;
		loop i from: 0 to: length(cfg_file) - 1 {
			string to_add <- "";
			loop k over: to_replace.keys {
				if ((cfg_file at i) contains k) {
					if (facets contains k) {
						to_add <- k + "\n\n" + (to_replace at k);
					}
					else {
						to_add <- k + " = " + (to_replace at k);
					}
					remove k from: to_replace;
					break;
				}
			}
			if (empty(to_add)) {to_add <- cfg_file at i;}
			res <- res + to_add + "\n";
		}
		save res to: config_file_path type: "text";
		write "Running method edit_config_file() of species world...";
	}
	
	string get_state_string {
		write "Running method get_state_string() of species world...";
		int dimensions <- 2 * length(helper_robot) + 3 * length(random_agent);
		string add_dim <- "additionalDimensions = " + string(dimensions);
		string limits <- "additionalDimensionLimits = [";
		loop times: length(helper_robot) {
			limits <- limits + "[0, " + string(grid_size - 1) + "], ";
			limits <- limits + "[0, " + string(grid_size - 1) + "], ";
		}
		loop times: length(random_agent) {
			limits <- limits + "[0, " + string(grid_size - 1) + "], ";
			limits <- limits + "[0, " + string(grid_size - 1) + "], ";
			limits <- limits + "[0, " + string(num_conditions - 1) + "], ";
		}
		limits <- copy_between(limits, 0, length(limits) - 2) + "]";
		return add_dim + "\n\n" + limits + "\n\n";
		write "Completed method get_state_string() of species world...";
	}
	
	string get_action_string {
		write "Running method get_action_string() of species world...";
		int dimensions <- 2 * length(helper_robot);
		string add_dim <- "additionalDimensions = " + string(dimensions);
		string limits <- "additionalDimensionLimits = [";
		loop times: length(helper_robot) {
			limits <- limits + "[0, " + string(grid_size - 1) + "], ";
			limits <- limits + "[0, " + string(grid_size - 1) + "], ";
		}
		limits <- copy_between(limits, 0, length(limits) - 2) + "]";
		write "Completed method get_action_string() of species world...";
		return add_dim + "\n\n" + limits + "\n\n";
	}
	
	string get_obs_string {
		write "Running method get_obs_string() of species world...";
		int dimensions <- length(random_agent);
		string add_dim <- "additionalDimensions = " + string(dimensions);
		string limits <- "additionalDimensionLimits = [";
		loop times: length(random_agent) {
			limits <- limits + "[0, " + string(num_conditions - 1) + "], ";
		}
		limits <- copy_between(limits, 0, length(limits) - 2) + "]";
		write "Completed method get_obs_string() of species world...";
		return add_dim + "\n\n" + limits + "\n\n";
	}
	
	/**
	 * Deletes and re-instantiates the named pipes used for communication
	 * between this program and the solver.
	 */
	action refresh_pipes {
		write "Running method refresh_pipes() of species world...";
		string return_dir <- command("pwd");
		string cmd <- command("cd ../../oppt_hmi_scripts && ./setPipes.sh");
		cmd <- command("cd " + return_dir);
		write "Completed method refresh_pipes() of species world...";
	}
	
	/**
	 * Initialises the problem grid. This is done by reading a string
	 * containing instances of "_" (for traversable cells) and "*" (for
	 * non-traversable cells, eg. walls).
	 */
    action initialise_grid {
    	write "Running method initialise_grid() of species world...";
		ask grid_cell {
			string cell_id <- grid_layout at ((grid_y * grid_size) + grid_x);
			traversable <- cell_id = "_";
			color <- traversable ? #white : #black;
		}
		write "Completed method initialise_grid() of species world...";
	}
	
	/**
	 * Initialises the random agents of this problem in GAMA.
	 * 
	 * @param init_randag_state the initial random agent state of the problem,
	 *                          as specified by the HMISolver config file
	 */
	string initialise_random_agents {
		write "Running method initialise_random_agents() of species world...";
		int idx <- 0;
		string randag_str <- "";
		string init_state_string <- "[";
		loop i from: 0 to: length(random_agents) - 1 {
			list<string> randag_details <- (random_agents at i) split_with ",";
			string type <- randag_details at 0;
			int id <- int(randag_details at 1);
			point start_point <- (randag_locations at type) at id;
			int start_condition <- (randag_conditions at type) at id;
			do create_random_agent(random_agents at i, start_point, start_condition);
			randag_str <- randag_str + (random_agents at i) + "\n";
			init_state_string <- init_state_string + string(start_point.x) + ", " + string(start_point.y) + ", " + string(start_condition);
			if (i < (length(random_agents) - 1)) {
				init_state_string <- init_state_string + ", ";
			}
			else {
				init_state_string <- init_state_string + "]";
			}
		}
		string cmd <- send_to_pipe(randag_str, randag_path);
		write "Completed method initialise_random_agents() of species world...";
		return init_state_string;
	}
	
	/**
	 * Initialises the robots of this problem in GAMA.
	 * 
	 * @param init_robot_state  the initial robot state of the problem, as
	 *                          specified by the HMISolver config file
	 * @param init_randag_state the initial random agent state of the problem,
	 *                          as specified by the HMISolver config file
	 */
    string initialise_robots {
    	write "Running method initialise_robots() of species world...";
    	string init_state_string <- "[";
    	loop i from: 0 to: num_robots - 1 {
    		point l <- robot_locations at i;
    		do create_robot(l, i);
    		init_state_string <- init_state_string + string(l.x) + ", " + string(l.y) + ", ";
    	}
    	write "Running method initialise_robots() of species world...";
    	return copy_between(init_state_string, 0, length(init_state_string) - 2) + "]";
	}
	
	/**
	 * @param state a state describing locations of agents
	 * @param idx   an index corresponding to the location of a certain agent
	 *              in the given state
	 * 
	 * @return a location from two adjacent elements in the given state,
	 *         starting from the given index
	 */
	point point_from_state(list<int> state, int idx) {
		write "Running and completing point_from_state() of species world...";
		return {state at idx, state at (idx + 1)};
	}
	
	/**
	 * Sends data relating to the random agent transition matrices of this
	 * problem over to the solver, for its planning purposes.
	 */
	action send_transition_matrices {
		write "Running method send_transition_matrices() of species world...";
		string matrices_data <- string(num_conditions) + "\n";
		loop k over: matrices.keys {
			matrices_data <- matrices_data + k + ",";
			matrix<float> current_matrix <- matrices at k;
			matrices_data <- matrices_data + matrix_to_string(current_matrix);
		}
		string cmd <- send_to_pipe(matrices_data, matrices_path);
		write "Completed method send_transition_matrices() of species world...";
	}
	
	/**
	 * @param mat a matrix of floating-point numbers
	 * 
	 * @return the same matrix, represented as a string of floats separated by
	 *         commas, from top to bottom and left to right
	 */
	string matrix_to_string(matrix<float> mat) {
		write "Running method matrix_to_string() of species world...";
		string res <- "";
		loop col from: 0 to: num_conditions - 1 {
			loop row from: 0 to: num_conditions - 1 {
				res <- res + string(mat at {row, col}) + ",";
			}
		}
		write "Completed method matrix_to_string() of species world...";
		return res + "\n";
	}
	
	/**
	 * Sends data relating to the grid of this problem over to the solver, for
	 * its planning and execution purposes.
	 */
	action send_grid {
		write "Running method send_grid() of species world...";
		string grid_data <- string(grid_size) + "," + string(grid_size) +
		    "," + grid_layout;
		string cmd <- send_to_pipe(grid_data, grid_path);
		write "Running method send_grid() of species world...";
	}
	
	/**
	 * Creates the robot(s) in this problem.
	 * 
	 * @param start_point                the robot's starting point
	 * @param initial_random_agent_state the initial state of the random agents
	 *                                   in the problem, useful for formulating
	 *                                   a belief
	 */
	action create_robot(point start_point, int id) {
		write "Running create_robot() action of species world...";
		create helper_robot {
			if (id = 0) {
				has_key <- true;
			}
			do set_grid_cell(start_point, id);
		    location <- my_cell.location;
		    do set_initial_state();
		}
		write "Completed create_robot() action of species world...";
	}
	
	/**
	 * Creates a new random agent in this problem.
	 * 
	 * @param given_id        the random agent's type and ID
	 * @param start_point     the random agent's starting location
	 * @param start_condition the random agent's starting condition
	 */
	action create_random_agent(string randag_details, point start_point, int start_condition) {
		write("Running create_dependent() action of species global...");
		create random_agent {
			do set_type_and_id(randag_details);
			my_cell <- grid_cell grid_at start_point;
			location <- my_cell.location;
			state_change_matrix <- matrices at type;
			condition <- start_condition;
			string base <- uri_base + lower_case(type);
			my_icon <- image_file(base + ".jpg");
			my_help_icon <- image_file(base + "_help.jpg");
		}
		write("Completed create_dependent() action of species global...");
	}

    /**
     * Communicates with a pipe/file. This method is used extensively for
     * sending and receiving data.
     * 
     * @param data    the data to send to a given location (if this method is
     *                sending data)
     * @param pipe    the location to or from which to send or receive data
     * @param sending whether this method is sending or receiving data
     * 
     * @return the empty string if this method is sending data, or the data in
     *         string form if this method is receiving data
     */
	string send_to_pipe(string data, string pipe) {
		write("Running send_to_pipe() function of species global...");
		string cmd;
		if (((pipe split_with ".") at 1) = "txt") {
	        cmd <- command("touch " + pipe);
		}
		cmd <- command("echo -n \"" + data + "\" > " + pipe);
		write("Completed send_to_pipe() function of species global...");
		return cmd;
	}
}

species random_agent {
	
	/** The random agent's type (eg. elderly). A random agent's type determines
	 *  what condition transition matrix it subscribes to. */
	string type;
	
	/** The random agent's unique ID. Each random agent in a problem must have
	 *  a unique type and ID combination. */
	int id;
	
	/** The grid cell occupied by this random agent. */
	grid_cell my_cell;
	
	/** The condition transition matrix to which this random agent 
	 * subscribes. */
	matrix<float> state_change_matrix;
	
	/** Whether this random agent knows it will be receiving care from a helper
	 *  robot or not. */
	bool left_alone;
	
	/** The condition of this random agent. At the moment this is an integer,
	 *  however this may change to become an emotion type. */
	int condition;
	
	/** The sprite of this random agent, used in experiments. */
	image_file my_icon;
	
	/** The sprite of this random agent when it requires help from a helper
	 *  robot. */
	image_file my_help_icon;
	
	/** The size of this random agent's sprite. */
	int my_size <- size;
	
	/** 
	 * Initialises this random agent. Most of this work is done in the
	 * `world.init()` method, however it starts assuming it will not be
	 * receiving help from a helper robot.
	 */
	init {
		write("Running init() of species random_agent...");
		left_alone <- true;
		write("Completed init() of species random_agent...");
	}
	
	/**
	 * Sets the type, ID and also name of this random agent.
	 * 
	 * @param randag_details a string containing a type and a numerical ID
	 *                       separated by commas, which will define this
	 *                       agent's type and ID
	 */
	action set_type_and_id(string randag_details) {
		write "Running method set_type_and_id() of species random_agent...";
		list<string> type_and_id <- randag_details split_with ",";
		type <- type_and_id at 0;
		id <- 0;
		name <- randag_details;
		write "Running method set_type_and_id() of species random_agent...";
	}
	
	/**
     * Changes state according to the agent's stochastic state change matrix.
     */
    reflex change_state when: left_alone {
    	write("Running change_state() reflex of species random_agent...");
	 	int i <- 0;
	 	list<float> condition_row <- state_change_matrix row_at condition;
	 	float state_change <- rnd(sum(condition_row)) - (condition_row at i);
	 	loop while: state_change > 0.0 {
	 		i <- i + 1;
	 		state_change <- state_change - (condition_row at i);
	 	}
	 	condition <- i;
	 	write("Completed change_state() reflex of species random_agent...");
    }
    
    /**
     * Makes a random move to a neighboring cell with 50% chance.
     */
    reflex make_move when: condition = 0 and left_alone {
    	write("Running make_move() reflex of species random_agent...");
    	int move_counter <- rnd(7);
        int x_change <- 0;
    	int y_change <- 0;
    	if (move_counter = 0) {y_change <- -1;}
	 	else if (move_counter = 1) {x_change <- 1;}
	 	else if (move_counter = 2) {y_change <- 1;}
	 	else if (move_counter = 3) {x_change <- -1;}
	 	my_cell <- my_cell.find_neighboring_cell(x_change, y_change);
	 	location <- my_cell.location;
	 	write("Completed make_move() reflex of species random_agent...");
    }
    
    /**
     * Shrinks and justifies this random agent's sprite accordingly
     * depending on how many other random agents are in the same cell. At
     * present, a given cell cannot handle more than four robots and/or random
     * agents.
     */
    action shrink_and_justify {
    	// write "Running method shrink_and_justify() of species random_agent...";
    	my_size <- size / reduction_factor;
    	list<point> offsets <- [{-1, -1}, {1, -1}, {-1, 1}, {1, 1}];
    	loop i from: 0 to: length(my_cell.dependents_in_cell) - 1 {
    		if ((my_cell.dependents_in_cell at i) = self) {
    			point multiplier <- {grid_size, grid_size} / 4.0;
    			location <- location + (offsets at i) * multiplier;
    		}
    	}
    	// write "Completed method shrink_and_justify() of species random_agent...";
    }
    
    /**
     * Sets up the sprite for this random agent. If there are several random
     * agents and/or robots at this random agent's cell, it also shrinks this
     * agent's sprite and moves it into a corner, so all of the agents can be
     * seen. It also changes the sprite's colour if it needs help.
     */
    aspect sprite {
    	// write "Running method sprite() of species random_agent...";
    	bool other_randags <- length(my_cell.dependents_in_cell) > 1;
    	bool is_there_robot <- !empty(my_cell.robots_in_cell);
    	if (other_randags or is_there_robot) {do shrink_and_justify();}
    	else                                 {my_size <- size;}
    	if (condition = 0) {draw my_icon size: my_size;}
    	else               {draw my_help_icon size: my_size;}
    	// write "Completed method sprite() of species random_agent...";
    }
	
}

species helper_robot {
	
	int id;
	
	/** The grid cell occupied by this helper robot. */
	grid_cell my_cell;
	
	/** Whether this robot has observations to send to the solver. */
	bool has_observations <- false;
	
	/** The path this robot must take to achieve its current action. This is
	 *  instantiated as empty because this robot starts with no action. */
	list path_to_follow <- [];
	
	/** The conditions of all random agents in the problem as believed by this
	 *  helper robot. */
	map<string, int> randag_conditions;
	
	/** The locations of all random agents in the problem as known by this
	 *  helper robot. */
	map<string, point> randag_locations;
	
	bool has_key <- false;
	
	/** The sprite of this helper robot, used for visualisation in 
	 * experiments. */
	image_file my_icon <- image_file("../includes/robot.jpg");
	
	int my_size <- size;
	
	init {}
	
	/**
	 * Sets the grid cell of this robot according to a given start point.
	 * 
	 * @param start_point the starting point of this robot
	 */
	action set_grid_cell(point start_point, int id) {
		write "Running method set_grid_cell() of species helper_robot...";
		bool x_out <- start_point.x < 0 or start_point.x >= grid_size;
		bool y_out <- start_point.y < 0 or start_point.y >= grid_size;
		if (x_out or y_out) {
			my_cell <- any(grid_cell where each.traversable);
		}
		else {
			my_cell <- grid_cell[start_point.x, start_point.y];
		}
		self.id <- id;
	    write "Completed method set_grid_cell() of species helper_robot...";
	}
	
	/**
	 * Sets the initial belief state of this robot according to a given initial
	 * state.
	 * 
	 * @param initial_randag_state the initial state of all of the random
	 *                             agents in this problem
	 */
	action set_initial_state {
		write "Running method set_initial_state() of species helper_robot...";
		loop i from: 0 to: length(random_agents) - 1 {
			list<string> randag_details <- (random_agents at i) split_with ",";
			string type <- randag_details at 0;
			int id <- int(randag_details at 1);
			put ((world.randag_locations at type) at id) key: (random_agents at i) in: randag_locations;
			put ((world.randag_conditions at type) at id) key: (random_agents at i) in: randag_conditions;
		}
		write "Completed method set_initial_state() of species helper_robot...";
	}
	
	/**
	 * Get the difference along the x- and y-axes between this helper robot's
	 * location and the given agent's location per unit movement along the axis
	 * with the larger distance.
	 * 
	 * @param agent_cell the given agent's cell
	 * 
	 * @return a point containing the change along the x- and y-axes
	 *         respectively per unit movement along the axis with the larger
	 *         distance
	 */
	point get_deltas(grid_cell agent_cell) {
		write "Running method get_deltas() of species helper_robot...";
		float x_dist <- agent_cell.grid_x - my_cell.grid_x;
		float y_dist <- agent_cell.grid_y - my_cell.grid_y;
		float max_dist <- max(abs(x_dist), abs(y_dist));
		write "Completed method get_deltas() of species helper_robot...";
		return {x_dist / max(1, max_dist), y_dist / max(1, max_dist)};
	}
	
	/**
	 * Checks whether this helper robot can see the given agent's cell.
	 * 
	 * "Seeing" in this method is defined by there existing a raster line of
	 * the form `y = mx + c` between this helper robot and the given random
	 * agent.
	 * 
	 * @param agent_cell the given agent's cell
	 * 
	 * @return whether this helper robot can see the given agent's cell
	 */
	bool can_see(grid_cell agent_cell) {
		write "Running method can_see() of species helper_robot...";
		float x <- my_cell.grid_x;
		float y <- my_cell.grid_y;
		point deltas <- get_deltas(agent_cell);
		grid_cell current_cell <- my_cell;
	 	loop while: current_cell.traversable and current_cell != agent_cell {
	 		x <- x + deltas.x;
	 		y <- y + deltas.y;
			current_cell <- grid_cell grid_at {round(x), round(y)};
	 	}
	 	write "Completed method can_see() of species helper_robot...";
	 	return current_cell = agent_cell;
	}
	
	/**
	 * Looks for a specific random agent in the problem. If this robot "sees"
	 * that random agent, there is a certain chance (defined by the variable
	 * `c` in this method) that it will observe that random agent's condition.
	 * 
	 * @param a the random agent that this helper robot is looking for
	 */
	action look_for(random_agent a) {
		write("Running look_for() action of species helper_robot...");
		float c <- 0.8;
	 	bool can_see <- can_see(a.my_cell);
	 	if can_see and rnd(100) / 100.0 <= c {
	 		put a.condition key: a.name in: randag_conditions;
	 	}
	 	write("Completed look_for() action of species helper_robot...");
	 }
	 
	 /**
	  * Changes a given random agent's condition to 0 (happy), observes this
	  * change in condition and leaves the given random agent alone.
	  * 
	  * @param a the given random agent
	  */
	action help_agent(random_agent a) {
	 	write("Running help_agent() action of species helper_robot...");
	 	ask a {
	 		left_alone <- true;
	 		condition <- 0;
	 	}
	 	put a.condition key: a.name in: randag_conditions;
	 	write("Completed help_agent() action of species helper_robot...");
	}
	 
	 /**
	  * Observes every random agent's location, and tries to observe each one's
	  * condition as well.
	  */
    reflex look_around {
	    write("Running look_around() reflex of species helper_robot...");
	 	loop r over: random_agent {
	 	    put {r.my_cell.grid_x, r.my_cell.grid_y} key: r.name in: randag_locations;
	 		float c <- 0.8;
	 		if rnd(100) / 100.0 <= c {
	 			ask helper_robot {
	 			    put r.condition key: r.name in: randag_conditions;
	 			}
	 		}
	    }
	 	//do look_for(r);
	 	write("Completed look_around() reflex of species helper_robot...");
	}
	 
	 /**
	 * @param strings a list of strings
	 * 
	 * @return a string containing each element of the given list in order,
	 *         separated by commas
	 */
	string pad_with_commas(list elements) {
		write "Running method pad_with_commas() of species helper_robot...";
		string res <- "";
		loop e over: elements {
			res <- res + string(e) + ",";
		}
		write "Completed method pad_with_commas() of species helper_robot...";
		return res;
	}
	
	/**
	 * @return this robot's knowledge of the next state and its belief of the
	 *         random agents' conditions, as two strings
	 */
	list<string> get_state_and_observation {
		write "Running method get_state_and_observation() of species helper_robot...";
		string next_state <- "";
		loop r over: helper_robot {
			next_state <- next_state + pad_with_commas([r.my_cell.grid_x, r.my_cell.grid_y]);
		}
	 	string observation <- "";
	 	loop r over: random_agents {
	 	    point location <- randag_locations at r;
	 	    int condition <- randag_conditions at r;
	 	    list<string> agent_info <- [location.x, location.y, condition];
	 	    next_state <- next_state + pad_with_commas(agent_info);
	 	    observation <- observation + string(condition) + ",";
	 	}
	 	write "Completed method get_state_and_observation() of species helper_robot...";
	 	return [next_state, observation];
	}
	
	/**
	 * Sends the state and observation observed by this helper robot to the
	 * solver.
	 */
	action send_state_and_observation {
		list<string> state_and_obs <- get_state_and_observation();
	 	string next_state <- state_and_obs at 0;
	 	string observation <- state_and_obs at 1;
	 	write("Next State:\n" + next_state);
	 	write("Observation:\n" + observation);
	 	// write("State string to send is " + state_and_obs at 0);
	 	// write("observation string to send is " + state_and_obs at 1);
	 	string res <- world.send_to_pipe(next_state, state_to_solver);
	 	res <- world.send_to_pipe(observation, obs_to_solver);
	}
	
	/**
	 * Notifies the random agents in the given grid cell that they will be
	 * looked after, causing them to wait for help to eventually arrive.
	 * 
	 * @param target_cell the cell that the robot is going to visit
	 */
	action notify_random_agents(grid_cell target_cell) {
		ask target_cell.dependents_in_cell {
	 		left_alone <- false;
	 	}
	}
	
	/**
	 * @param target_cell the cell that this robot wants to move to
	 * 
	 * @return a list of grid cells that this robot must pass through to arrive
	 *         at the given target cell
	 */
	list find_path_to_target(grid_cell target_cell) {
		list<grid_cell> traversables <- grid_cell where each.traversable;
	 	path shortest_path <- path_between(traversables, my_cell, target_cell);
	 	return shortest_path.vertices;
	}
	
	action do_action(point<int> target_point) {
		write "Target point is " + string(target_point);
		loop a over: my_cell.dependents_in_cell {
	 		do help_agent(a);
	 	}
		ask my_cell {
	 		color <- #white;
	 	}
		grid_cell target_cell <- grid_cell grid_at target_point;
		do notify_random_agents(target_cell);
		path_to_follow <- find_path_to_target(target_cell);
		ask target_cell {
			color <- #orange;
		}
		has_observations <- true;
	}
	
	reflex take_action when: empty(path_to_follow) and has_key {
	    // write("Running take_action() reflex of species helper_robot...");
	 	has_key <- false;
	 	if (has_observations) {
	 		do send_state_and_observation();
	 	}
	 	// write("Helper robot is waiting for action...");
	 	string action_str <- command("cat < " + path_to_gama);
	 	action_str <- copy_between(action_str, 0, length(action_str) - 1);
	 	write("ACTION:\n" + action_str);
	 	// write("Received action " + action_str + " from solver");
	 	list<int> points <- action_str split_with ",";
	 	pair<helper_robot, int> max_robot_path_length <- self::-1;
	 	loop i from: 0 to: length(helper_robot) - 1 {
	 		point<int> target_point <- {points at (2*i), points at (2*i + 1)};
	 		ask helper_robot at i {
	 			do do_action(target_point);
	 			int path_length <- length((helper_robot at i).path_to_follow);
	 			if (path_length > max_robot_path_length.value) {
	 				max_robot_path_length <- (helper_robot at i)::path_length;
	 			}
	 		}
	 	}
        ask max_robot_path_length.key {
	 			has_key <- true;
	 	}
	 	// write("Completed take_action() reflex of species helper_robot...");
	 }
	 
	 reflex execute_step when: !empty(path_to_follow) {
	 	// write("Running execute_step() reflex of species helper_robot...");
	 	my_cell <- grid_cell(path_to_follow at 0);
	 	location <- my_cell.location;
	 	path_to_follow <- copy_between(path_to_follow, 1, length(path_to_follow));
	 	// write("Completed execute_step() reflex of species helper_robot...");
	 }
	 
//	 reflex help_agent {
//	 	// write("Running help_agent() reflex of species helper_robot...");
//	 	loop r over: random_agent {
//	 		if (my_cell = r.my_cell) {
//	 			do help_agent(r);
//	 		}
//	 	}
//	 	// write("Completed help_agent() reflex of species helper_robot...");
//	 }
	 
	 action shrink_and_justify(int num_randags) {
    	my_size <- size / reduction_factor;
    	list<point> offsets <- [{-1, -1}, {1, -1}, {-1, 1}, {1, 1}];
    	int idx <- my_cell.robots_in_cell index_of self;
    	point multiplier <- {grid_size, grid_size} / 4.0;
    	location <- location + ((offsets at (idx + num_randags)) * multiplier);
    }
	 
	 aspect sprite {
	 	bool randags_in_cell <- !empty(my_cell.dependents_in_cell);
	 	bool other_robots <- length(my_cell.robots_in_cell) > 1;
	 	if (randags_in_cell or other_robots) {
    		do shrink_and_justify(length(my_cell.dependents_in_cell));
    	}
    	else {
    		my_size <- size;
    	}
    	draw my_icon size: my_size;
     }
	
}

grid grid_cell width: grid_size height: grid_size neighbors: 4 {
	bool traversable;
	list<helper_robot> robots_in_cell -> {helper_robot inside self};
	list<random_agent> dependents_in_cell -> {random_agent inside self};
	
	/**
	 * Finds the neighboring cell of a given agent's location in a given
	 * direction.
	 * 
	 * @param cell:   the given cell on the grid
	 * @param x_diff: the difference along the x-axis between the given cell and the required neighbor
	 * @param y_diff: the difference along the y-axis between the given cell and the required neighbor
	 * @return        the cell at the correct offset from the given cell
	 */
	grid_cell find_neighboring_cell(int x_diff, int y_diff) {
		// write("Running find_neighboring_cell() function of species grid_cell...");
		point new_point <- {self.grid_x + x_diff, self.grid_y + y_diff};
		bool x_out <- new_point.x < 0 or new_point.x >= grid_size;
		bool y_out <- new_point.y < 0 or new_point.y >= grid_size;
		if (x_out or y_out) {return self;}
		if ((grid_cell grid_at new_point).traversable) {
			// write("Completed find_neighboringin_cell() function of species grid_cell...");
			return grid_cell grid_at new_point;
		}
		// write("Completed find_neighboring_cell() function of species grid_cell...");
		return self;
    }
}

experiment out type: gui {
	float minimum_cycle_duration <- 2.0#second;
	output {
		display main_display {
			grid grid_cell       lines: #black;
			species helper_robot aspect: sprite;
			species random_agent aspect: sprite;
		}
	}
	parameter "Number of robots" category: "Robot" var: num_robots init: 1;
	parameter "Location of each robot" category: "Robot" var: robot_locations;
	parameter "Type and number of random agents" category: "Random agents" var: randag_map;
	parameter "Location of each random agent" category: "Random agents" var: randag_locations;
	parameter "Condition of each random agent" category: "Random agents" var: randag_conditions;
}
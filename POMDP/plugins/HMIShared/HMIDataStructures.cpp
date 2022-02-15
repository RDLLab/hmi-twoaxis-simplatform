#include "HMIDataStructures.hpp"

namespace oppt
{

namespace hmi
{

Grid instantiateGrid(std::string &pathToGrid) {

    // Extract the grid details from the plaintext and return them in struct form
    std::string cmd = "cat < " + pathToGrid;
    std::string gridDetails = execute(cmd.c_str());
    // std::cout << "The grid is " << gridDetails << std::endl;
    return Grid(gridDetails);
}

std::vector<TypeAndId> instantiateTypesAndIDs(std::string &pathToRandomAgents) {

    // Get details from the file containing details about all random agents.
    std::string cmd = "cat < " + pathToRandomAgents;
    std::string typesAndIDsDetails = execute(cmd.c_str());
    std::vector<TypeAndId> out;
    // std::cout << "The types and IDs are " << typesAndIDsDetails << std::endl;

    while (!typesAndIDsDetails.empty()) {

        // Extract the type of the current agent from the plaintext data.
        std::string type = typesAndIDsDetails.substr(0, typesAndIDsDetails.find(","));
        // // std::cout << "Type is " << type << std::endl;
        typesAndIDsDetails = typesAndIDsDetails.substr(typesAndIDsDetails.find(",") + 1);

        // Extract the ID of the current agent from the plaintext data.
        int id = std::stoi(typesAndIDsDetails);
        // // std::cout << "ID is " << std::to_string(id) << std::endl;
        typesAndIDsDetails = typesAndIDsDetails.substr(typesAndIDsDetails.find("\n") + 1);

        // Add this to the resulting vector.
        out.push_back(std::make_pair(type, id));
    }
    return out;
}

std::unordered_map<std::string, TransitionMatrix> instantiateTransitionMatrices(std::string &pathToMatrices) {
    std::string cmd = "cat < " + pathToMatrices;
    std::string matricesDetails = execute(cmd.c_str());

    // std::cout << "Matrix details are " << matricesDetails << std::endl;

    // Get the number of conditions from the file
    int numberOfConditions = std::stoi(matricesDetails);

    // Clip this part from the file
    matricesDetails = matricesDetails.substr(matricesDetails.find("\n") + 1);

    std::unordered_map<std::string, TransitionMatrix> typesToMatrices;

    while (!matricesDetails.empty()) {

        // Extract the type of random agent and its corresponding transition matrix from the data, 
        // and add it to the map of types to transition matrices.
        std::string type = matricesDetails.substr(0, matricesDetails.find(","));
        std::string matrixDetails = matricesDetails.substr(0, matricesDetails.find("\n"));
        typesToMatrices.insert(std::pair<std::string, TransitionMatrix>(type, TransitionMatrix(numberOfConditions, matrixDetails)));
        matricesDetails = matricesDetails.substr(matricesDetails.find("\n") + 1);
    }

    return typesToMatrices;
}

std::pair<int, std::string> getShortestPath(const Grid &grid, int x, int y, int destX, int destY) {
    
    // Instantiate data structures for storing which cells are nearby and which paths have
    // already been explored.
    std::cout << "Instantiating data structures..." << std::endl;
    std::set<std::string> explored;
    std::vector<CoordAndPath> frontier = {std::make_pair(Coordinate(x, y), "")};

    while (!frontier.empty()) {

        std::cout << "Getting the first element from the frontier..." << std::endl;
        // Take the first element of the frontier.
        std::vector<CoordAndPath>::iterator frontIt = frontier.begin();
        Coordinate coord = frontIt->first;

        std::cout << "Getting the coordinates and path from this element..." << std::endl;
        // Break down the element's x- and y-coordinates and paths.
        int currentX = coord.getX();
        int currentY = coord.getY();
        std::string path = frontIt->second;

        if (currentX == destX && currentY == destY) {

            // We've found the shortest path to the destination coordinates,
            // so we return the distance and the path.
            std::cout << "Found destination!" << std::endl;
            int distance = path.length();
            return std::make_pair(distance, path);
        }

        // Path can now be considered as explored, so we remove it from the frontier.
        std::cout << "Adding to explored..." << std::endl;
        explored.insert(path);
        frontier.erase(frontIt);

        for (size_t i = 0; i < std::min(MOVES.size(), DIRECTIONS.size()); ++i) {

            // Obtain new coordinates by (hypothetically) moving in the given direction.
            std::cout << "Moving in the direction " << MOVES[i] << std::endl;
            int newX = currentX + DIRECTIONS[i].getX();
            int newY = currentY + DIRECTIONS[i].getY();
            Coordinate newCoord(newX, newY);

            // Conditions to ensure moving in the given direction is actually possible.
            std::cout << "Deciding whether location is in bounds..." << std::endl;
            bool xInBounds = newX > -1 && newX < grid.getWidth();
            bool yInBounds = newY > -1 && newY < grid.getHeight();

            if (xInBounds && yInBounds) {

                bool validCell = grid.getGrid()[newCoord.toPosition(grid)];
                if (!validCell) return std::make_pair(-1, "");

                // Add the new direction to the current path.
                std::cout << "Adding new direction to path..." << std::endl;
                std::string newPath = path + MOVES[i];

                // Create a new iterator to check if this new path already exists in
                // the frontier.
                std::cout << "Checking if coordinate already exists in frontier..." << std::endl;
                std::vector<CoordAndPath>::iterator frontierIt = frontier.begin();

                for ( ; frontierIt != frontier.end(); ++frontierIt) {

                    // If the element already exists, then the iterator points to it.
                    if (frontierIt->first.getX() == newX && frontierIt->first.getY() == newY) break;
                }

                // Use an iterator to determine whether the new path has already been explored.
                std::cout << "Checking if path has already been explored..." << std::endl;
                std::set<std::string>::iterator exploredIt = explored.find(newPath);

                // Determine the total cost of this new path.
                std::cout << "Determining total cost of new path..." << std::endl;
                int pathCost = newPath.length() + abs(destX - newX) + abs(destY - newY);

                // Check whether these coordinates are in the frontier or the path has been explored.
                std::cout << "Checking if coordinates are in frontier or explored..." << std::endl;
                bool notInFrontierOrExplored = frontierIt == frontier.end() && exploredIt == explored.end();

                // If these coordinates are already in the frontier, then check if this new path to these
                // coordinates is shorter than the existing shortest path to them.
                bool lowerPathCostFrontier = frontierIt != frontier.end() && newPath.length() < frontierIt->second.length();

                if (notInFrontierOrExplored || lowerPathCostFrontier) {

                    // If this new path is shorter than the current shortest path in the frontier,
                    // erase the path in the frontier.
                    std::cout << "Checking if new path is shorter than existing one..." << std::endl;
                    if (lowerPathCostFrontier) frontier.erase(frontierIt);

                    // Instantiate an iterator to determine where to place this new path in the frontier
                    // according to its total cost.
                    std::vector<CoordAndPath>::iterator insertIt = frontier.begin();
                    for ( ; insertIt != frontier.end(); ++insertIt) {

                        // Determine the total path cost of the given element in the frontier.
                        int insertXDistance = abs(destX - insertIt->first.getX());
                        int insertYDistance = abs(destY - insertIt->first.getY());
                        int insertPathCost = insertIt->second.length() + insertXDistance + insertYDistance;

                        if (pathCost < insertPathCost) {

                            // The cost of the new path is smaller than the cost of the given element in
                            // the frontier, so we insert the path into the index before the given element
                            // in the frontier.
                            CoordAndPath toAdd(newCoord, newPath);
                            frontier.insert(insertIt, toAdd);
                            break;
                        }
                    }
                }
            }
        }
    }

    // Unsuccessful in finding the shortest path, so return a "null" result.
    std::cout << "Couldn't find a shortest path!" << std::endl;
    return std::make_pair(-1, "");
}

std::string execute(const char * command) {
    std::array<char, 512> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

}
}

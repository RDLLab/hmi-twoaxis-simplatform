#ifndef _HMI_DATA_STRUCTURES_HPP_
#define _HMI_DATA_STRUCTURES_HPP_

#include "oppt/opptCore/core.hpp"

#include <vector>
#include <string>
#include <unordered_map>
#include <set>
#include <array>
#include <memory>
#include <stdio.h>
#include <stdexcept>
#include <cmath>
#include <map>
#include <random>
#include <utility>
#include <algorithm>
#include <iostream>

namespace oppt
{
namespace hmi
{
    
const FloatType BASE_REWARD = 100.0;
const FloatType MAX_REWARD = 10000.0;
const FloatType MIN_REWARD = -1000000000.0;

typedef std::pair<std::string, int> TypeAndId;

const VectorString MOVES = {"N", "E", "S", "W"};

// This struct manages the grid on which the robots and random agents move.
// The underlying structure is a boolean matrix, where the truth of a cell
// is determined by whether it can be traversed by agents or not. Put more
// simply, a cell representing the floor would be "true", while a cell
// representing a wall would be "false".
struct Grid {

    public:

    /**
     * Constructor for the Grid struct. It takes a string representation and 
     * enriches it to be a data structure that is easier to use.
     * 
     * @param gridDetails the string representation of this grid
    **/
    Grid(std::string &gridDetails) {

        std::cout << "Running constructor of Grid()..." << std::endl;

        // Set the width of the grid
        width_ = std::stoi(gridDetails);
        gridDetails = gridDetails.substr(gridDetails.find(",") + 1);

        // Set the height of the grid
        height_ = std::stoi(gridDetails);
        gridDetails = gridDetails.substr(gridDetails.find(",") + 1);

        // Define grid according to width and height provided
        grid_ = std::vector<bool>(width_ * height_);
        for (int i = 0; i < width_ * height_; ++i) {

            // Set whether the current grid square is traversible or not according
            // to the given data
            grid_[i] = gridDetails.at(i) == '_';
        }

        std::cout << "Completed constructor of Grid()..." << std::endl;
    }

    int getWidth() const {
        return width_;
    }

    int getHeight() const {
        return height_;
    }

    std::vector<bool> getGrid() const {
        return grid_;
    }

    Grid() : width_(0), height_(0), grid_() { }

    private:

    // The underlying grid data structure in this struct. It has dimensions
    // `height_` * `width_`.
    std::vector<bool> grid_;

    // The width of the grid.
    int width_;

    // The height of the grid.
    int height_;
};

struct Coordinate {

    public:

    Coordinate(int x, int y) : x_(x), y_(y) { }

    Coordinate(int pos, Grid &grid) {
        x_ = pos % grid.getWidth();
        y_ = pos / grid.getHeight();
    }

    Coordinate() : x_(-1), y_(-1) { }

    int getX() const {
        return x_;
    }

    int getY() const {
        return y_;
    }

    void setX(int x) {
        x_ = x;
    }

    void setY(int y) {
        y_ = y;
    }

    int toPosition(const Grid &grid) {
        return y_ * grid.getWidth() + x_;
    }

    bool operator==(const Coordinate& rCoord) {
        return x_ == rCoord.getX() && y_ == rCoord.getY();
    }

    private:

    int x_;
    int y_;

};

typedef std::pair<Coordinate, std::string> CoordAndPath;
const std::vector<Coordinate> DIRECTIONS = {Coordinate(0, -1),   // north
                                            Coordinate(1, 0),    // east
                                            Coordinate(0, 1),    // south
                                            Coordinate(-1, 0)};  // west

// This struct defines how individual random agents behave according to their type.
// Behaviour is defined as the likelihood of a random agent changing their condition,
// ie. a "happier" agent would have a transition matrix whereby it is more likely to
// transition to a "happy" condition.
struct TransitionMatrix {

    // The underlying matrix data structure of this struct. A given cell in row 'a'
    // and column 'b' denotes the probability of transitioning from condition 'a'
    // to condition 'b'.
    std::vector<std::vector<float>> matrix_;

    // The type of random agent that is associated with this transition matrix, eg.
    // toddlers, elderly people, etc.
    std::string type_;

    /**
     * Constructor for the transition matrix struct. It takes a string representation
     * and enriches it to be a data structure that is easier to understand and use.
     * 
     * @param numberOfConditions the number of conditions covered by transition
     *                           matrices for the given problem. Note that all
     *                           transition matrices must cover the same conditions.
     * 
     * @param matrixDetails      the string representation of this transition matrix
    **/
    TransitionMatrix(int numberOfConditions, std::string &matrixDetails) {
        // std::cout << "Individual matrix details are " << matrixDetails << std::endl;

        // Define to what type of random agent this transition matrix applies
        type_ = matrixDetails.substr(0, matrixDetails.find(","));
        matrixDetails = matrixDetails.substr(matrixDetails.find(",") + 1);

        // Define the matrix according to the given number of conditions. This must
        // be square because it must be defined that an agent can transition from any one
        // condition to another (even if the probability of that is 0.0)
        matrix_ = std::vector<std::vector<float>>(numberOfConditions, std::vector<float>(numberOfConditions));
        for (int fromCondition = 0; fromCondition < numberOfConditions; ++fromCondition) {
            for (int toCondition = 0; toCondition < numberOfConditions; ++toCondition) {

                // Set value of the current matrix cell according to the given data
                matrix_[fromCondition][toCondition] = std::stof(matrixDetails);
                matrixDetails = matrixDetails.substr(matrixDetails.find(",") + 1);
            }
        }
    }

    TransitionMatrix() : type_(""), matrix_() { }
};

/**
 * Instantiates a Grid struct from the path to a plaintext representation of 
 * a grid. See the Readme for details as to how to create a plaintext representation 
 * of a grid.
 * 
 * @param pathToGrid the path to a plaintext representation of a grid
 * 
 * @return           the same information, but in a data structure as opposed to a string
**/
Grid instantiateGrid(std::string &pathToGrid);

std::vector<TypeAndId> instantiateTypesAndIDs(std::string &pathToRandomAgents);

/**
 * Instantiates each random agent type and their corresponding transition matrix. 
 * This data is taken from a plaintext file detailing the number of conditions in 
 * the given problem, as well as each random agent type and their corresponding 
 * transition matrix. See the Readme for details as to how to create a plaintext 
 * representation of a map between random agent types and transition matrices.
 * 
 * @param pathToMatrices the path to a plaintext representation of random agent
 *                       types and their corresponding transition matrices
 * 
 * @result               the same information, but in a data structure as opposed to a string
**/
std::unordered_map<std::string, TransitionMatrix> instantiateTransitionMatrices(std::string &pathToMatrices);

/**
 * Standard A* algorithm to find the shortest path from a given starting pair of x- and y-coordinates
 * to a given destination pair of x- and y-coordinates on a given grid.
 * 
 * @param grid  the grid on which to determine the shortest path
 * @param x     the starting x-coordinate
 * @param y     the starting y-coordinate
 * @param destX the destination x-coordinate
 * @param destY the destination y-coordinate
 * 
 * @return      a pair containing the length and details of the shortest path from the starting
 *              coordinates to the destination coordinates. The shortest path is represented by
 *              a string of 'N' (north), 'S' (south), 'E' (east) and 'W' (west) characters. If
 *              there is no shortest path, this method will return the pair (-1, "").
**/
std::pair<int, std::string> getShortestPath(const Grid &grid, int x, int y, int destX, int destY);

/**
 * Runs the given command in the Terminal and captures the output buffer. 
 * For the purposes of this program, this is mostly to capture the output from 
 * `cat` commands. The implementation code for this method was obtained from 
 * Stack Overflow.
 * 
 * @param command the command to be executed in the Terminal
 * 
 * @return        the buffer output from the Terminal.
**/
std::string execute(const char * command);
}
}

#endif
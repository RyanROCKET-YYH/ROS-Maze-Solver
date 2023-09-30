/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Yuhong YAO
 * ANDREW ID: yuhongy
 * LAST UPDATE: 9/23/2023
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the left-hand rule. The code is intentionaly left obfuscated.
 *
 */

#include "student.h"

// enum for turtle's current state
enum TurtleState {				 
	turned_bumped = 0,
	turned_forward = 1,
	moving_forward = 2,
};

// enum represent turn direction
enum TurnDirection {
	left = -1,
	right = 1,
};

/**
 * @brief Get the next direction for the turtle based on its current orientation and intended turn direction.
 *
 * @param nw_or Current orientation of the turtle.
 * @param turn Direction turtle intends to turn: either left or right.
 * @return TurtleOrientation representing the new orientation after turning.
 */
TurtleOrientation getNextDir(int32_t nw_or, TurnDirection turn){
	int32_t cycle = 4;
	int32_t nextDir = (nw_or + turn + cycle)%4;
	return static_cast<TurtleOrientation>(nextDir);
}

/**
 * @brief Updates the turtle's state and orientation using the wall-following rule.
 * 
 * If the turtle is currently moving forward, it will turn left. If it bumped into a wall, it will turn right.
 * Otherwise, it will continue moving forward.
 *
 * @param nw_or Current orientation of the turtle in local map. 
 * @param cs Current state of the turtle.
 * @param bumped Check if bumped.
 */
void TurtleStateUpdate(int32_t &nw_or, TurtleState &cs, bool bumped){
	if (cs == moving_forward){
		nw_or = getNextDir(nw_or, left);
		cs = turned_forward;
	} else if (bumped){
		nw_or = getNextDir(nw_or, right);
		cs = turned_bumped;
	} else {
		cs = moving_forward;
	}
}

/**
 * @brief Determines the next move for the turtle based on its current state, whether it bumped into a wall, and whether it reached the end of the maze.
 * 
 * @param bumped Check if bumped.
 * @param atend Check Goal.
 * @return Returns a turtleResult structure containing the next move for the turtle and the number of visits to the current cell
 */
turtleResult studentTurtleStep(bool bumped, bool atend) {
	// Local map to keep track of number of visits for each cell
	static int8_t localMap[23][23] = {0};
	// Starting position of the turtle		
	static int8_t localX = 11;
	static int8_t localY = 11;
	// Current orientation of the turtle
	static int32_t nw_or = north;
	// Current state of the turtle
	static TurtleState cs = moving_forward;
	ROS_INFO("Current state: %d", cs);
	turtleResult result;
	// If the turtle GOAL, stop and return the number of visits
	if (atend) {
		result.nextMove = STOP;
		result.visits = localMap[localX][localY];
		return result;
	}
	// Update the turtle's state
	TurtleStateUpdate(nw_or, cs, bumped);
	// Determine the next move and update the turtle's position and orientation based on its current state
	switch (cs) {
		case moving_forward:
			result.nextMove = MOVE;
			switch (nw_or) {
                case north:
                    localY--;
                    break;
                case east:
                    localX++;
                    break;
                case south:
                    localY++;
                    break;
                case west:
                    localX--;
                    break;
				default:
					ROS_ERROR("Unexpected value for turtle's direction: %d", nw_or);
					break;
            }
			// Increment the number of visits for the current cell
			localMap[localX][localY]++;
			break;
		case turned_forward:
			result.nextMove = TURN_LEFT;
			break;
		case turned_bumped:
			result.nextMove = TURN_RIGHT;
			break;
		default:
			ROS_ERROR("Unexpected value for turtle's state: %d", cs);
			break;
	}
	result.visits = localMap[localX][localY];
	return result;
 }
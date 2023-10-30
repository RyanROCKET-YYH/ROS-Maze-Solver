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
	moved = 0,
	CheckRight = 1,
	CheckFront = 2,
	CheckLeft = 3,
	ReadyForWallCheck = 4,
	DecideNextMove = 5,
	leftTwice = 6,
	leftOnce = 7,
	moving_forward = 8,
	Goal = 9,
	Initialized = 10,
	TurnAround = 11,
	DecisionMade = 12,
	CheckAlldirection = 13,
};

// enum represent turn direction
enum TurnDirection {
	left = -1,
	right = 1,
};

/**
 * @brief Get the next direction for the turtle based on its current orientation and intended turn direction.
 *
 * @param direction Current orientation of the turtle.
 * @param turn Direction turtle intends to turn: either left or right.
 * @return TurtleOrientation representing the new orientation after turning.
 */
TurtleOrientation getNextDir(TurtleOrientation direction, TurnDirection turn){
	int32_t cycle = 4;
	int32_t nextDir = (direction + turn + cycle)%4;
	return static_cast<TurtleOrientation>(nextDir);
}

void WallUpdate(TurtleOrientation &direction, int8_t (&localMap)[23][23], int8_t x, int8_t y, bool bumped) {
	if (!bumped) {
		switch (direction) {
			case north:
				localMap[x][y] &= ~0b0001;
				break;
			case east:
				localMap[x][y] &= ~0b0010;
				break;
			case south:
				localMap[x][y] &= ~0b0100;
				break;
			case west:
				localMap[x][y] &= ~0b1000;
				break;
			default:
				ROS_ERROR("Invalid orientation");
				break;
		}
	}
}

void ComingDirection_No_Wall(TurtleOrientation &direction, int8_t (&localMap)[23][23], int8_t x, int8_t y) {
	switch (direction) {
			case north:
				localMap[x][y] &= ~0b0100;
				break;
			case east:
				localMap[x][y] &= ~0b1000;
				break;
			case south:
				localMap[x][y] &= ~0b0001;
				break;
			case west:
				localMap[x][y] &= ~0b0010;
				break;
			default:
				ROS_ERROR("Invalid orientation");
				break;
		}
}

struct DirectionVisitCounts {
	int32_t northCount;
	int32_t eastCount;
	int32_t southCount;
	int32_t westCount;
};

DirectionVisitCounts get_visitCounts(int8_t x, int8_t y, int8_t (&localMap)[23][23], int32_t (&visitCounts)[23][23], DirectionVisitCounts counts) {

	counts.northCount = (localMap[x][y] & 0b0001) ? INT32_MAX : visitCounts[x][y-1];
    counts.eastCount  = (localMap[x][y] & 0b0010) ? INT32_MAX : visitCounts[x+1][y];
    counts.southCount = (localMap[x][y] & 0b0100) ? INT32_MAX : visitCounts[x][y+1];
    counts.westCount  = (localMap[x][y] & 0b1000) ? INT32_MAX : visitCounts[x-1][y];
	return counts;
}

TurtleOrientation get_lstVisitedDir (TurtleOrientation currentDirection, DirectionVisitCounts counts) {
	TurtleOrientation priorityOrder[4][3] = {
		{east, north, west, south},  // Current: north
        {south, east, north, west},  // Current: east
        {west, south, east, north},  // Current: south
        {north, west, south, east}   // Current: west
    };

	int32_t directionCounts[4] = {
		counts.northCount,
		counts.eastCount,
		counts.southCount,
		counts.westCount
	};

	int32_t minCount = directionCounts[0];
	for (int8_t i = 1; i < 4; i++) {
		if (directionCounts[i] < minCount) {
			minCount = directionCounts[i];
		}
	}

	for (int8_t i = 0; i < 4; i++) {
		TurtleOrientation desiredDirection = priorityOrder[currentDirection][i];
		if (directionCounts[desiredDirection] == minCount && directionCounts[desiredDirection] != INT32_MAX) {
			return desiredDirection;
		}
	}

	// error handling
	ROS_ERROR("Invalid direction");
	return currentDirection;
}

turtleMove get_nextMove(TurtleOrientation currentDirection, TurtleOrientation desiredDirection, TurtleState &cs) {
	switch(currentDirection) {
		case north:
			switch(desiredDirection) {
				case north:
					cs = moving_forward;
					return MOVE;
				case east:
					cs = TurnAround;
					return TURN_RIGHT;
				case south:
					cs = leftTwice;
					return TURN_LEFT;
				case west:
					cs = leftOnce;
					return TURN_LEFT;
				default:
					ROS_ERROR("Invalid direction");
					break;
			}
		case east:
			switch(desiredDirection) {
				case north:
					cs = leftOnce;
					return TURN_LEFT;
				case east:
					cs = moving_forward;
					return MOVE;
				case south:
					cs = TurnAround;
					return TURN_RIGHT;
				case west:
					cs = leftTwice;
					return TURN_LEFT;
				default:
					ROS_ERROR("Invalid direction");
					break;
			}
		case south:
			switch(desiredDirection) {
				case north:
					cs = TurnAround;
					return TURN_RIGHT;
				case east:
					cs = leftOnce;
					return TURN_LEFT;
				case south:
					cs = moving_forward;
					return MOVE;
				case west:
					cs = leftTwice;
					return TURN_LEFT;
				default:
					ROS_ERROR("Invalid direction");
					break;
			}
		case west:
			switch(desiredDirection) {
				case north:
					cs = leftTwice;
					return TURN_LEFT;
				case east:
					cs = TurnAround;
					return TURN_RIGHT;
				case south:
					cs = leftOnce;
					return TURN_LEFT;
				case west:
					cs = moving_forward;
					return MOVE;
				default:
					ROS_ERROR("Invalid direction");
					break;
			}
		default:
			ROS_ERROR("Invalid direction");
			break;
	}
}

void printLocalMapCell(int8_t (&localMap)[23][23], int8_t x, int8_t y) {
    printf("localMap[%d][%d]: %x\n", x, y, localMap[x][y]);
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
	static int32_t visitCounts[23][23] = {0};
	static int8_t localMap[23][23] = {0b1111};
	// Starting position of the turtle		
	static int8_t localX = 11;
	static int8_t localY = 11;
	printLocalMapCell(localMap, localX, localY);
	// Current orientation of the turtle
	static TurtleOrientation direction = north;
	// Current state of the turtle
	static TurtleState cs = Initialized;
	ROS_INFO("Current state: %d", cs);
	turtleResult result;
	static DirectionVisitCounts counts = {-1,-1,-1,-1};
	static TurtleOrientation desiredDirection = north;
	// If the turtle GOAL, stop and return the number of visits
	switch (cs) {
		case Initialized:
			visitCounts[localX][localY]++;
			cs = CheckAlldirection;
			direction = getNextDir(direction, left);
			result.nextMove = TURN_LEFT;
			result.visits = visitCounts[localX][localY];
			break;
		case CheckAlldirection:
			WallUpdate(direction, localMap, localX, localY, bumped);
			direction = getNextDir(direction, right);
			if (direction == west) {
				cs = DecideNextMove;
				result.nextMove = STOP;
				result.visits = visitCounts[localX][localY];
				break;
			}
			result.nextMove = TURN_RIGHT;
			result.visits = visitCounts[localX][localY];
			break;
		case moved:
			visitCounts[localX][localY]++;
			if (visitCounts[localX][localY] == 1 && !atend) {
				ComingDirection_No_Wall(direction, localMap, localX, localY);
				cs = ReadyForWallCheck;
				direction = getNextDir(direction, left);
				result.nextMove = TURN_LEFT;
				result.visits = visitCounts[localX][localY];
				break;
			} else if (!atend && visitCounts[localX][localY] > 1) {
				cs = DecideNextMove;
				result.nextMove = STOP;
				result.visits = visitCounts[localX][localY];
				break;
			} else if (atend) {
				cs = Goal;
				result.nextMove = STOP;
				result.visits = visitCounts[localX][localY];
				break;
			}
		case ReadyForWallCheck:
			cs = CheckLeft;
			WallUpdate(direction, localMap, localX, localY, bumped);
			direction = getNextDir(direction, right);
			result.nextMove = TURN_RIGHT;
			result.visits = visitCounts[localX][localY];
			break;
		case CheckLeft:
			cs = CheckFront;
			WallUpdate(direction, localMap, localX, localY, bumped);
			direction = getNextDir(direction, right);
			result.nextMove = TURN_RIGHT;
			result.visits = visitCounts[localX][localY];
			break;
		case CheckFront:
			cs = CheckRight;
			WallUpdate(direction, localMap, localX, localY, bumped);
			direction = getNextDir(direction, right);
			result.nextMove = TURN_RIGHT;
			result.visits = visitCounts[localX][localY];
			break;
		case CheckRight:
			cs = DecideNextMove;
			WallUpdate(direction, localMap, localX, localY, bumped);
			result.nextMove = STOP;
			result.visits = visitCounts[localX][localY];
			break;
		case DecideNextMove:
			counts = get_visitCounts(localX, localY, localMap, visitCounts, counts);
			desiredDirection = get_lstVisitedDir(direction, counts);
			cs = DecisionMade;
			result.nextMove = STOP;
			result.visits = visitCounts[localX][localY];
			break;
		case DecisionMade:
			result.nextMove = get_nextMove(direction, desiredDirection, cs);
			result.visits = visitCounts[localX][localY];
			break;
		case leftTwice:
			direction = getNextDir(direction, left);
			result.nextMove = TURN_LEFT;
			result.visits = visitCounts[localX][localY];
			cs = leftOnce;
			break;
		case leftOnce:
			direction = getNextDir(direction, left);
			result.nextMove = TURN_LEFT;
			result.visits = visitCounts[localX][localY];
			cs = moving_forward;
			break;
		case TurnAround: 
			direction = getNextDir(direction, right);
			result.nextMove = TURN_RIGHT;
			result.visits = visitCounts[localX][localY];
			cs = moving_forward;
			break;
		case moving_forward: 
			switch (direction) {
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
					ROS_ERROR("Invalid orientation");
					break;
			}
			result.nextMove = MOVE;
			result.visits = visitCounts[localX][localY];
			cs = moved;
			break;
		default:
			ROS_ERROR("Invalid state");
			break;
	}
	return result;
 }
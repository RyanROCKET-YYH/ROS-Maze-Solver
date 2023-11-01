/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Yuhong YAO
 * ANDREW ID: yuhongy
 * LAST UPDATE: 10/31/2023
 *
 * This file is an algorithm to solve the ece642rtle maze
 *
 */

#include "student.h"

// enum for turtle's current state
enum TurtleState {				 
	Initialized,
	CheckWall,
	Right,
	DecideNextMove,
	Move,
	leftOnce,
	leftTwice,
	rightOnce,
	Goal,
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

void WallUpdate(TurtleOrientation direction, int32_t (&localMap)[23][23], int32_t x, int32_t y, bool bumped) {
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

TurtleOrientation NextMove(TurtleOrientation currentDir, int32_t (&visitCounts)[23][23], int32_t (&localMap)[23][23], int32_t x, int32_t y) {
	int32_t northCount = visitCounts[x][y-1];
	int32_t eastCount = visitCounts[x+1][y];
	int32_t southCount = visitCounts[x][y+1];
	int32_t westCount = visitCounts[x-1][y];

	int32_t walls = localMap[x][y];

	TurtleOrientation priorityOrder[4][4] = {
		{east, north, west, south},  // Current: NORTH
        {south, east, north, west}, // Current: EAST
        {west, south, east, north},  // Current: SOUTH
        {north, west, south, east}  // Current: WEST
	};

	int32_t minvisitCount = INT32_MAX;
	TurtleOrientation nextDir = error;
	for (int8_t i = 0; i < 4; i++) {
		TurtleOrientation direction = priorityOrder[currentDir][i];
		int32_t count = 0;
		bool hasWall = false;
		switch (direction) {
			case north:
				count = northCount;
				hasWall = ((walls & 0b0001) != 0);
				break;
			case east:
				count = eastCount;
				hasWall = ((walls & 0b0010) != 0);
				break;
			case south:
				count = southCount;
				hasWall = ((walls & 0b0100) != 0);
				break;
			case west:
				count = westCount;
				hasWall = ((walls & 0b1000) != 0);
				break;
			default:
				ROS_ERROR("Invalid orientation");
				break;
		}
		if (!hasWall && count <= minvisitCount) {
			minvisitCount = count;
			nextDir = direction;
		}
	}
	return nextDir;
}

int8_t getTurns(TurtleOrientation currentDir, TurtleOrientation desiredDir) {
	int8_t cycle = 4;
	int8_t difference = (desiredDir - currentDir + cycle) % cycle;
	return difference;
}

void printLocalMapCell(int32_t (&localMap)[23][23], int32_t x, int32_t y) {
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
	static int32_t localMap[23][23];
	static bool isInitialized = false;
	if (!isInitialized) {
		for (int i = 0; i < 23; ++i) {
			for (int j = 0; j < 23; ++j) {
				localMap[i][j] = 0b1111;
			}
		}
		isInitialized = true;
	}
	// Starting position of the turtle		
	static int32_t localX = 11;
	static int32_t localY = 11;
	printLocalMapCell(localMap, localX, localY);
	// Current orientation of the turtle
	static TurtleOrientation direction = north;
	// Current state of the turtle
	static TurtleState cs = Initialized;
	static int8_t spinCounter = 0;
	static int8_t turns = -1;
	TurtleOrientation desiredDir = error;
	ROS_INFO("Current state: %d", cs);
	turtleResult result;
	result.nextMove = STOP;
	// If the turtle GOAL, stop and return the number of visits
	switch (cs) {
		case Initialized:   // S1. Initialized
			visitCounts[localX][localY]++;
			cs = CheckWall;
			break;
	
		case CheckWall: // S3. CheckWall
			WallUpdate(direction, localMap, localX, localY, bumped);
			if (visitCounts[localX][localY] == 1 && spinCounter < 3) {
				direction = getNextDir(direction, right);
				cs = Right;
			} else if (spinCounter == 3) {
				cs = DecideNextMove;
			}
			break;
	
		case Right: // S2. Right
			direction = getNextDir(direction, right);
			spinCounter += 1;
			result.nextMove = TURN_RIGHT;
			if (spinCounter <= 3) {
				cs = CheckWall;
			}
			break;

		case DecideNextMove: // S4. DecideNextMove
			desiredDir = NextMove(direction, visitCounts, localMap, localX, localY);
			turns = getTurns(direction, desiredDir);
			if (desiredDir != -1) {
				switch (turns) {
					case 0:
						cs = Move;
						break;
					case 1:
						cs = leftOnce;
						break;
					case 2:
						cs = leftTwice;
						break;
					case 3:
						cs = rightOnce;
						break;
					default:
						ROS_ERROR("ERROR WHEN DECIDE NEXT MOVE");
						break;
				}
			}
			break;

		case leftTwice: // S7. leftTwice
			spinCounter = 2;
			direction = getNextDir(direction, left);
			result.nextMove = TURN_LEFT;
			spinCounter -= 1;
			cs = leftOnce;
			break;

		case leftOnce: // S6. leftOnce
			spinCounter = 1;
			direction = getNextDir(direction, left);
			result.nextMove = TURN_LEFT;
			spinCounter -= 1;
			cs = Move;
			break;

		case rightOnce: // S8. RightOnce
			spinCounter = 1;
			direction = getNextDir(direction, right);
			result.nextMove = TURN_RIGHT;
			spinCounter -= 1;
			cs = Move;
			break;

		case Move: // S5. Move
			spinCounter = 0;
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
					ROS_ERROR("Invalid orientation while moving");
					break;
			}
			visitCounts[localX][localY]++;
			result.nextMove = MOVE;
			if (atend) {
				cs = Goal;
			} else if (visitCounts[localX][localY] != 1 && localMap[localX][localY] != 0b1111) {
				cs = DecideNextMove;
			} else if (visitCounts[localX][localY] == 1) {
				cs = CheckWall;
			}
			break;

		case Goal: // S9. Goal
			result.nextMove = STOP;
			break;

		default:
			ROS_ERROR("Invalid state");
			break;
	}
	result.visits = visitCounts[localX][localY];
	return result;
 }
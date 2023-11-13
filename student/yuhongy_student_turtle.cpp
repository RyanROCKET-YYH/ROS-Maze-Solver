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

#ifdef testing
#include "yuhongy_student_mock.h"
#endif
#ifndef testing
#include "student.h"
#endif


static int32_t visitCounts[23][23];
static int32_t localMap[23][23];
static int8_t spinCounter = 0;
static TurtleOrientation desiredDir = error;

// enum for turtle's current state
// enum TurtleState {				 
// 	Initialized,
// 	CheckWall,
// 	Right,
// 	DecideNextMove,
// 	Move,
// 	leftOnce,
// 	leftTwice,
// 	rightOnce, // unnecessary
// 	Goal,
// };

// Current state of the turtle
static TurtleState cs = Initialized;

static TurtleOrientation direction = north;

#ifdef testing
int32_t getVisitCounts(int32_t x, int32_t y) {
    return visitCounts[x][y];
}

void setVisitCounts(int32_t x, int32_t y, int32_t count) {
    visitCounts[x][y] = count;
}

int32_t getLocalMap(int32_t x, int32_t y) {
    return localMap[x][y];
}

void setLocalMap(int32_t x, int32_t y, int32_t value) {
    localMap[x][y] = value;
}

TurtleState getTurtleState() {
	return cs;
}

void setTurtleState(TurtleState state) {
	cs = state;
}

TurtleOrientation getTurtleOrientation() {
	return direction;
}

void setTurtleOrientation(TurtleOrientation dir) {
	direction = dir;
}

int8_t getSpinCounter() {
	return spinCounter;
}

void setSpinCounter(int8_t count) {
	spinCounter = count;
}

TurtleOrientation mock_desiredDir = error;
TurtleOrientation getMockDesiredDir() {
	return mock_desiredDir;
}

void setMockDesiredDir(TurtleOrientation dir) {
	mock_desiredDir = dir;
}

TurtleOrientation getDesiredDir() {
	return desiredDir;
}

int32_t mock_localX;
int32_t mock_localY;
void setMockLocalCord(int32_t x, int32_t y) {
	mock_localX = x;
	mock_localY = y;
}

int32_t getMockLocalX() {
	return mock_localX;
}

int32_t getMockLocalY() {
	return mock_localY;
}
#endif

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

/**
 * @brief update the local map based on the turtle's current position and orientation.
 *
 * @param direction Current orientation of the turtle.
 * @param localMap turtle's localMap to store wall info.
 * @param x, y turtle's current coordinate
 * @param bumped check if bump
 */
void WallUpdate(TurtleOrientation direction, int32_t (&localMap)[23][23], int32_t x, int32_t y, bool bumped) {
	if (!bumped) {
		switch (direction) {
			case north:
				localMap[x][y] &= ~0x01;
				break;
			case east:
				localMap[x][y] &= ~0x02;
				break;
			case south:
				localMap[x][y] &= ~0x04;
				break;
			case west:
				localMap[x][y] &= ~0x08;
				break;
			default:
				ROS_ERROR("Invalid orientation");
				break;
		}
	}
}

/**
 * @brief decide the next move based on the lowest visits and bump condition
 *
 * @param currentDir Current orientation of the turtle.
 * @param visitCounts Turtle's visit count array
 * @param localMap turtle's localMap to store wall info.
 * @param x, y turtle's current coordinate
 * @return desiredDir
 */
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
				hasWall = ((walls & 0x01) != 0);
				break;
			case east:
				count = eastCount;
				hasWall = ((walls & 0x02) != 0);
				break;
			case south:
				count = southCount;
				hasWall = ((walls & 0x04) != 0);
				break;
			case west:
				count = westCount;
				hasWall = ((walls & 0x08) != 0);
				break;
			default:
				ROS_ERROR("Invalid orientation");
				break;
		}
		if (!hasWall && count < minvisitCount) {
			minvisitCount = count;
			nextDir = direction;
		}
		// ROS_INFO("Direction: %d, Count: %d, Has Wall: %d, Min Visit Count: %d, Next Dir: %d, CurrentDir: %d",
        //     direction, count, hasWall, minvisitCount, nextDir, currentDir);
	}
	return nextDir;
}

/**
 * @brief get the number of turns required to reach the desired direction from the current direction
 * 
 * @param currentDir Current orientation of the turtle.
 * @param desiredDir Turtle's desired direction
 * 
 * @return turns turtle needs to make
 */
int8_t getTurns(TurtleOrientation currentDir, TurtleOrientation desiredDir) {
	int8_t cycle = 4;
	int8_t difference = static_cast<int8_t>((desiredDir - currentDir + cycle) % cycle);
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
	visitCounts[23][23] = {0};
	static bool isInitialized = false;
	if (!isInitialized) {
		for (int i = 0; i < 23; ++i) {
			for (int j = 0; j < 23; ++j) {
				localMap[i][j] = 0x0F;
			}
		}
		isInitialized = true;
	}
	// Starting position of the turtle		
	static int32_t localX = 11;
	static int32_t localY = 11;
	// printLocalMapCell(localMap, localX, localY);

	// Current orientation of the turtle
	// static TurtleOrientation direction = north;

	// Current state of the turtle
	// static TurtleState cs = Initialized;

	// static int8_t spinCounter = 0;

	static int8_t turns = -1;
	desiredDir = error;
	// ROS_INFO("Current state: %d", cs);
	turtleResult result;
	result.nextMove = STOP;
	// If the turtle GOAL, stop and return the number of visits
	switch (cs) {
		case Initialized:   // S1. Initialized
			#ifndef testing
			visitCounts[localX][localY]++;
			#endif
			#ifdef testing
			visitCounts[mock_localX][mock_localY]++;
			#endif
			cs = CheckWall; // transition. True (S1->S3)
			break;
	
		case CheckWall: // S3. CheckWall
			WallUpdate(direction, localMap, localX, localY, bumped);
			// printLocalMapCell(localMap, localX, localY);
			if (atend) {   // transition: Atend (S3->S9)
				cs = Goal;
				break;
			}
			if (spinCounter < 3) {   // transition: spinCounter < 3 (S3->S2)
				cs = Right;
			} else if (spinCounter == 3) {  // transition: spinCounter == 3 (S3->S4)
				cs = DecideNextMove;
			}
			break;
	
		case Right: // S2. Right
			direction = getNextDir(direction, right);
			spinCounter = static_cast<int8_t>(spinCounter + 1);
			result.nextMove = TURN_RIGHT;
			if (spinCounter <= 3) { // transitionL spinCounter <=3 (S2->S3)
				cs = CheckWall;
			}
			break;

		case DecideNextMove: // S4. DecideNextMove
			if (atend) {  // transition: Atend (S4->S9)
				cs = Goal;
				break;
			}
			desiredDir = NextMove(direction, visitCounts, localMap, localX, localY);
			#ifdef testing
			if (mock_desiredDir != error) {
				desiredDir = getMockDesiredDir();
				direction = getTurtleOrientation();
			}
			#endif
			turns = getTurns(direction, desiredDir);
			if (desiredDir != -1) { // transition: desiredDir != -1 (S4->S5,S8,S7,S6)
				switch (turns) {
					case 0:		// transition: direction - desiredDir == 0 (S4->S5)
						cs = Move;
						break;
					case 1:    // transition: direction - desiredDir == 1 (S4->S8)
						cs = rightOnce;
						break;
					case 2:   // transition: direction - desiredDir == 2 (S4->S7)
						cs = leftTwice;
						break;
					case 3:  // transition: direction - desiredDir == 3 (S4->S6)
						cs = leftOnce;
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
			spinCounter = static_cast<int8_t>(spinCounter - 1);
			cs = leftOnce; // transion: Ture (S7->S6)
			break;

		case leftOnce: // S6. leftOnce
			spinCounter = 1;
			direction = getNextDir(direction, left);
			result.nextMove = TURN_LEFT;
			spinCounter = static_cast<int8_t>(spinCounter - 1);
			cs = Move; // transion: Ture (S6->S5)
			break;

		case rightOnce: // S8. RightOnce
			spinCounter = 1;
			direction = getNextDir(direction, right);
			result.nextMove = TURN_RIGHT;
			spinCounter = static_cast<int8_t>(spinCounter - 1);
			cs = Move; // transion: Ture (S7->S6)
			break;

		case Move: // S5. Move
			spinCounter = 0;
			desiredDir = error;
			switch (direction) {
				case north:
					localY--;
					#ifdef testing
					mock_localY--;
					#endif
					break;
				case east:
					localX++;
					#ifdef testing
					mock_localX++;
					#endif
					break;
				case south:
					localY++;
					#ifdef testing
					mock_localY++;
					#endif
					break;
				case west:
					localX--;
					#ifdef testing
					mock_localX--;
					#endif
					break;
				default:
					ROS_ERROR("Invalid orientation while moving");
					break;
			}
			visitCounts[localX][localY]++;
			result.nextMove = MOVE;
			#ifdef testing
			visitCounts[mock_localX][mock_localY]++;
			if (atend) {  // transion: Atend (S5->S9)
				cs = Goal;
			} else if (visitCounts[mock_localX][mock_localY] != 1 && localMap[mock_localX][mock_localY] != 0x0F) {  // transion: visitCounts > 1 && localMap != 0b1111  (S5->S4)
				cs = DecideNextMove;
			} else if (visitCounts[mock_localX][mock_localY] == 1) {  // transion: visitCounts == 1 && localMap == 0b1111  (S5->S3)
				cs = CheckWall;
			}
			#endif

			#ifndef testing
			if (atend) {  // transion: Atend (S5->S9)
				cs = Goal;
			} else if (visitCounts[localX][localY] != 1 && localMap[localX][localY] != 0x0F) {  // transion: visitCounts > 1 && localMap != 0b1111  (S5->S4)
				cs = DecideNextMove;
			} else if (visitCounts[localX][localY] == 1) {  // transion: visitCounts == 1 && localMap == 0b1111  (S5->S3)
				cs = CheckWall;
			}
			#endif

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

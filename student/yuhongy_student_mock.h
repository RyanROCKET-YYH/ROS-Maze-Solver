/*
 * 18-642 Unit Testing 
 * @author Yuhong YAO 
 * @AndrewID: yuhongy
 */


#include <iostream>
#include <cstdint>

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

enum TurtleOrientation {
	north = 0,
	east = 1,
	south = 2,
	west = 3,
	error = -1
};

enum TurnDirection {
	left = -1,
	right = 1,
};

enum turtleMove {MOVE, TURN_LEFT, TURN_RIGHT, STOP};

struct turtleResult {
    turtleMove nextMove;
    int32_t visits;
};

TurtleOrientation getNextDir(TurtleOrientation direction, TurnDirection turn);
void WallUpdate(TurtleOrientation direction, int32_t (&localMap)[23][23], int32_t x, int32_t y, bool bumped);
TurtleOrientation NextMove(TurtleOrientation currentDir, int32_t (&visitCounts)[23][23], int32_t (&localMap)[23][23], int32_t x, int32_t y);
int8_t getTurns(TurtleOrientation currentDir, TurtleOrientation desiredDir);
turtleResult studentTurtleStep(bool bumped, bool atend);

// Mock student_maze functions

bool will_bump();
bool at_end();

// Functions called by main testing program to get or set values
TurtleOrientation test_orientation_result();
TurtleOrientation test_NextMove_result();

int32_t getVisitCounts(int32_t x, int32_t y);
int32_t getLocalMap(int32_t x, int32_t y);
void test_WallUpdate();
void test_getTurns();
void test_studentTurtleStep();
void mock_set_bump(bool bump);
void mock_set_atend(bool atend);

// get and set functions for static variables
int32_t getVisitCounts(int32_t x, int32_t y);
void setVisitCounts(int32_t x, int32_t y, int32_t value);
int32_t getLocalMap(int32_t x, int32_t y);
void setLocalMap(int32_t x, int32_t y, int32_t value);
TurtleState getTurtleState();
void setTurtleState(TurtleState state);
TurtleOrientation getTurtleOrientation();
void setTurtleOrientation(TurtleOrientation dir);
int8_t getSpinCounter();
void setSpinCounter(int8_t count);

void ROS_ERROR(std::string e);
void ROS_INFO(const char* format, ...);
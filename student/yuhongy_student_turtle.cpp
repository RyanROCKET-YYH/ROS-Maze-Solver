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

// Ignore this line until project 5
turtleMove studentTurtleStep(bool bumped) { return MOVE; }

// OK TO MODIFY BELOW THIS LINE
// this procedure takes the current turtle position and orientation and returns
// true=submit changes, false=do not submit changes
// Ground rule -- you are only allowed to call the helper functions "bumped(..)"
// and "atend(..)", and NO other turtle methods or maze methods (no peeking at
// the maze!)

// Define the Coordinate type
typedef int32_t Cord;			
struct Point2D {
	Cord x;
	Cord y;
};

// left hand orientation enum
enum TurtleOrientation {
	north = 0,
	east = 1,
	south = 2,
	west = 3,
};

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
 * @param nw_or Current orientation of the turtle. This will be updated based on the wall-following rule.
 * @param cs Current state of the turtle. This will be updated based on the wall-following rule.
 * @param bumped Boolean indicating if the turtle bumped into a wall.
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
 * @brief Update the start position of the turtle based on its current position.
 * 
 * @param pos_ Current position of the turtle.
 * @param startPoint Reference to the starting point to be updated.
 */
void updateStartPosition(QPointF &pos_, Point2D &startPoint) {
    startPoint.x = pos_.x();
    startPoint.y = pos_.y();
}

/**
 * @brief Update the end position of the turtle based on its current position and orientation.
 * 
 * @param pos_ Current position of the turtle.
 * @param nw_or Current orientation of the turtle.
 * @param endPoint Reference to the ending point to be updated.
 * @param startPoint Reference to the starting point, used for certain orientations.
 */
void updateEndPosition(QPointF &pos_, int32_t nw_or, Point2D &endPoint, Point2D &startPoint) {
    endPoint.x = pos_.x();
    endPoint.y = pos_.y();
    
    if (nw_or == north || nw_or == east) {
        nw_or == north ? endPoint.y++ : endPoint.x++;
    } else {
        endPoint.x++;
        endPoint.y++;
        nw_or == south ? startPoint.x++ : startPoint.y++;
    }
}

/**
 * @brief Update turtle's coordinate in the maze.
 * 
 * @param pos_ Current position of the turtle, which will be updated based on the orientation.
 * @param nw_or Current orientation of the turtle.
 */
void moveTurtleBasedOnOrientation(QPointF &pos_, int32_t nw_or, int8_t &localX, int8_t &localY) {
    switch (nw_or) {
        case east:
            pos_.setY(--pos_.ry());
			localX++;
            break;
        case south:
            pos_.setX(++pos_.rx());
			localY++;
            break;
        case west:
            pos_.setY(++pos_.ry());
			localX--;
            break;
        case north:
            pos_.setX(--pos_.rx());
			localY--;
            break;
        default:
            ROS_ERROR("Unexpected value for turtle's direction: %d", nw_or);
            break;
    }
}

/**
 * @brief Moves the turtle in the maze based on wall following rule. (either left-hand or right-hand)
 *        The function determines the turtle's next position and orientation.
 * 
 * @param pos_ Current position of the turtle.
 * @param nw_or Current orientation of the turtle.
 * 
 * @return true if the changes to position and submit the orientation, false otherwise.
 */

bool studentMoveTurtle(QPointF &pos_, int &nw_or) {    
	static int32_t wait;
	static TurtleState cs;			// current state of turtle
	const int32_t TIMEOUT = 2; 	 	// bigger number slows down simulation so you can see what's happening
	// Local map to keep track of number of visits for each square
	static int8_t localMap[23][23] = {0};
	// Starting position of the turtle		
	static int8_t localX = 11;
	static int8_t localY = 11;

	ROS_INFO("Turtle update Called  w=%d", wait);
	bool aend, moving_flag;
  	if (!wait) {
		Point2D startPoint, endPoint;
		// update start and end position so that we can check if bumped
    	updateStartPosition(pos_, startPoint);
        updateEndPosition(pos_, nw_or, endPoint, startPoint);

		bool bp = bumped(startPoint.x, startPoint.y, endPoint.x, endPoint.y);  // if there is a bump (boolean)
		aend = atend(pos_.x(), pos_.y()); 									   // if arrvies at end (boolean)

		ROS_INFO("Current state: %d, Orientation: %d", cs, nw_or);

		// left hand rule to update turtle's state and orientation for further pos changes
		TurtleStateUpdate(nw_or, cs, bp);

		ROS_INFO("Orientation=%d  STATE=%d", nw_or, cs);
		moving_flag = (cs == 2);

		// update turtle's postion while not at end and moving
		if (moving_flag == true && aend == false) {    // when intend to move forward
			moveTurtleBasedOnOrientation(pos_, nw_or, localX, localY);
			localMap[localX][localY]++;
			displayVisits(localMap[localX][localY]);
			moving_flag = false;
		}
		wait = TIMEOUT;
  	} else {
		wait -= 1;
	}
	if (aend) {
		return false; 			// don't submit change if reaches destination
	}
	return wait == TIMEOUT;		// return true if it's time to submit the changes, false otherwise
}
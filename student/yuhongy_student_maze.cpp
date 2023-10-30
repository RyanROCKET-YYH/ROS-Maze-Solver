/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Yuhong YAO
 * ANDREW ID:	yuhongy
 * LAST UPDATE:	9/29/2023
 *
 * This file keeps track of where the turtle is in the maze
 * and updates the location when the turtle is moved. It shall not
 * contain the maze solving logic/algorithm.
 *
 * This file is used along with student_turtle.cpp. student_turtle.cpp shall
 * contain the maze solving logic/algorithm and shall not make use of the
 * absolute coordinates or orientation of the turtle.
 *
 * This file shall call studentTurtleStep(..) in student_turtle.cpp to determine
 * the next move the turtle will make, and shall use translatePos(..) and
 * translateOrnt(..) to translate this move into absolute coordinates
 * to display the turtle.
 *
 */

#include "student.h"

/*
 * This procedure takes the current turtle position and orientation and returns true=accept changes, false=do not accept changes
 * Ground rule -- you are only allowed to call the three helper functions defined in student.h, and NO other turtle methods or maze methods (no peeking at the maze!)
 * This file interfaces with functions in student_turtle.cpp
 */



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
 * @brief Move the turtle based on its current position and orientation.
 * 		  interact with studentTurtleStep in turtle.cpp to determine
 * 		  the next move the turtle will make either move forward, turn left, turn right or stop.
 * 
 * @param pos_ Current position of the turtle.
 * @param nw_or Current orientation of the turtle.
 * @return Returns true if changes are accepted, false otherwise.
 */
bool moveTurtle(QPointF& pos_, int& nw_or) {
	static int32_t wait;
	const int32_t TIMEOUT = 0;

	ROS_INFO("Turtle update Called  w=%d", wait);
	ROS_INFO("Current Orientation: %d", nw_or);
	bool aend, moving_flag;
  	if (!wait) {
		Point2D startPoint, endPoint;
		// update start and end position so that we can check if bumped
		updateStartPosition(pos_, startPoint);
		updateEndPosition(pos_, nw_or, endPoint, startPoint);
		bool bp = bumped(startPoint.x, startPoint.y, endPoint.x, endPoint.y);  // if there is a bump (boolean)
		bool aend = atend(pos_.x(), pos_.y());
		turtleResult result = studentTurtleStep(bp, aend);
		turtleMove nextMove = result.nextMove;
		int32_t visits = result.visits;
		nw_or = translateOrnt(nw_or, nextMove);
		if (nextMove == STOP) {
    		return false; // Don't submit changes if the turtle should stop
		}
		if (nextMove == MOVE) {
			pos_ = translatePos(pos_, nextMove, nw_or);
		}
		displayVisits(visits);
		wait = TIMEOUT;
  	} else {
		wait -= 1;
	}
  	// return studentMoveTurtle(pos_, nw_or);
	return wait == TIMEOUT;	
}

/**
 * @brief Translate the turtle's position based on its absoulte postion, move and orientation.
 * 
 * @param pos_ Current position of the turtle.
 * @param nextMove The next move the turtle will make.
 * @param nw_or Current orientation of the turtle.
 * @return Returns the new position of the turtle.
 */
QPointF translatePos(QPointF pos_, turtleMove nextMove, int32_t nw_or) {
	switch (nextMove) {
        case MOVE:
            switch (nw_or) {
				case east:
                    pos_.setX(++pos_.rx());
                    break;
                case south:
                    pos_.setY(++pos_.ry());
                    break;
                case west:
                    pos_.setX(--pos_.rx());
                    break;
                case north:
                    pos_.setY(--pos_.ry());
                    break;
				default:
					ROS_ERROR("Unexpected value for turtle's direction: %d", nw_or);
					break;
			}
            break;
        case TURN_LEFT:
			break;
        case TURN_RIGHT:
			break;
        case STOP:
            break;
		default:
			ROS_ERROR("Unexpected value for turtle's next move: %d", nextMove);
    }
    return pos_;
}

/**
 * @brief Translate the turtle's orientation based on its move.
 * 
 * @param orientation Current orientation of the turtle.
 * @param nextMove The next move the turtle will make.
 * @return Returns the new orientation of the turtle.
 */
int translateOrnt(int orientation, turtleMove nextMove) {
	int32_t cycle = 4;			// number of orientations

	switch (nextMove) {
		case STOP:
			break;
		case TURN_LEFT:
			orientation = (orientation + cycle - 1) % cycle;
			break;
		case TURN_RIGHT:
			orientation = (orientation + cycle + 1) % cycle;
			break;
		case MOVE:
			break;
		default:
			ROS_ERROR("Unexpected value for turtle's direction: %d", orientation);
			break;
	}
	return orientation;
}

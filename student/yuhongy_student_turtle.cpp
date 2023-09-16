/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Yuhong YAO
 * ANDREW ID: yuhongy
 * LAST UPDATE:
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the left-hand rule. The code is intentionaly left obfuscated.
 *
 */

#include "student.h"

// Ignore this line until project 5
turtleMove studentTurtleStep(bool bumped) { return MOVE; }

// OK TO MODIFY BELOW THIS LINE

const int32_t TIMEOUT = 20;  // bigger number slows down simulation so you can see what's happening   
int32_t wait; // w: countdown time.

// Define the Coordinate type
typedef double Cord;
struct Point2D {
    Cord x;
    Cord y;
};

// this procedure takes the current turtle position and orientation and returns
// true=submit changes, false=do not submit changes
// Ground rule -- you are only allowed to call the helper functions "bumped(..)"
// and "atend(..)", and NO other turtle methods or maze methods (no peeking at
// the maze!)


// left hand orientation enum
/*enum TurtleOrientation {
	north = 0,
	east = 1,
	south = 2,
	west = 3,
};*/

// right hand orientation enum
enum TurtleOrientation {
	// enum for turtle's orientation
	south = 0,
	west = 1,
	north = 2,
	east = 3,
};

enum TurtleState {
	// enum for turtle's current state
	turned_bumped = 0,
	turned_forward = 1,
	moving_forward = 2,
};
TurtleState cs;

// module that determine turtle's next state
void determineNextDirectionAndState(TurtleOrientation &direction, TurtleState &state, bool bumped) {
    TurtleOrientation turnDirection;
    TurtleState nextState;
    // Define the turn directions for each orientation
    switch(direction) {
        case north:
            turnDirection = east;
            break;
        case east:
            turnDirection = south;
            break;
        case south:
            turnDirection = west;
            break;
        case west:
            turnDirection = north;
            break;
        default:
            ROS_ERROR("Unexpected value for turtle's direction: %d", direction);
            return;
    }
    if (state == moving_forward) {
        direction = turnDirection;
        nextState = turned_forward;
    } else if (bumped) {
        direction = static_cast<TurtleOrientation>((direction + 2) % 4);  // Opposite direction
		direction = static_cast<TurtleOrientation>((direction + 3) % 4);  // then turn left
        nextState = turned_bumped;
    } else {
        nextState = moving_forward;
    }
    state = nextState;
}


bool studentMoveTurtle(QPointF &pos_, int &nw_or) {    
	// call in everyloops to return wait time
	ROS_INFO("Turtle update Called  w=%d", wait);
	bool aend;
  	bool mod = true;
  	if (!wait) {
		Point2D startPoint;
    	Point2D endPoint;
    
    	startPoint.x = pos_.x();	// initialize the position and used to check for bump
    	startPoint.y = pos_.y();

    	endPoint.x = pos_.x();
    	endPoint.y = pos_.y();	

		/*if (nw_or == north || nw_or == east) {
      		nw_or == north ? fy2++ : fx2++;
    	} else {
      		fx2 += 1;
			fy2 += 1;
			nw_or == south ? fx1++ : fy1++;
    	}*/
    	// right hand rule
		//get updated coordination
		if (nw_or == south || nw_or == west) {
      		nw_or == south ? endPoint.y++ : endPoint.x++;
    	} else {
      		endPoint.x += 1;
			endPoint.y += 1;
			nw_or == north ? startPoint.x++ : startPoint.y++;
    	}

		bool bp = bumped(startPoint.x, startPoint.y, endPoint.x, endPoint.y);  // see if there is a bump (boolean)
		//ROS_INFO("bumped?: %s", bp ? "ture" : "false");
		aend = atend(pos_.x(), pos_.y()); // check if arrvies at end (boolean)
		//ROS_INFO("at end?: %s", aend ? "ture" : "false");
		ROS_INFO("Current state: %f, Orientation: %d", cs, nw_or);
		//left hand rule
		/*switch(nw_or) {
			case north:
				cs == moving_forward ? (nw_or = west, cs = turned_forward) :
				bp ? (nw_or = east, cs = turned_bumped) : cs = moving_forward;
				break;
			
			case east:
				cs == moving_forward ? (nw_or = north, cs = turned_forward) :
				bp ? (nw_or = south, cs = turned_bumped) : cs = moving_forward;
				break;

			case south:
				cs == moving_forward ? (nw_or = east, cs = turned_forward) :
				bp ? (nw_or = west, cs = turned_bumped) : cs = moving_forward;
				break;

			case west:
				cs == moving_forward ? (nw_or = south, cs = turned_forward) :
				bp ? (nw_or = north, cs = turned_bumped) : cs = moving_forward;
				break;
		}*/
		// right hand rule
		
		switch(nw_or) {
			// according to turtle's current direction, and currentstate for bumped and aend
			// decide which direction or action with turtle do at next time tick
			// input: nw_or, cs, bp. output: cs, nw_or
			case north:
				cs == moving_forward ? (nw_or = east, cs = turned_forward) :
				bp ? (nw_or = west, cs = turned_bumped) : cs = moving_forward;
				break;
			
			case east:
				cs == moving_forward ? (nw_or = south, cs = turned_forward) :
				bp ? (nw_or = north, cs = turned_bumped) : cs = moving_forward;
				break;

			case south:
				cs == moving_forward ? (nw_or = west, cs = turned_forward) :
				bp ? (nw_or = east, cs = turned_bumped) : cs = moving_forward;
				break;

			case west:
				cs == moving_forward ? (nw_or = north, cs = turned_forward) :
				bp ? (nw_or = south, cs = turned_bumped) : cs = moving_forward;
				break;
			
			default:
				ROS_ERROR("Unexpected value for turtle's direction: %d", nw_or);
				break;
		}
		
		// determineNextDirectionAndState(nw_or, cs, bp); right now it doesn't work

		/*	
		if (nw_or == north) {
		if (cs == moving_forward) {	// intend to move forward
			nw_or = west;    // turn left 0->3 is turning left
			cs = turned_forward;    // recently turned
		} else if (bp) {    // if bumped, means can't move foward
			nw_or = east;    // since left hand rule, it will turn right, 0->1 turning right
			cs = turned_bumped;    // turned and bumped
		} else
			cs = moving_forward;    //intend to move forward
		} else if (nw_or == east) {   
		if (cs == moving_forward) {
			nw_or = north;  
			cs = turned_forward;
		} else if (bp) {
			nw_or = south;    
			cs = turned_bumped;
		} else
			cs = moving_forward;
		} else if (nw_or == south) {
		if (cs == moving_forward) {
			nw_or = east;    
			cs = turned_forward;
		} else if (bp) {
			nw_or = west;    
			cs = turned_bumped;
		} else
			cs = moving_forward;
		} else if (nw_or == west) {
		if (cs == moving_forward) {
			nw_or = south;    
			cs = turned_forward;
		} else if (bp) {
			nw_or = north;    
			cs = turned_bumped;
		} else
			cs = moving_forward;
		}*/


		ROS_INFO("Orientation=%f  STATE=%f", nw_or, cs);
		bool moving_flag = cs == 2;
		mod = true;
		/*if (z == true && aend == false) {    // when intend to move forward
			if (nw_or == east)
				pos_.setY(pos_.y() - 1);    // or = 1, turn left (y-1), or = 1 is east
			if (nw_or == south)
				pos_.setX(pos_.x() + 1);    // south nw_or = 2, x+1
			if (nw_or == west)
				pos_.setY(pos_.y() + 1);    // east nw_or = 3, y+1
			if (nw_or == north)
				pos_.setX(pos_.x() - 1);    // north nw_or = 0, x-1
			z = false;
			mod = true;
		}*/
		// update the turtle's coordination in the maze for next loop
		// input: flag(z), aend, nw_or. output: pos
		if(moving_flag == true && aend == false) {
			switch(nw_or) {
				case west:
					pos_.setY(pos_.y() - 1);    // west y-1
					break;
				case north:
					pos_.setX(pos_.x() + 1);    // north
					break;
				case east:
					pos_.setY(pos_.y() + 1);    // east
					break;
				case south:
					pos_.setX(pos_.x() - 1);    // south
					break;
				default:
					ROS_ERROR("Unexpected value for turtle's direction: %d", nw_or);
					break;
			}
			moving_flag = false;
			mod = true;
		}
  	}
	if (aend) {
		return false; // don't submit change if reaches destination
	}
  	if (!wait) {
		wait = TIMEOUT;
	} else {
		wait -= 1;
	}

	return wait == TIMEOUT;
}

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
// this procedure takes the current turtle position and orientation and returns
// true=submit changes, false=do not submit changes
// Ground rule -- you are only allowed to call the helper functions "bumped(..)"
// and "atend(..)", and NO other turtle methods or maze methods (no peeking at
// the maze!)
typedef int32_t Cord;			 // Define the Coordinate type
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
/*
// right hand orientation enum
enum TurtleOrientation {		 // enum for turtle's orientation
	south = 0,
	west = 1,
	north = 2,
	east = 3,
};
*/
enum TurtleState {				 // enum for turtle's current state
	turned_bumped = 0,
	turned_forward = 1,
	moving_forward = 2,
};

enum TurnDirection {
	left = -1,
	right = 1,
};

TurtleOrientation getNextDir(TurtleOrientation nw_or, TurnDirection turn){
	int8_t cycle = 4;
	int8_t nextDir = (nw_or + turn + cycle)%4;
	return static_cast<TurtleOrientation>(nextDir);
}

// module that determine turtle's next state
void TurtleStateUpdate(TurtleOrientation &nw_or, TurtleState &cs, bool bumped){
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

/* this section get turtle's pos and orientation
 * update turtle's pos and orientation using wall following rule
 */
bool studentMoveTurtle(QPointF &pos_, TurtleOrientation &nw_or) {    
	// call in everyloops to return wait time 
	static int32_t wait;
	static TurtleState cs;	
	const int32_t TIMEOUT = 2; 	 // bigger number slows down simulation so you can see what's happening
	ROS_INFO("Turtle update Called  w=%d", wait);
	bool aend, moving_flag;
  	bool mod = true;
  	if (!wait) {
		Point2D startPoint;
    	Point2D endPoint;
    
    	startPoint.x = pos_.x();	// initialize the position and used to check for bump
    	startPoint.y = pos_.y();

    	endPoint.x = pos_.x();
    	endPoint.y = pos_.y();	

		if (nw_or == north || nw_or == east) {
      		nw_or == north ? endPoint.y++ : endPoint.x++;
    	} else {
      		endPoint.x++;
			endPoint.y++;
			nw_or == south ? startPoint.x++ : startPoint.y++;
    	}

    	// right hand rule
		//get updated coordination
		/*
		if (nw_or == south || nw_or == west) {
      		nw_or == south ? endPoint.y++ : endPoint.x++;
    	} else {
      		endPoint.x += 1;
			endPoint.y += 1;
			nw_or == north ? startPoint.x++ : startPoint.y++;
    	}
		*/
		
		bool bp = bumped(startPoint.x, startPoint.y, endPoint.x, endPoint.y);  // if there is a bump (boolean)
		aend = atend(pos_.x(), pos_.y()); 									   // if arrvies at end (boolean)
		ROS_INFO("Current state: %d, Orientation: %d", cs, nw_or);
		ROS_INFO("Current position: x = %d, y = %d", pos_.x(), pos_.y());

		//left hand rule
		TurtleStateUpdate(nw_or, cs, bp);

		// right hand rule
		/*switch(nw_or) {
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
		*/
		// determineNextDirectionAndState(nw_or, cs, bp); 

		ROS_INFO("Orientation=%d  STATE=%d", nw_or, cs);
		moving_flag = (cs == 2);
		mod = true;
		if (moving_flag == true && aend == false) {    // when intend to move forward
			switch (nw_or)
			{
			case east:
				pos_.setY(--pos_.ry());	// or = 1, turn left (y-1), or = 1 is east
				break;
			case south:
				pos_.setX(++pos_.rx());	// south nw_or = 2, x+1
				break;
			case west:
				pos_.setY(++pos_.ry());	// east nw_or = 3, y+1
				break;
			case north:
				pos_.setX(--pos_.rx());	// north nw_or = 0, x-1
				break;
			default:
				ROS_ERROR("Unexpected value for turtle's direction: %d", nw_or);
				break;
			}  
			moving_flag = false;
			mod = true;
		}
		// update the turtle's coordination in the maze for next loop
		// input: flag(z), aend, nw_or. output: pos
		/*if(moving_flag == true && aend == false) {			// right-hand rule
			switch(nw_or) 
			{
			case west:
				pos_.setY(--pos_.ry());    // west y-1
				break;
			case north:
				pos_.setX(++pos_.rx());    // north
				break;
			case east:
				pos_.setY(++pos_.ry());    // east
				break;
			case south:
				pos_.setX(--pos_.rx());    // south
				break;
			default:
				ROS_ERROR("Unexpected value for turtle's direction: %d", nw_or);
				break;
			}
			moving_flag = false;
			mod = true;
		}*/	
		wait = TIMEOUT;
  	} else {
		wait -= 1;
	}
	if (aend) {
		return false; 			// don't submit change if reaches destination
	}
	return wait == TIMEOUT;
}
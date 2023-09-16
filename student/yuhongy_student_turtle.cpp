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
int32_t wait;        // w: countdown time. cs: current state.
bool moving_flag, bp, aend, mod;

// this procedure takes the current turtle position and orientation and returns
// true=submit changes, false=do not submit changes
// Ground rule -- you are only allowed to call the helper functions "bumped(..)"
// and "atend(..)", and NO other turtle methods or maze methods (no peeking at
// the maze!)

/*enum TurtleOrientation {
	north = 0,
	east = 1,
	south = 2,
	west = 3,
};*/

enum TurtleOrientation {
	south = 0,
	west = 1,
	north = 2,
	east = 3,
};

enum TurtleState {
	turned_bumped = 0,
	turned_forward = 1,
	moving_forward = 2,
};
TurtleState cs;

bool studentMoveTurtle(QPointF &pos_, int &nw_or) {
	int fx1, fy1, fx2, fy2; // current position of turtle
	ROS_INFO("Turtle update Called  w=%f", wait);

  
  	mod = true;
  	if (!wait) {
		fx1 = pos_.x();
		fy1 = pos_.y(); // initialize the position and used to check for bump
		fx2 = pos_.x();
		fy2 = pos_.y();
		/*if (nw_or == north || nw_or == east) {
      		nw_or == north ? fy2++ : fx2++;
    	} else {
      		fx2 += 1;
			fy2 += 1;
			nw_or == south ? fx1++ : fy1++;
    	}*/
    	// right hand rule
		if (nw_or == south || nw_or == west) {
      		nw_or == south ? fy2++ : fx2++;
    	} else {
      		fx2 += 1;
			fy2 += 1;
			nw_or == north ? fx1++ : fy1++;
    	}

		bp = bumped(fx1, fy1, fx2, fy2);  // see if there is a bump (boolean)
		//ROS_INFO("bumped?: %s", bp ? "ture" : "false");
		aend = atend(pos_.x(), pos_.y()); // check if arrvies at end (boolean)
		//ROS_INFO("at end?: %s", aend ? "ture" : "false");
		ROS_INFO("Current state: %f, Orientation: %d", cs, nw_or);
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
		moving_flag = cs == 2;
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
	if (aend) return false; // don't submit change if reaches destination
  	if (!wait) {
		wait = TIMEOUT;
	} else wait -= 1;

	return wait == TIMEOUT;
}

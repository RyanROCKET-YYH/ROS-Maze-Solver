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

#define TIMEOUT 2 // bigger number slows down simulation so you can see what's happening
float w, cs;              // w: countdown time. cs: current state.
float fx1, fy1, fx2, fy2; // current position of turtle
float z, aend, mod, bp, q;

// this procedure takes the current turtle position and orientation and returns
// true=submit changes, false=do not submit changes
// Ground rule -- you are only allowed to call the helper functions "bumped(..)"
// and "atend(..)", and NO other turtle methods or maze methods (no peeking at
// the maze!)

enum TurtleOrientation {
	west = 0,
	south = 1,
	east = 2,
	north = 3,
};

bool studentMoveTurtle(QPointF &pos_, int &nw_or) {
  ROS_INFO("Turtle update Called  w=%f", w);
  ROS_INFO("Current value of cs: %f", cs);
  mod = true;
  if (w == 0) {
    fx1 = pos_.x();
    fy1 = pos_.y(); // initialize the position and used to check for bump
    fx2 = pos_.x();
    fy2 = pos_.y();
    if (nw_or < 2) { // magic number what does nw_or point to at 0,1,2,3 
      if (nw_or == west)
        fy2 += 1; //
      else    // when nw_or = 1, fx += 1
        fx2 += 1; //
    } else {
      fx2 += 1;
      fy2 += 1;
      if (nw_or == east)
        fx1 += 1;
      else
        fy1 += 1;
    }
    bp = bumped(fx1, fy1, fx2, fy2);  // see if there is a bump (boolean)
	ROS_INFO("bumped?: %s", bp ? "ture" : "false");
    aend = atend(pos_.x(), pos_.y()); // check if arrvies at end (boolean)
	ROS_INFO("at end?: %s", aend ? "ture" : "false");
    if (nw_or == west) {
      if (cs == 2) {	// not sure what makes cs means and nw_or=3 didn't happened
        nw_or = north;    // cs = 2, turn right
        cs = 1;
      } else if (bp) {    //  if bumped, cs becomes 0 turtle turn left
        nw_or = south;
        cs = 0;
      } else
        cs = 2;
    } else if (nw_or == south) {    // 
      if (cs == 2) {
        nw_or = west;    // turn right
        cs = 1;
      } else if (bp) {
        nw_or = east;    // bumped, turn left which is east
        cs = 0;
      } else
        cs = 2;
    } else if (nw_or == east) {
      if (cs == 2) {
        nw_or = south;    // turn right
        cs = 1;
      } else if (bp) {
        nw_or = north;    // bumped, turn left which is north
        cs = 0;
      } else
        cs = 2;
    } else if (nw_or == north) {
      if (cs == 2) {
        nw_or = east;    // turn right
        cs = 1;
      } else if (bp) {
        nw_or = west;    // turn left, west
        cs = 0;
      } else
        cs = 2;
    }
    ROS_INFO("Orientation=%f  STATE=%f", nw_or, cs);
    z = cs == 2;
    mod = true;
    if (z == true && aend == false) {
      if (nw_or == south)
        pos_.setY(pos_.y() - 1);    // move south when nw_or = 1
      if (nw_or == east)
        pos_.setX(pos_.x() + 1);    // move east when nw_or = 2
      if (nw_or == north)
        pos_.setY(pos_.y() + 1);    // move north when nw_or = 3
      if (nw_or == west)
        pos_.setX(pos_.x() - 1);    // move west when nw_or = 0
      z = false;
      mod = true;
    }
  }
  if (aend)
    return false; // don't submit change if reaches destination
  if (w == 0)
    w = TIMEOUT; // countdown to 0 and reset to TIMEOUT
  else
    w -= 1;
  if (w == TIMEOUT)
    return true; // submit change
  return false;
}

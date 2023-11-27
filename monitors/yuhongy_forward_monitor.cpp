/*
 * Author: Yuhong Yao 
 * AndrewID: yuhongy
 *
 * This monitor checks that turtle only move in the forward direction
 *
 */
#ifdef testing
#include "yuhongy_mock.h"
#endif
#ifndef testing
#include "monitor_interface.h"
#endif

// Keeps track of the last pose received and the last orientation
// moved is true if at least one pose has been received, false otherwise
static Pose last_pose;
static bool moved = false;
static Orientation last_orientation;
static bool isForwardMovement = false;

// Flag that doesn't print pose updates if the turtle has turned 0 steps
static const bool suppress_double_visits = true;

std::string orientationToString(Orientation o) {
  switch(o) {
  case NORTH:
    return "NORTH";
  case WEST:  
    return "WEST";
  case SOUTH: 
    return "SOUTH";
  case EAST:  
    return "EAST";
  default:    
    return "ERROR";
  }
}

/*
 * Whenever the turtle turned, compare the current orientation  
 * to the previous orientation and throw an invariant violation
 * if the orientations is differed by 2.
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
  std::string o_str = orientationToString(o);
  // Print pose info
  // Last conditional makes sure that if suppress_double_visits is
  // true, that the same pose isn't printed twice
  if ((last_pose.x != x || last_pose.y != y) && moved) {
    
    switch(last_orientation) {
      case NORTH:
        isForwardMovement = (last_pose.y + 1 == y);
        break;
      case SOUTH:
        isForwardMovement = (last_pose.y - 1 == y);
        break;
      case EAST:
        isForwardMovement = (last_pose.x + 1 == x);
        break;
      case WEST:
        isForwardMovement = (last_pose.x - 1 == x);
        break;
      default:
        isForwardMovement = false;
        break;
    }
    ROS_INFO("[[%ld ns]] 'Pose' was sent. Data: x = %d, y=%d, o=%s", t.toNSec(), x, y, o_str.c_str());

    if (!isForwardMovement) {
      std::string last_o_str = orientationToString(last_orientation);
      ROS_WARN("VIOLATION: Turtle is facing %s but moved %s!", o_str.c_str(), last_o_str.c_str());
    }
  }

  // store last Orientation in memory
  last_orientation = o;
  // store last Pose in memory
  last_pose.x = x;
  last_pose.y = y;

  // Update this flag the first time the turtle moved
  if (!moved) {
    moved = true;
  }
}

/*
 * Empty interrupt handlers beyond this point
 */

void tickInterrupt(ros::Time t) {
}

void visitInterrupt(ros::Time t, int visits) {
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
}
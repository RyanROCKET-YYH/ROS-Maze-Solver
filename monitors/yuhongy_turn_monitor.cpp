/*
 * Author: Yuhong Yao 
 * AndrewID: yuhongy
 *
 * This monitor checks that the invariant "turtle shall turns no
 * more than 90 degrees per simulator time step" is not violated.
 * It keeps track of the previous diretion of the turtle and compares it
 * to the current direction to check the invariant.
 */
#ifdef testing
#include "yuhongy_mock.h"
#endif
#ifndef testing
#include "monitor_interface.h"
#endif

// Keeps track of the last pose received
// moved is true if at least one pose has been received, false otherwise
// static Pose last_pose;
static bool turned = false;
static Orientation last_orientation;

// Flag that doesn't print pose updates if the turtle has turned 0 degrees
static const bool suppress_double_turns = true;

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
 * Returns absolute value of x
 * WARNING: unsafe for edge-case values
 */
inline int abs(int x) {
  return x < 0 ? -1*x : x;
}

/*
 * Whenever the turtle turned, compare the current orientation
 * to the previous orientation and throw an invariant violation
 * if the orientations is differed by 2.
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
  std::string o_str = orientationToString(o);
  // Print pose info
  // Last conditional makes sure that if suppress_double_turns is
  // true, that the same pose isn't printed twice
  if (!suppress_double_turns || !turned || last_orientation != o) {
    ROS_INFO("[[%ld ns]] 'Pose' was sent. Data: x = %d, y=%d, o=%s", t.toNSec(), x, y, o_str.c_str());
  }

  // Check that the turtle has turned before and that the degree is not more than 90
  if (turned && (abs(last_orientation - o)) == 2) {
    std::string last_o_str = orientationToString(last_orientation);
    ROS_WARN("VIOLATION: Difference between last orientation: %s and current orientation: %s is more than 90 degrees!", last_o_str.c_str(), o_str.c_str());
  }

  // store last Orientation in memory
  last_orientation = o;

  // Update this flag the first time the turtle turned
  if (!turned) {
    turned = true;
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
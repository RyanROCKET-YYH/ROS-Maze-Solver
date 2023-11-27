/*
 * Author: Yuhong Yao 
 * AndrewID: yuhongy
 *
 * This monitor ensures the turtle do not move across the wall
 * In my implementation, everytime I receive a pose, the bumpInterrupt will be called
 * so that I can check if the turtle is moving across the wall by
 * checking the moving direction is bumped or not
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
static bool last_bumped = false;


/*
 * checks if the turtle has moved and since the last bump is the same direction as the moving direction
 * it will check if the turtle is moving across the wall
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
  if (moved && last_bumped) {
    ROS_WARN("VIOLATION: Turtle is moving across the wall!");
  }

  // Update last_pose and moved
  if (last_pose.x != x || last_pose.y != y) {
    moved = true;
    last_pose.x = x;
    last_pose.y = y;
  } else {
    moved = false;
  }
}

// keep track of the last bumped
void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    // Print bump info
    ROS_INFO("[[%ld ns]] 'Bump' was sent. Data: x1 = %d, y1=%d, x2=%d, y2=%d, bumped=%d", t.toNSec(), x1, y1, x2, y2, bumped);
    // Check if the turtle is moving across the wall
    last_bumped = bumped;
}

/*
 * Empty interrupt handlers beyond this point
 */

void tickInterrupt(ros::Time t) {
}

void visitInterrupt(ros::Time t, int visits) {
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
}
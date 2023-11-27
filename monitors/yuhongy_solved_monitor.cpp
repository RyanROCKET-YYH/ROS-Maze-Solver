/*
 * Author: Yuhong Yao 
 * AndrewID: yuhongy
 *
 * This monitor ensures that the turtle does not when it has solve the maze.
 * 
 */

#include "monitor_interface.h"

// Keeps track of the last pose received
static Pose last_pose;
static bool solved = false;
static Orientation last_orientation;

/*
 * update the solved flag when the turtle reaches the end of the maze
 */
void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
  if (atEnd) {
    ROS_INFO("Turtle has solved the maze!");
    solved = true;
    last_pose.x = x;
    last_pose.y = y;
  }
}

/*
 * Check if the turtle has moved after solving the maze
 * by comparing the current pose to the last pose and orientation
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {

  if (solved && (last_pose.x != x || last_pose.y != y || last_orientation != o)) {
    ROS_WARN("VIOLATION: Turtle has moved after solving the maze!");
  }
  // store last Orientation in memory
  last_orientation = o;
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


/*
 * Author: Yuhong Yao 
 * AndrewID: yuhongy
 *
 * This monitor ensures that the turtle does not search for the end of the maze
 * it can only call atend for its current position
 * 
 */

#include "monitor_interface.h"

// Keeps track of the last pose received
static Pose curr_pose;
static bool solved = false;
static bool poseUpdated = false; // flag that indicates if the pose has been updated    

/*
 * Update the current pose of the turtle
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    curr_pose.x = x;
    curr_pose.y = y;
    poseUpdated = true;
}

/*
 * Check if the turtle is calling the atend function for its current position
 */
void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
    if (poseUpdated && (curr_pose.x != x || curr_pose.y != y)) {
        ROS_WARN("VIOLATION: Turtle is not calling atend based on current location!");
    }

    if (atEnd) {
        ROS_WARN("Successful at end of the maze");
        solved = true;
    } else {
        ROS_INFO("[[%ld ns]] 'Atend' was sent. Data: x = %d, y=%d", t.toNSec(), x, y);
    }
    poseUpdated = false;
}


/*
 * Empty interrupt handlers beyond this point
 */

void visitInterrupt(ros::Time t, int visits) {
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
}

void tickInterrupt(ros::Time t) {
}

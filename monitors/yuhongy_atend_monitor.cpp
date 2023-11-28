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
static ros::Time lastPoseUpdateTime; // Timestamp of the last pose update   
static bool poseUpdated = false; // Flag to indicate if the pose has been updated since the last atend call
static const int endThreshold = 10; // Threshold for the distance between the current pose and the end pose
static int endCounter = 0;

/*
 * Update the current pose of the turtle
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    curr_pose.x = x;
    curr_pose.y = y;
    lastPoseUpdateTime = t; 
    poseUpdated = true;
}

/*
 * Check if the turtle is calling the atend function for its current position
 */
void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
    if (t < lastPoseUpdateTime) {
        return; // Ignore this interrupt if the pose was updated after this interrupt was sent
    }

    if (atEnd) {
        endCounter++;
        ROS_INFO("Successful at end of the maze");
        solved = true;
        if (endCounter > endThreshold) {
            ros::shutdown();
        }
    } else {
        ROS_INFO("[[%ld ns]] 'Atend' was sent. Data: x = %d, y=%d", t.toNSec(), x, y);
        if (poseUpdated && (curr_pose.x != x || curr_pose.y != y)) {
            ROS_WARN("VIOLATION: Turtle is calling atend(%d, %d) but current location is (%d, %d)!", x, y, curr_pose.x, curr_pose.y);
        }
    }
    poseUpdated = false;
}

void tickInterrupt(ros::Time t) {
    if (solved) {
        ROS_WARN("Turetle at the end of MAZE needs stop!");
    }
}

/*
 * Empty interrupt handlers beyond this point
 */

void visitInterrupt(ros::Time t, int visits) {
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
}



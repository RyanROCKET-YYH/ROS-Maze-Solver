/*
 * Author: Yuhong Yao 
 * AndrewID: yuhongy
 *
 * This monitor checks that the in each tick_interrupt  (every time moveTurtle is called)
 * each of the three interrupts (poseInterrupt, visitInterrupt, and bumpInterrupt) happens eacetly once 
 * in response to each tick_interrupt.
 *
 */
#ifdef testing
#include "yuhongy_mock.h"
#endif
#ifndef testing
#include "monitor_interface.h"
#endif

// Keeps track of the each interrupt are being called
static bool pose_call = false;
static bool visits_call = false;
static bool bump_call = false;

/*
 * Whenever the turtle turned, compare the current orientation
 */
void tickInterrupt(ros::Time t) {
    // reset the flag in the tickInterrupt
    pose_call = false;
    visits_call = false;
    bump_call = false;
}

/*
 * record the time in each interrupt and check if the poseinterrupt is called more than once
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
    if (pose_call) {
        ROS_WARN("VIOLATION: poseInterrupt is called more than once in a tick");
    } else {
        pose_call = true;
    }
}

/*
 * record the time in each interrupt and check if the visitinterrupt is called more than once
 */
void visitInterrupt(ros::Time t, int visits) {
    if (visits_call) {
        ROS_WARN("VIOLATION: visitInterrupt is called more than once in a tick");
    } else {
        visits_call = true;
    }
}

/*
 * record the time in each interrupt and check if the bumpinterrupt is called more than once
 */
void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    if (bump_call) {
        ROS_WARN("VIOLATION: bumpInterrupt is called more than once in a tick");
    } else {
        bump_call = true;
    }
}

/*
 * Empty interrupt handlers beyond this point
 */
void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
}
/*
 * Author: Yuhong Yao 
 * AndrewID: yuhongy
 *
 * This monitor ensures the turtle do not check the wall bump coindition in the wrong direction
 * In my implementation, everytime I receive a pose, the bumpInterrupt will be called
 * so that I can check if the turtle is checking the wall in the forward direction
 * by comparing the endpoints of the wall segment based on the current pose and orientation
 * with the endpoints of the wall segment from bumpInterrupt
 */
#ifdef testing
#include "yuhongy_mock.h"
#endif
#ifndef testing
#include "monitor_interface.h"
#endif

// get the current pose of the turtle from poseInterrupt
static Pose curr_pose;
static Orientation curr_orientation;
static bool poseUpdated = false; // flag that indicates if the pose has been updated
static bool orientationUpdated = false;

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
 * get the current pose of the turtle
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
  if (curr_orientation != o) {
    curr_orientation = o;
    orientationUpdated = true;
  }
  curr_pose.x = x;
  curr_pose.y = y;
  poseUpdated = true;
}

/*
 * get the endpoints of the turtle based on its current position and orientation
 */
Endpoints getEndpoints(Pose pose, Orientation o) {
  Endpoints curr_endpoints;
  switch(o) {
    case NORTH:
      curr_endpoints = {pose.x, pose.y, pose.x + 1, pose.y};
      break;
    case SOUTH:
      curr_endpoints = {pose.x, pose.y + 1, pose.x + 1, pose.y + 1};
      break;
    case EAST:
      curr_endpoints = {pose.x + 1, pose.y, pose.x + 1, pose.y + 1};
      break;
    case WEST:
      curr_endpoints = {pose.x, pose.y, pose.x, pose.y + 1};
      break;
    default:
      ROS_ERROR("Unexpected value for turtle's direction");
      break;
  }
  return curr_endpoints;
}

// make sure the turtle is checking the wall in the forward direction
void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
    // Print bump info
    Endpoints curr_endpoints = getEndpoints(curr_pose, curr_orientation);
    ROS_INFO("[[%ld ns]] 'Bump' was sent. Data: x1 = %d, y1=%d, x2=%d, y2=%d, bumped=%d", t.toNSec(), x1, y1, x2, y2, bumped);

    // Check if the turtle is moving across the wall
    if (!(curr_endpoints.x1 == x1 && curr_endpoints.y1 == y1 && curr_endpoints.x2 == x2 && curr_endpoints.y2 == y2) 
    && poseUpdated && orientationUpdated) {
      ROS_WARN("VIOLATION: Turtle is not checking the wall in the forward orientation, instead checking %s", orientationToString(curr_orientation).c_str());
    }

    poseUpdated = false;
    orientationUpdated = false;
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
#include "yuhongy_student_mock.h"
#include <iostream>

static TurtleOrientation mock_orientation;
static bool mock_bump;
static bool mock_atend;
static bool mock_error = false;

bool will_bump() {
  return mock_bump;
}

bool at_end() {
  return mock_atend;
}

/* Functions used to instrument CUnit tests */
TurtleOrientation test_orientation_result() {
  return mock_orientation;
}

TurtleOrientation test_NextMove_result() {
  return mock_orientation;
}

void mock_set_bump(bool bump) {
  mock_bump = bump;
}

void mock_set_atend(bool atend) {
  mock_atend = atend;
}

/* Dummy ROS_ERROR */
void ROS_ERROR(std::string e) {
  mock_error = true;
  std::cout << e << std::endl;
}


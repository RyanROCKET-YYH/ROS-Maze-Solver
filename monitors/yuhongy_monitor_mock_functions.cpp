#include "monitor_interface.h"
#include <cstdio>
#include <cstdarg>


static bool mock_warn = false;

void ROS_INFO(const char* format, ...) {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

void ROS_WARN(const char* format, ...) {
    mock_warn = true;
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

bool was_ros_warn() {
  return mock_warn;
}

void reset_ros_warn() {
    mock_warn = false;
}

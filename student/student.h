#include <ros/ros.h>
#include <boost/bind.hpp>
#include <ece642rtle/timeInt8.h>
#include <std_msgs/Empty.h>
#include <ece642rtle/RTIbump.h>
#include <ece642rtle/RTIatend.h>
#include <ece642rtle/PoseOrntBundle.h>
#include <ece642rtle/bumpEcho.h>
#include <ece642rtle/aendEcho.h>
#include <QPointF>

// Functions to interface with ROS. Don't change these lines!
bool bumped(int x1,int y1,int x2,int y2);
bool atend(int x, int y);
void displayVisits(int visits);
bool moveTurtle(QPointF& pos_, int& nw_or);

// Scope-preserving changes to these lines permitted (see p5 writeup)
// left hand orientation enum
enum TurtleOrientation {
	north = 0,
	east = 1,
	south = 2,
	west = 3,
};

enum turtleMove {MOVE, TURN_LEFT, TURN_RIGHT, STOP};
QPointF translatePos(QPointF pos_, turtleMove nextMove, int32_t nw_or);
int translateOrnt(int orientation, turtleMove nextMove);
turtleResult studentTurtleStep(bool bumped, bool atend);

// OK to change below this line
bool studentMoveTurtle(QPointF& pos_, int& nw_or);

// Define the Coordinate type
typedef int32_t Cord;			
struct Point2D {
	Cord x;
	Cord y;
};

struct turtleResult {
    turtleMove nextMove;
    int32_t visits;
};

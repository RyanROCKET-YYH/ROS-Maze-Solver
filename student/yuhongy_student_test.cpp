#include "yuhongy_student_mock.h"
#include <CUnit/Basic.h>
#include <cstdint>

void test_t1() { // test transition from initialized to checkWall
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(Initialized);
    int32_t x = 11;
    int32_t y = 11;
    setMockLocalCord(x, y);
    setVisitCounts(x, y, 0);
    int32_t mockX = getMockLocalX();
    int32_t mockY = getMockLocalY();
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();

    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(getVisitCounts(mockX,mockY), 1);
    CU_ASSERT_EQUAL(return_state, CheckWall);
}   

void test_t1_1() { // test transition from initialized to checkWall
    mock_set_bump(false);
    mock_set_atend(true);
    setTurtleState(Initialized);
    int32_t x = 11;
    int32_t y = 11;
    setMockLocalCord(x, y);
    setVisitCounts(x, y, 0);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(getVisitCounts(getMockLocalX(), getMockLocalY()), 1);
    CU_ASSERT_EQUAL(return_state, CheckWall);
}  // need 4 tests for this transition

void test_t1_2() { // test transition from initialized to checkWall
    mock_set_bump(true);
    mock_set_atend(false);
    setTurtleState(Initialized);
    int32_t x = 11;
    int32_t y = 11;
    setMockLocalCord(x, y);
    setVisitCounts(x, y, 0);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(getVisitCounts(getMockLocalX(), getMockLocalY()), 1);
    CU_ASSERT_EQUAL(return_state, CheckWall);
}

void test_t1_3() { // test transition from initialized to checkWall
    mock_set_bump(true);
    mock_set_atend(true);
    setTurtleState(Initialized);
    int32_t x = 11;
    int32_t y = 11;
    setMockLocalCord(x, y);
    setVisitCounts(x, y, 0);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();

    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(getVisitCounts(getMockLocalX(), getMockLocalY()), 1);
    CU_ASSERT_EQUAL(return_state, CheckWall);
}

void test_t2() { // test transition from CheckWall to Right
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(CheckWall);
    setSpinCounter(0);
    setTurtleOrientation(north);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    int32_t x = 11;
    int32_t y = 11;

    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(getLocalMap(x, y), 0xE);
    CU_ASSERT_EQUAL(return_state, Right);
}

void test_t2_1() { // test transition from CheckWall to Right
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(CheckWall);
    setSpinCounter(1);
    setTurtleOrientation(east);
    int32_t x = 11;
    int32_t y = 11;
    setLocalMap(x, y, 0xF);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(getLocalMap(x, y), 0xD);
    CU_ASSERT_EQUAL(return_state, Right);
}

void test_t2_2() { // test transition from CheckWall to Right
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(CheckWall);
    setSpinCounter(2);
    setTurtleOrientation(south);
    int32_t x = 11;
    int32_t y = 11;
    setLocalMap(x, y, 0xF);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(getLocalMap(x, y), 0xB);
    CU_ASSERT_EQUAL(return_state, Right);
}

void test_t2_3() { // test transition from CheckWall to Right
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(CheckWall);
    setSpinCounter(3);
    setTurtleOrientation(west);
    int32_t x = 11;
    int32_t y = 11;
    setLocalMap(x, y, 0xF);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(getLocalMap(x, y), 0x7);
    CU_ASSERT_NOT_EQUAL(return_state, Right);
}

void test_t2_4() { // test transition from CheckWall to Right
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(CheckWall);
    setSpinCounter(4);
    setTurtleOrientation(west);
    int32_t x = 11;
    int32_t y = 11;
    setLocalMap(x, y, 0xF);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(getLocalMap(x, y), 0x7);
    CU_ASSERT_NOT_EQUAL(return_state, Right);
}

void test_t2_5() { // test transition from CheckWall to Right
    mock_set_bump(true);
    mock_set_atend(false);
    setTurtleState(CheckWall);
    setSpinCounter(1);
    setTurtleOrientation(west);
    int32_t x = 11;
    int32_t y = 11;
    setLocalMap(x, y, 0xF);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(getLocalMap(x, y), 0xF);
    CU_ASSERT_EQUAL(return_state, Right);
}

void test_t2_6() { // test transition from CheckWall to Right
    mock_set_bump(false);
    mock_set_atend(true);
    setTurtleState(CheckWall);
    setSpinCounter(1);
    setTurtleOrientation(west);
    int32_t x = 11;
    int32_t y = 11;
    setLocalMap(x, y, 0xF);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(getLocalMap(x, y), 0x7);
    CU_ASSERT_NOT_EQUAL(return_state, Right);
} // need 7 tests for this transition



void test_t3() { // test transition from Right to CheckWall
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(Right);
    setSpinCounter(0);
    setTurtleOrientation(west);
    int32_t x = 11;
    int32_t y = 11;
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, TURN_RIGHT);
    CU_ASSERT_EQUAL(getSpinCounter(), 1);
    CU_ASSERT_EQUAL(getTurtleOrientation(), north);
    CU_ASSERT_EQUAL(return_state, CheckWall);
}

void test_t3_1() { // test transition from Right to CheckWall
    mock_set_bump(true);
    mock_set_atend(false);
    setTurtleState(Right);
    setSpinCounter(1);
    setTurtleOrientation(north);
    int32_t x = 11;
    int32_t y = 11;
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, TURN_RIGHT);
    CU_ASSERT_EQUAL(getSpinCounter(), 2);
    CU_ASSERT_EQUAL(getTurtleOrientation(), east);
    CU_ASSERT_EQUAL(return_state, CheckWall);
}

void test_t3_2() { // test transition from Right to CheckWall
    mock_set_bump(true);
    mock_set_atend(false);
    setTurtleState(Right);
    setSpinCounter(2);
    setTurtleOrientation(east);
    int32_t x = 11;
    int32_t y = 11;
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, TURN_RIGHT);
    CU_ASSERT_EQUAL(getSpinCounter(), 3);
    CU_ASSERT_EQUAL(getTurtleOrientation(), south);
    CU_ASSERT_EQUAL(return_state, CheckWall);
}

void test_t3_3() { // test transition from Right to CheckWall
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(Right);
    setSpinCounter(3);
    setTurtleOrientation(south);
    int32_t x = 11;
    int32_t y = 11;
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, TURN_RIGHT);
    CU_ASSERT_EQUAL(getSpinCounter(), 4);
    CU_ASSERT_EQUAL(getTurtleOrientation(), west);
    CU_ASSERT_NOT_EQUAL(return_state, CheckWall);
}

void test_t3_4() { // test transition from Right to CheckWall
    mock_set_bump(false);
    mock_set_atend(true);
    setTurtleState(Right);
    setSpinCounter(1);
    setTurtleOrientation(south);
    int32_t x = 11;
    int32_t y = 11;
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, TURN_RIGHT);
    CU_ASSERT_EQUAL(getSpinCounter(), 2);
    CU_ASSERT_EQUAL(getTurtleOrientation(), west);
    CU_ASSERT_EQUAL(return_state, CheckWall);
} // need 5 tests for this transition

void test_t4() { // test transition from CheckWall to Goal
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(CheckWall);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_NOT_EQUAL(return_state, Goal);
}

void test_t4_1() { // test transition from CheckWall to Goal
    mock_set_bump(false);
    mock_set_atend(true);
    setTurtleState(CheckWall);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(return_state, Goal);
}

void test_t4_2() { // test transition from CheckWall to Goal
    mock_set_bump(true);
    mock_set_atend(false);
    setTurtleState(CheckWall);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_NOT_EQUAL(return_state, Goal);
} // need 3 tests for this transition

void test_t5() { // test transition from CheckWall to DecidNextMove
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(CheckWall);
    setSpinCounter(3);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(return_state, DecideNextMove);
}

void test_t5_1() { // test transition from CheckWall to DecidNextMove
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(CheckWall);
    setSpinCounter(2);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_NOT_EQUAL(return_state, DecideNextMove);
} // need 2 tests for this transition

void test_t6() { // test transition from DecidNextMove to Goal
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(DecideNextMove);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_NOT_EQUAL(return_state, Goal);
}

void test_t6_1() { // test transition from DecidNextMove to Goal
    mock_set_bump(false);
    mock_set_atend(true);
    setTurtleState(DecideNextMove);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(return_state, Goal);
} 

void test_t6_2() { // test transition from DecidNextMove to Goal
    mock_set_bump(true);
    mock_set_atend(false);
    setTurtleState(DecideNextMove);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_NOT_EQUAL(return_state, Goal);
} // need 3 tests for this transition   

void test_t7() { // test transition from DecidNextMove to leftOnce
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(DecideNextMove);
    setMockDesiredDir(north);
    setTurtleOrientation(east);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(return_state, leftOnce);
}

void test_t8() { // test transition from DecidNextMove to leftTwice
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(DecideNextMove);
    setMockDesiredDir(east);
    setTurtleOrientation(west);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(return_state, leftTwice);
}

void test_t9() { // test transition from DecidNextMove to Move
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(DecideNextMove);
    setMockDesiredDir(south);
    setTurtleOrientation(south);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(return_state, Move);
}

void test_t10() { // test transition from DecidNextMove to rightOnce
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(DecideNextMove);
    setMockDesiredDir(east);
    setTurtleOrientation(north);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(return_state, rightOnce);
} 

void test_t17() { // test transition from DecidNextMove with an invalid desired direction
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(DecideNextMove);
    setMockDesiredDir(error);
    setTurtleOrientation(north);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(return_state, DecideNextMove);
} 
// test 7-10 covers all 4 transitions from DecideNextMove to other states

void test_t11() { // test transition from leftTwice to leftOnce
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(leftTwice);
    setTurtleOrientation(east);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();

    CU_ASSERT_EQUAL(result.nextMove, TURN_LEFT);
    CU_ASSERT_EQUAL(getSpinCounter(), 1);
    CU_ASSERT_EQUAL(getTurtleOrientation(), north);
    CU_ASSERT_EQUAL(return_state, leftOnce);
}

void test_t12() { // test transition from leftOnce to Move
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(leftOnce);
    setTurtleOrientation(south);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();

    CU_ASSERT_EQUAL(result.nextMove, TURN_LEFT);
    CU_ASSERT_EQUAL(getSpinCounter(), 0);
    CU_ASSERT_EQUAL(getTurtleOrientation(), east);
    CU_ASSERT_EQUAL(return_state, Move);
}

void test_t13() { // test transition from rightOnce to Move
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(rightOnce);
    setTurtleOrientation(south);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    
    CU_ASSERT_EQUAL(result.nextMove, TURN_RIGHT);
    CU_ASSERT_EQUAL(getSpinCounter(), 0);
    CU_ASSERT_EQUAL(getTurtleOrientation(), west);
    CU_ASSERT_EQUAL(return_state, Move);
}

void test_t14() { // test transition from Move to DecideNextMove
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(Move);
    int32_t x = 8;
    int32_t y = 11;
    setMockLocalCord(x, y);
    setTurtleOrientation(north);
    setVisitCounts(x, y-1, 1);
    setLocalMap(x, y-1, 0x8);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    int32_t mockX = getMockLocalX();
    int32_t mockY = getMockLocalY();

    CU_ASSERT_EQUAL(result.nextMove, MOVE);
    CU_ASSERT_EQUAL(getDesiredDir(), error);
    CU_ASSERT_EQUAL(getVisitCounts(mockX,mockY), 2);
    CU_ASSERT_EQUAL(getMockLocalX(), 8);
    CU_ASSERT_EQUAL(getMockLocalY(), 10);
    CU_ASSERT_EQUAL(return_state, DecideNextMove);
}

void test_t14_1() { // test transition from Move to DecideNextMove
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(Move);
    int32_t x = 11;
    int32_t y = 11;
    setMockLocalCord(x, y);
    setTurtleOrientation(east);
    setVisitCounts(x+1, y, 1);
    setLocalMap(x+1, y, 0xF);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    int32_t mockX = getMockLocalX();
    int32_t mockY = getMockLocalY();

    CU_ASSERT_EQUAL(result.nextMove, MOVE);
    CU_ASSERT_EQUAL(getDesiredDir(), error);
    CU_ASSERT_EQUAL(getVisitCounts(mockX,mockY), 2);
    CU_ASSERT_EQUAL(getMockLocalX(), 12);
    CU_ASSERT_EQUAL(getMockLocalY(), 11);
    CU_ASSERT_NOT_EQUAL(return_state, DecideNextMove);
}

void test_t14_2() { // test transition from Move to DecideNextMove
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(Move);
    int32_t x = 11;
    int32_t y = 11;
    setMockLocalCord(x, y);
    setTurtleOrientation(south);
    setVisitCounts(x, y+1, 0);
    setLocalMap(x, y+1, 0x4);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    int32_t mockX = getMockLocalX();
    int32_t mockY = getMockLocalY();

    CU_ASSERT_EQUAL(result.nextMove, MOVE);
    CU_ASSERT_EQUAL(getDesiredDir(), error);
    CU_ASSERT_EQUAL(getVisitCounts(mockX,mockY), 1);
    CU_ASSERT_EQUAL(getMockLocalX(), 11);
    CU_ASSERT_EQUAL(getMockLocalY(), 12);
    CU_ASSERT_NOT_EQUAL(return_state, DecideNextMove);
}

void test_t14_3() { // test transition from Move to DecideNextMove
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(Move);
    int32_t x = 11;
    int32_t y = 11;
    setMockLocalCord(x, y);
    setTurtleOrientation(west);
    setVisitCounts(x-1, y, 2);
    setLocalMap(x-1, y, 0x1);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    int32_t mockX = getMockLocalX();
    int32_t mockY = getMockLocalY();

    CU_ASSERT_EQUAL(result.nextMove, MOVE);
    CU_ASSERT_EQUAL(getDesiredDir(), error);
    CU_ASSERT_EQUAL(getVisitCounts(mockX,mockY), 3);
    CU_ASSERT_EQUAL(getMockLocalX(), 10);
    CU_ASSERT_EQUAL(getMockLocalY(), 11);
    CU_ASSERT_EQUAL(return_state, DecideNextMove);
}

void test_t15() { // test transition from Move to Goal
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(Move);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();

    CU_ASSERT_EQUAL(result.nextMove, MOVE);
    CU_ASSERT_NOT_EQUAL(return_state, Goal);
}

void test_t15_1() { // test transition from Move to Goal
    mock_set_bump(false);
    mock_set_atend(true);
    setTurtleState(Move);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();

    CU_ASSERT_EQUAL(result.nextMove, MOVE);
    CU_ASSERT_EQUAL(return_state, Goal);
}

void test_t16() { // test transition from Move to CheckWall
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(Move);
    int32_t x = 11;
    int32_t y = 11;
    setMockLocalCord(x, y);
    setTurtleOrientation(west);
    setVisitCounts(x-1, y, 0);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    int32_t mockX = getMockLocalX();
    int32_t mockY = getMockLocalY();

    CU_ASSERT_EQUAL(result.nextMove, MOVE);
    CU_ASSERT_EQUAL(getDesiredDir(), error);
    CU_ASSERT_EQUAL(getVisitCounts(mockX,mockY), 1);
    CU_ASSERT_EQUAL(getMockLocalX(), 10);
    CU_ASSERT_EQUAL(getMockLocalY(), 11);
    CU_ASSERT_EQUAL(return_state, CheckWall);
}

void test_t16_1() { // test transition from Move to CheckWall
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(Move);
    int32_t x = 11;
    int32_t y = 11;
    setMockLocalCord(x, y);
    setTurtleOrientation(north);
    setVisitCounts(x, y-1, 0);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    int32_t mockX = getMockLocalX();
    int32_t mockY = getMockLocalY();

    CU_ASSERT_EQUAL(result.nextMove, MOVE);
    CU_ASSERT_EQUAL(getDesiredDir(), error);
    CU_ASSERT_EQUAL(getVisitCounts(mockX,mockY), 1);
    CU_ASSERT_EQUAL(getMockLocalX(), 11);
    CU_ASSERT_EQUAL(getMockLocalY(), 10);
    CU_ASSERT_EQUAL(return_state, CheckWall);
}


void test_t16_2() { // test transition from Move to CheckWall
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(Move);
    int32_t x = 11;
    int32_t y = 11;
    setMockLocalCord(x, y);
    setTurtleOrientation(north);
    setVisitCounts(x, y-1, 1);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    int32_t mockX = getMockLocalX();
    int32_t mockY = getMockLocalY();

    CU_ASSERT_EQUAL(result.nextMove, MOVE);
    CU_ASSERT_EQUAL(getDesiredDir(), error);
    CU_ASSERT_EQUAL(getVisitCounts(mockX,mockY), 2);
    CU_ASSERT_EQUAL(getMockLocalX(), 11);
    CU_ASSERT_EQUAL(getMockLocalY(), 10);
    CU_ASSERT_NOT_EQUAL(return_state, CheckWall);
}


int init() {
  // Any test initialization code goes here
  return 0;
}

int cleanup() {
  // Any test cleanup code goes here
  return 0;
}

/* Skeleton code from http://cunit.sourceforge.net/example.html */
int main() {

  CU_pSuite pSuite = NULL;

  /* initialize the CUnit test registry */
  if (CUE_SUCCESS != CU_initialize_registry())
    return CU_get_error();

  /* add a suite to the registry */
  pSuite = CU_add_suite("UNIT TEST #1", init, cleanup);
  if (NULL == pSuite) {
    CU_cleanup_registry();
    return CU_get_error();
  }

  /* add the tests to the suite */
  if (
        (NULL == CU_add_test(pSuite, "test of transition S1 -> S3", test_t1)) ||
        (NULL == CU_add_test(pSuite, "test of transition S1 -> S3", test_t1_1)) ||
        (NULL == CU_add_test(pSuite, "test of transition S1 -> S3", test_t1_2)) ||
        (NULL == CU_add_test(pSuite, "test of transition S1 -> S3", test_t1_3)) ||
        (NULL == CU_add_test(pSuite, "test of transition S3 -> S2", test_t2)) ||
        (NULL == CU_add_test(pSuite, "test of transition S3 -> S2", test_t2_1)) ||
        (NULL == CU_add_test(pSuite, "test of transition S3 -> S2", test_t2_2)) ||
        (NULL == CU_add_test(pSuite, "test of transition S3 -> S2", test_t2_3)) ||
        (NULL == CU_add_test(pSuite, "test of transition S3 -> S2", test_t2_3)) ||
        (NULL == CU_add_test(pSuite, "test of transition S3 -> S2", test_t2_4)) ||
        (NULL == CU_add_test(pSuite, "test of transition S3 -> S2", test_t2_5)) ||
        (NULL == CU_add_test(pSuite, "test of transition S3 -> S2", test_t2_6)) ||
        (NULL == CU_add_test(pSuite, "test of transition S2 -> S3", test_t3)) ||
        (NULL == CU_add_test(pSuite, "test of transition S2 -> S3", test_t3_1)) ||
        (NULL == CU_add_test(pSuite, "test of transition S2 -> S3", test_t3_2)) ||
        (NULL == CU_add_test(pSuite, "test of transition S2 -> S3", test_t3_3)) ||
        (NULL == CU_add_test(pSuite, "test of transition S2 -> S3", test_t3_4)) ||
        (NULL == CU_add_test(pSuite, "test of transition S3 -> S9", test_t4)) ||
        (NULL == CU_add_test(pSuite, "test of transition S3 -> S9", test_t4_1)) ||
        (NULL == CU_add_test(pSuite, "test of transition S3 -> S9", test_t4_2)) ||
        (NULL == CU_add_test(pSuite, "test of transition S3 -> S4", test_t5)) ||
        (NULL == CU_add_test(pSuite, "test of transition S3 -> S4", test_t5_1)) ||
        (NULL == CU_add_test(pSuite, "test of transition S4 -> S9", test_t6)) ||
        (NULL == CU_add_test(pSuite, "test of transition S4 -> S9", test_t6_1)) ||
        (NULL == CU_add_test(pSuite, "test of transition S4 -> S9", test_t6_2)) ||
        (NULL == CU_add_test(pSuite, "test of transition S4 -> S6", test_t7)) ||
        (NULL == CU_add_test(pSuite, "test of transition S4 -> S7", test_t8)) ||
        (NULL == CU_add_test(pSuite, "test of transition S4 -> S5", test_t9)) ||
        (NULL == CU_add_test(pSuite, "test of transition S4 -> S8", test_t10)) ||
        (NULL == CU_add_test(pSuite, "test of transition S7 -> S6", test_t11)) ||
        (NULL == CU_add_test(pSuite, "test of transition S6 -> S5", test_t12)) ||
        (NULL == CU_add_test(pSuite, "test of transition S8 -> S5", test_t13)) ||
        (NULL == CU_add_test(pSuite, "test of transition S5 -> S4", test_t14)) ||
        (NULL == CU_add_test(pSuite, "test of transition S5 -> S4", test_t14_1)) ||
        (NULL == CU_add_test(pSuite, "test of transition S5 -> S4", test_t14_2)) ||
        (NULL == CU_add_test(pSuite, "test of transition S5 -> S4", test_t14_3)) ||
        (NULL == CU_add_test(pSuite, "test of transition S5 -> S9", test_t15)) ||
        (NULL == CU_add_test(pSuite, "test of transition S5 -> S9", test_t15_1)) ||
        (NULL == CU_add_test(pSuite, "test of transition S5 -> S3", test_t16)) ||
        (NULL == CU_add_test(pSuite, "test of transition S5 -> S3", test_t16_1)) ||
        (NULL == CU_add_test(pSuite, "test of transition S5 -> S3", test_t16_2)) ||
        (NULL == CU_add_test(pSuite, "test of transition S5 -> S3", test_t17)))


    {
      CU_cleanup_registry();
      return CU_get_error();
    }
  
  /* Run all tests using the CUnit Basic interface */
  CU_basic_set_mode(CU_BRM_VERBOSE);
  CU_basic_run_tests();
  CU_cleanup_registry();
  return CU_get_error();

  return 0;
}
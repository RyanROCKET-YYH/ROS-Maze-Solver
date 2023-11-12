#include "yuhongy_student_mock.h"
#include <CUnit/Basic.h>
#include <cstdint>

void test_t1() { // test transition from initialized to checkWall
    mock_set_bump(false);
    mock_set_atend(false);
    setTurtleState(Initialized);
    turtleResult result = studentTurtleStep(will_bump(), at_end());
    TurtleState return_state = getTurtleState();
    int32_t x = 11;
    int32_t y = 11;

    CU_ASSERT_EQUAL(result.nextMove, STOP);
    CU_ASSERT_EQUAL(getVisitCounts(x, y), 1);
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
    CU_ASSERT_EQUAL(return_state, Right);
}

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
  if ((NULL == CU_add_test(pSuite, "test of transition S1 -> S3", test_t1)) ||
      (NULL == CU_add_test(pSuite, "test of transition S3 -> S2", test_t2)) ||
      (NULL == CU_add_test(pSuite, "test of transition S3 -> S2", test_t2_1)) ||
      (NULL == CU_add_test(pSuite, "test of transition S3 -> S2", test_t2_2)) ||
      (NULL == CU_add_test(pSuite, "test of transition S3 -> S2", test_t2_3)) ||
      (NULL == CU_add_test(pSuite, "test of transition S3 -> S2", test_t2_3)) ||
      (NULL == CU_add_test(pSuite, "test of transition S2 -> S3", test_t3)) ||
      (NULL == CU_add_test(pSuite, "test of transition S2 -> S3", test_t3_1)) ||
      (NULL == CU_add_test(pSuite, "test of transition S2 -> S3", test_t3_2)) ||
      (NULL == CU_add_test(pSuite, "test of transition S2 -> S3", test_t3_3)) ||
      (NULL == CU_add_test(pSuite, "test of transition S3 -> S4", test_t4)) ||
      (NULL == CU_add_test(pSuite, "test of transition S3 -> S4", test_t4_1)))
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
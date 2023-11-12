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
      (NULL == CU_add_test(pSuite, "test of transition S3 -> S2", test_t2)))
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
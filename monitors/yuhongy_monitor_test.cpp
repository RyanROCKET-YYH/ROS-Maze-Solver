/*
 * STUDENT NAME: Yuhong YAO
 * ANDREW ID: yuhongy
 * 
 */

#include <CUnit/CUnit.h>
#include <CUnit/Basic.h>
#include "yuhongy_mock.h"


extern void poseInterrupt(ros::Time t, int x, int y, Orientation o);

void test_turn_monitor() {
    // Test that the poseInterrupt function works
    // by checking that the last_orientation is updated
    // and that the pose is printed
    reset_ros_warn();
    poseInterrupt(ros::Time(0), 3, 3, NORTH);
    poseInterrupt(ros::Time(0), 3, 3, NORTH);
    
    CU_ASSERT_NOT_TRUE(was_ros_warn());
}

void test_turn_monitor_1() {
    reset_ros_warn();
    poseInterrupt(ros::Time(0), 3, 3, NORTH);
    poseInterrupt(ros::Time(0), 3, 3, SOUTH);
    
    CU_ASSERT_TRUE(was_ros_warn());
}

void test_turn_monitor_2() {
    reset_ros_warn();
    poseInterrupt(ros::Time(0), 3, 3, NORTH);
    poseInterrupt(ros::Time(0), 3, 3, EAST);
    
    CU_ASSERT_NOT_TRUE(was_ros_warn());
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
        (NULL == CU_add_test(pSuite, "test of turn monitor", test_turn_monitor)) ||
        (NULL == CU_add_test(pSuite, "test of turn monitor 1", test_turn_monitor_1)) ||
        (NULL == CU_add_test(pSuite, "test of turn monitor 2", test_turn_monitor_2))
        )


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
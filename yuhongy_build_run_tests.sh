#!/bin/bash
cd student

COMPILER=g++
STD=-std=c++11
CPPFLAGS=-Dtesting
OUTPUT=-o yuhongy_student_tests
SOURCES="yuhongy_student_test.cpp yuhongy_student_turtle.cpp yuhongy_mock_functions.cpp"
LIBRARIES=-lcunit

$COMPILER $STD $CPPFLAGS $OUTPUT $SOURCES $LIBRARIES

if [ $? -eq 0 ]; then
    # Run the tests
    ./yuhongy_student_tests

    exit $?
else
    echo "Compilation failed."
    exit 1
fi
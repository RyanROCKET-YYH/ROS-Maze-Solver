#!/bin/bash
cd student

COMPILER=g++
STD=-std=c++11
CPPFLAGS=-Dtesting
OUTPUT=yuhongy_student_tests
SOURCES="yuhongy_student_test.cpp yuhongy_student_turtle.cpp yuhongy_mock_functions.cpp"
LIBRARIES=-lcunit

$COMPILER $STD $CPPFLAGS -o $OUTPUT $SOURCES $LIBRARIES

if [ $? -eq 0 ]; then
    # Run the tests
    ./yuhongy_student_tests
    cd ..
    exit $?
else
    echo "Compilation failed."
    cd ..
    exit 1
fi
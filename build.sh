#!/bin/bash

LINK="-lIrrlicht"
SRCS="main.cpp types.cpp util.cpp"
INC=""
CPP_FLAGS="$1"

g++ -std=c++11 ${CPP_FLAGS} ${INC} ${SRCS} ${LINK} -o RigidVoxels

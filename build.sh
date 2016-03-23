#!/bin/bash

LINK="-lIrrlicht"
SRCS="main.cpp types.cpp"
INC=""

g++ -std=c++11 ${INC} ${SRCS} ${LINK} -o RigidVoxels

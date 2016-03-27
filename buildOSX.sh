#!/bin/bash

LINK="-lIrrlicht -lglfw3"
SRCS="main.cpp types.cpp util.cpp"
INC=""

g++ -std=c++11 ${INC} ${SRCS} ${LINK} -o RigidVoxels -framework OpenGL -framework Cocoa -framework IOKit

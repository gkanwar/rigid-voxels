#!/bin/bash

LINK="-lcairo `pkg-config --libs gtk+-2.0`"
SRCS="main.cpp types.cpp"
INC="`pkg-config --cflags gtk+-2.0`"

g++ -std=c++11 ${INC} ${SRCS} ${LINK} -o RigidVoxels

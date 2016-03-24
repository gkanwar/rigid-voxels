#ifndef UTIL_H
#define UTIL_H

#include <irrlicht/irrlicht.h>
#include <iostream>

using namespace std;
using namespace irr::core;

ostream& operator<<(ostream& os, vector3df v);
ostream& operator<<(ostream& os, vector3di v);

#endif

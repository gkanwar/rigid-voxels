#include "util.h"

ostream& operator<<(ostream& os, vector3df v) {
  os << "(" << v.X << "," << v.Y << "," << v.Z << ")";
  return os;
}

ostream& operator<<(ostream& os, vector3di v) {
  os << "(" << v.X << "," << v.Y << "," << v.Z << ")";
  return os;
}

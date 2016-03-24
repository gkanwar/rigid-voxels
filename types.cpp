#include "types.h"

using namespace irr::core;

void Collision::applyForces() {
  // Ignore internal collisions
  if (p1->parent == p2->parent) return;
  
  // R is vector from p2 to p1
  vector3df r = p1->x - p2->x;
  double rad = r.getLength();
  vector3df rhat = r.normalize();

  vector3df f1(0,0,0);
  vector3df f2(0,0,0);
    
  // Spring model
  double spMag = -k*(PART_D - rad);
  f1 -= spMag*rhat;
  f2 += spMag*rhat;

  // Damping model
  f1 += p1->v*eta;
  f2 += p2->v*eta;

  // Shear force
  double vrdot1 = p1->v.dotProduct(rhat);
  double vrdot2 = p2->v.dotProduct(rhat);
  vector3df vt1 = p1->v - vrdot1*rhat;
  vector3df vt2 = p2->v - vrdot2*rhat;
  f1 -= kt*vt1;
  f2 += kt*vt2;

  // Update parent forces
  p1->parent->f += f1;
  p2->parent->f += f2;
  int p1i = p1->index;
  int p2i = p2->index;
  // Torques
  vector3df p1r = p1->x - p1->parent->x;
  vector3df p2r = p2->x - p2->parent->x;
  p1->parent->t += p1r.crossProduct(f1);
  p2->parent->t += p2r.crossProduct(f2);
}


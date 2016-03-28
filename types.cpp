#include "types.h"

using namespace irr::core;


void Collision::applyForces() {
  int collType = o1->getType() | o2->getType();
  if (collType == PART) {
    applyPartPart();
  }
  else if (collType == (PART | PLANE)) {
    applyPlanePart();
  }
}

void Collision::applyPartPart() {
  Particle* p1 = (Particle *) o1;
  Particle* p2 = (Particle *) o2;

  // Ensure particles actually intersect
  vector3df d = p1->pos - p2->pos;
  double dSq = d.getLengthSQ();
  if (dSq > PART_D*PART_D) {
    return;
  }

  // Ignore internal collisions
  if (p1->parent == p2->parent) return;

  // R is vector from p2 to p1
  vector3df r = p1->pos - p2->pos;
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
  vector3df p1r = p1->pos - p1->parent->pos;
  vector3df p2r = p2->pos - p2->parent->pos;
  p1->parent->t += p1r.crossProduct(f1);
  p2->parent->t += p2r.crossProduct(f2);
}

void Collision::applyPlanePart() {

  //cout << "plane part collision" << endl;
  Particle* part;
  Plane* plane;

  if (o1->getType() == PLANE) {
    part = (Particle *) o2;
    plane = (Plane *) o1;
  }
  else {
    part = (Particle *) o1;
    plane = (Plane *) o2;
  }

  // Don't bother colliding with fixed objects
  if (part->parent->fixed) {
    return;
  }

  // diff is vector from plane center to particle center
  vector3df diff = plane->pos - part->pos;


  // plane is defined by norm . (x, y, z)  = d with x, y, z relative to point
  double d = diff.dotProduct(plane->norm);

  double normLength = (plane->norm.getLength());

  vector3df closestPoint = (plane->norm * d) / plane->norm.getLengthSQ();

  // Put closestPoint back into world space
  closestPoint += part->pos;

  // R is vector from plane to part
  vector3df r = part->pos - closestPoint;

  // If not actually colliding, return
  if (r.getLengthSQ() > PART_D*PART_D) {
    return;
  }
  double rad = r.getLength();
  vector3df rhat = r.normalize();

  vector3df f(0,0,0);

  // Spring model
  double spMag = -k*(PART_D - rad);
  f -= spMag*rhat;

  // Damping model
  f += part->v*eta;

  // Shear force
  double vrdot = part->v.dotProduct(rhat);
  vector3df vt = part->v - vrdot*rhat;

  f -= kt*vt;

  // Update parent forces
  part->parent->f += f;

  int pi = part->index;
  // Torques
  vector3df pr = part->pos - part->parent->pos;
  part->parent->t += pr.crossProduct(f);
}


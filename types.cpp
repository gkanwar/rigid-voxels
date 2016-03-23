#include "types.h"

void Collision::applyForces() {
  // Ignore internal collisions
  if (p1->parent == p2->parent) return;
  
  // R is vector from p2 to p1
  double r_x = p1->x - p2->x;
  double r_y = p1->y - p2->y;
  double rad = sqrt(r_x*r_x + r_y*r_y);
  double r_xhat = r_x/rad;
  double r_yhat = r_y/rad;

  double p1fx = 0.0;
  double p1fy = 0.0;
  double p2fx = 0.0;
  double p2fy = 0.0;
    
  // Spring model
  double spMag = -k*(PART_D - rad);
  p1fx -= spMag*r_xhat;
  p1fy -= spMag*r_yhat;
  p2fx += spMag*r_xhat;
  p2fy += spMag*r_yhat;

  // Damping model
  p1fx += p1->vx*eta;
  p1fy += p1->vy*eta;
  p2fx += p2->vx*eta;
  p2fy += p2->vy*eta;

  // TODO: Shear force

  // Update parent forces
  p1->parent->fx += p1fx;
  p1->parent->fy += p1fy;
  p2->parent->fx += p2fx;
  p2->parent->fy += p2fy;
  int p1i = p1->index;
  int p2i = p2->index;
  // Local rotated coords
  double p1lx = p1->x - p1->parent->x;
  double p1ly = p1->y - p1->parent->y;
  double p2lx = p2->x - p2->parent->x;
  double p2ly = p2->y - p2->parent->y;
  
  p1->parent->t += p1lx*p1fy - p1ly*p1fx;
  p2->parent->t += p2lx*p2fy - p2ly*p2fx;
}


#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>
#include <map>
#include <unistd.h>

#include "types.h"

// Drawing
#include "vdb.h"

using namespace std;

void pt(double x, double y, double z) {
  vdb_point(x,y,z);
}

bool fc(double f1, double f2) {
  return fabs(f1-f2) < 0.0001;
}

int main() {

  // Test position pushing
  // Obj o1, o2;
  // o1.addPart(0.0, 0.0);
  // o2.addPart(1.0, 1.0);

  // o1.x = 1.0;
  // o1.y = 2.0;
  // o1.push();
  // assert(o1.parts[0]->x == 1.0);
  // assert(o1.parts[0]->y == 2.0);
  // assert(o1.parts[0]->vx == 0.0);
  // assert(o1.parts[0]->vy == 0.0);

  // o1.vx = 3.0;
  // o1.vy = 4.0;
  // o1.push();
  // assert(o1.parts[0]->x == 1.0);
  // assert(o1.parts[0]->y == 2.0);
  // assert(o1.parts[0]->vx == 3.0);
  // assert(o1.parts[0]->vy == 4.0);

  // o2.x = 1.0;
  // o2.y = 2.0;
  // o2.theta = M_PI/2.0;
  // o2.push();
  // assert(o2.parts[0]->x == 0.0);
  // assert(o2.parts[0]->y == 3.0);

  // o2.w = 1.0;
  // o2.push();
  // assert(o2.parts[0]->x == 0.0);
  // assert(o2.parts[0]->y == 3.0);
  // assert(fc(o2.parts[0]->vx, -1.0));
  // assert(fc(o2.parts[0]->vy, -1.0));


  // Drop o1 onto o2 (1-part each)
  Obj o1, o2;
  o1.addPart(0.0, 0.0);
  o1.addPart(1.0, 0.0);
  o2.addPart(0.0, 0.5);
  o2.addPart(0.0, -0.5);
  o1.y = 3.0;
  o1.vy = -1.0;
  o2.fixed = true;

  Voxels v;

  const double ts = 0.1;

  // LOOP
  int iter = 0;
  while (true) {
    usleep(100000);

    // Step 0: Init objs
    v.voxels.clear();
    o1.clearStepVals();
    o2.clearStepVals();
  
    // Step 1: Push obj state into particles
    o1.push();
    o2.push();
  
    // Step 2: Run through particles and stick into voxels
    o1.dumpIntoVoxels(v);
    o2.dumpIntoVoxels(v);
  
    // Step 3: Detect collisions, compute forces, add these to object
    vector<Collision> cs;
    v.findCollisions(cs);
    for (Collision &c : cs) {
      c.applyForces();
    }
  
    // Step 4: Integrate forces
    cout << "o1:" << endl;
    o1.integrateForce(ts);
    cout << "o2:" << endl;
    o2.integrateForce(ts);
  
    // Step 5: Integrate velocities
    o1.integrateVel(ts);
    o2.integrateVel(ts);

    cout << "o1 vy: " << o1.vy << endl;
    cout << "o2 vy: " << o2.vy << endl;
    if (iter % 10 == 0) {
      o1.draw(iter/10.0, &pt);
      o2.draw(iter/10.0, &pt);
    }
    ++iter;
  }
}

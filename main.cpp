#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>
#include <map>
#include <unistd.h>

#include "types.h"

// Drawing
#include "vdb.h"
#include "idraw.h"

using namespace std;

// Draw backend options
enum Draw { Vdb, Irr };

void pt(double x, double y, double z) {
  vdb_point(x,y,z);
}
void ln(double x, double y, double z,
        double x2, double y2, double z2) {
  vdb_line(x,y,z,x2,y2,z2);
}

bool fc(double f1, double f2) {
  return fabs(f1-f2) < 0.0001;
}

int main(int argc, char** argv) {
  Draw draw = Draw::Vdb;
  if (argc > 1) {
    if (strcmp(argv[1], "vdb") == 0) {
      draw = Draw::Vdb;
      cout << "Using vdb draw backend." << endl;
    }
    else if (strcmp(argv[1], "irr") == 0) {
      draw = Draw::Irr;
      cout << "Using irr draw backend." << endl;
    }
    else {
      cerr << "Unknown draw backend, picking vdb." << endl;
    }
  }
  else {
    cout << "Using vdb draw backend by default." << endl;
  }

  // Init drawing
  if (draw == Draw::Irr) {
    int err = idraw::init();
    if (err) {
      cerr << "Irrlicht init failed with: " << err << endl;
      return err;
    }
  }

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
  o1.addPart(0.0, 0.5);
  o1.addPart(0.0, -0.5);
  o1.theta = M_PI/2;
  
  o2.addPart(0.5, 0.5);
  o2.addPart(0.5, -0.5);
  o1.y = 3.0;
  o1.vy = -1.0;
  o2.fixed = true;

  // Add objects to draw backend
  if (draw == Draw::Irr) {
    idraw::addObj(&o1);
    idraw::addObj(&o2);
  }

  Voxels v;

  const double ts = 0.01;

  // LOOP
  int iter = 0;
  while (true) {
    usleep(10000);

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


    // Draw
    if (draw == Draw::Vdb) {
      if (iter % 10 == 0) {
        o1.push();
        o1.draw(iter/10.0, &pt, &ln);
        o2.push();
        o2.draw(iter/10.0, &pt, &ln);
      }
    }
    else if (draw == Draw::Irr) {
      int ret = idraw::step();
      if (ret) break;
    }
    
    ++iter;
  }

  // Cleanup
  if (draw == Draw::Irr) {
    idraw::cleanup();
  }
}

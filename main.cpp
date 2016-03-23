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
  Obj o1, o2, o3, o4;
  o1.addPart(0.0, 0.5);
  o1.addPart(0.0, -0.5);
  o1.theta = M_PI/2;
  o1.y = 3.0;
  o1.vy = -1.0;
  
  o2.addPart(0.5, 0.5);
  o2.addPart(0.5, -0.5);
  o2.fixed = true;

  o3.addPart(0.0, 0.5);
  o3.addPart(0.0, -0.5);
  o3.theta = M_PI/4;
  o3.y = 5.0;
  o3.vy = -0.5;

  o4.addPart(0.0, 0.5);
  o4.addPart(0.0, -0.5);
  o4.theta = -M_PI/4;
  o4.y = 6.0;
  o4.vy = 0.0;

  vector<Obj*> objects = {&o1, &o2, &o3, &o4};

  
  // Add objects to draw backend
  if (draw == Draw::Irr) {
    for (Obj* o : objects) {
      idraw::addObj(o);
    }
  }

  Voxels v;

  const double ts = 0.01;

  // LOOP
  int iter = 0;
  while (true) {
    usleep(10000);

    // Step 0: Init objs
    v.voxels.clear();
    for (Obj* o : objects) {
      o->clearStepVals();
    }
  
    // Step 1: Push obj state into particles
    for (Obj* o : objects) {
      o->push();
    }
  
    // Step 2: Run through particles and stick into voxels
    for (Obj* o : objects) {
      o->dumpIntoVoxels(v);
    }
  
    // Step 3: Detect collisions, compute forces, add these to object
    vector<Collision> cs;
    v.findCollisions(cs);
    for (Collision &c : cs) {
      c.applyForces();
    }
  
    // Step 4: Integrate forces
    for (Obj* o : objects) {
      o->integrateForce(ts);
    }
  
    // Step 5: Integrate velocities
    for (Obj* o : objects) {
      o->integrateVel(ts);
    }


    // Draw
    if (draw == Draw::Vdb) {
      if (iter % 10 == 0) {
        for (Obj* o : objects) {
          o->push();
          o->draw(iter/10.0, &pt, &ln);
        }
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

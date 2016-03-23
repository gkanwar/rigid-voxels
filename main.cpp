#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>
#include <map>
#include <unistd.h>

#include "main.h"

using namespace std;

#define PART_D 0.5
#define SIZE 100 // 10.0 x 10.0 region

#define k 10.0
#define eta 0.01
#define kt 0.1

struct Voxels {
  map< pair<int,int>, vector<Particle*> > voxels;
  double xbase=-5.0, ybase=-3.0;
  double size=PART_D;

  void findCollisions(vector<Collision> &out) {
    for (auto &kv : voxels) {
      if (kv.second.size() > 1) {
        // Collision!
        cout << "Found collision: " << kv.first.first << "," << kv.first.second << endl;
        assert(kv.second.size() == 2); // FOR NOW
        Collision c;
        c.p1 = kv.second[0];
        c.p2 = kv.second[1];
        out.push_back(c);
      }
    }
  }
};

struct Obj {
  vector<Particle*> parts;
  vector< pair<double,double> > locs;
  
  double x=0.0, y=0.0; // Linear pos
  double vx=0.0, vy=0.0; // Linear velocity
  double theta=0.0; // Angular pos
  double w=0.0; // Angular velocity

  double fx=0.0, fy=0.0; // Step force
  double t=0.0; // Torque

  bool fixed = false;

  // Integrate steps
  void integrateForce(double ts) {
    if (fixed) return;
    cout << "Integrating force " << fx << "," << fy << endl;
    vx += fx*ts;
    vy += fy*ts;
    w += t*ts;
  }

  void integrateVel(double ts) {
    if (fixed) return;
    x += vx*ts;
    y += vy*ts;
    theta += w*ts;
  }

  // Clear between steps
  void clearStepVals() {
    fx = 0.0;
    fy = 0.0;
    t = 0.0;
  }
  
  // Push velocities/positions into parts
  void push() {
    assert(locs.size() == parts.size());
    for (int i = 0; i < locs.size(); ++i) {
      Particle *p = parts[i];
      pair<double,double> loc = locs[i];
      p->x = x + cos(theta)*loc.first - sin(theta)*loc.second;
      p->y = y + sin(theta)*loc.first + cos(theta)*loc.second;
      double rad = sqrt(loc.first*loc.first + loc.second*loc.second);
      double locTheta = atan2(loc.second, loc.first)+theta+M_PI/2.0;
      p->vx = vx + cos(locTheta)*rad*w;
      p->vy = vy + sin(locTheta)*rad*w;
    }
  }

  void dumpIntoVoxels(Voxels &v) {
    for (Particle *p : parts) {
      int xi = (int)((p->x-v.xbase) / v.size);
      int yi = (int)((p->y-v.ybase) / v.size);
      assert(xi < SIZE);
      assert(yi < SIZE);
      v.voxels[make_pair(xi,yi)].push_back(p);
      cout << "Dumped part into : " << xi << "," << yi << endl;
    }
  }

  void addPart(double lx, double ly) {
    Particle *p = new Particle();
    p->parent = this;
    p->index = locs.size();
    parts.push_back(p);
    locs.emplace_back(lx, ly);
  }
};

bool fc(double f1, double f2) {
  return fabs(f1-f2) < 0.0001;
}

void Collision::applyForces() {
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
  // int p1i = p1->index;
  // int p2i = p2->index;
  // pair<double,double> p1l = p1->parent->locs[p1i];
  // double p1r = sqrt(p1l.first*p1l.first + p1l.second*p1l.second);
  // pair<double,double> p2l = p2->parent->locs[p2i];
  // double p2r = sqrt(p2l.first*p2l.first + p2l.second*p2l.second);
  // p1->parent->t += p1r*
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
  o2.addPart(0.0, 0.0);
  o1.y = 3.0;
  o1.vy = -1.0;
  o2.fixed = true;

  Voxels v;

  const double ts = 0.01;

  // LOOP
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

    cout << "o1 vy: " << o1.vy << endl;
    cout << "o2 vy: " << o2.vy << endl;
  }
}

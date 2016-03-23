#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <map>
#include <cmath>
#include <cassert>
#include <iostream>

using namespace std;

#define PART_D 0.5
#define SIZE 100 // 10.0 x 10.0 region

#define k 10.0
#define eta 0.01
#define kt 0.1

struct Obj;

struct Particle {
  double x=0.0, y=0.0;
  double vx=0.0, vy=0.0;
  Obj *parent;
  int index;
};

struct Collision {
  // TODO: Support multi-part collision
  Particle *p1;
  Particle *p2;

  void applyForces();
};

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

#endif
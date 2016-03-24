#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <map>
#include <cmath>
#include <cassert>
#include <iostream>

#include <irrlicht/irrlicht.h>

#include "util.h"

using namespace std;
using namespace irr::core;

#define PART_D 0.5
#define SIZE 100 // 10.0 x 10.0 x 10.0 region

// Enclose all constant params
namespace {
const double k = 10.0;
const double eta = 0.01;
const double kt = 0.1;
} // anonymous namespace

struct Obj;

struct Particle {
  vector3df x;
  vector3df v;
  Obj *parent;
  int index;

  // Draw debugging
  vector3df l;
  bool last = false;
  void draw(double z, void (*pt)(double,double,double),
            void (*ln)(double,double,double,double,double,double)) {
    pt(x.X, x.Y, x.Z);
    if (last) {
      ln(x.X,x.Y,x.Z,l.X,l.Y,l.Z);
    }
    last = true;
    l = x;
  }
};

struct Collision {
  // TODO: Support multi-part collision
  Particle *p1;
  Particle *p2;

  void applyForces();
};

struct Voxels {
  std::map< vector3di, vector<Particle*> > voxels;
  double xbase=-5.0, ybase=-3.0, zbase=-5.0;
  double size=PART_D;

  void findCollisions(vector<Collision> &out) {
    for (auto &kv : voxels) {
      if (kv.second.size() > 1) {
        // Collision!
        cout << "Found collision: " << kv.first << endl;
        for (int i = 0; i < kv.second.size(); ++i) {
          for (int j = i+1; j < kv.second.size(); ++j) {
            Particle *p1 = kv.second[0];
            Particle *p2 = kv.second[1];
            vector3df d = p1->x - p2->x;
            double dSq = d.getLengthSQ();
            if (dSq < PART_D*PART_D) {
              Collision c;
              c.p1 = kv.second[0];
              c.p2 = kv.second[1];
              out.push_back(c);
            }
          }
        }
      }
      else if (kv.second.size() == 1) {
        Particle *p1 = kv.second[0];
        // Search adjacent
        for (int i = -1; i <= 1; ++i) {
          for (int j = -1; j <= 1; ++j) {
            for (int k = -1; k <= 1; ++k) {
              if (i == 0 && j == 0 && k == 0) continue;
              auto loc = vector3di(kv.first.X+i, kv.first.Y+j, kv.first.Z+k);
              if (voxels.count(loc)) {
                for (Particle *p2 : voxels[loc]) {
                  vector3df d = p1->x - p2->x;
                  double dSq = d.getLengthSQ();
                  if (dSq < PART_D*PART_D) {
                    // Collision!
                    Collision c;
                    c.p1 = p1;
                    c.p2 = p2;
                    out.push_back(c);
                  }
                }
              }
            }
          }
        }
      }
    }
  }
};

struct Obj {
  vector<Particle*> parts;
  vector< vector3df > locs;
  
  vector3df x; // Linear pos
  vector3df v; // Linear velocity
  quaternion theta; // Angular pos
  vector3df w; // Angular velocity

  vector3df f; // Step force
  vector3df t; // Torque

  bool fixed = false;

  // Integrate steps
  void integrateForce(double ts) {
    if (fixed) return;
    cout << "Integrating force " << f << "; " << t << endl;
    v += f*ts;
    w += t*ts;
  }

  void integrateVel(double ts) {
    if (fixed) return;
    x += v*ts;
    // Compose quaterion for current rotation with new rotation
    vector3df dtheta = w*ts;
    double angle = dtheta.getLength();
    dtheta.normalize();
    quaternion dthetaq;
    dthetaq.fromAngleAxis((float)angle, dtheta);
    theta = dthetaq*theta;
  }

  // Clear between steps
  void clearStepVals() {
    f = vector3df(0,0,0);
    t = vector3df(0,0,0);
  }
  
  // Push velocities/positions into parts
  void push() {
    assert(locs.size() == parts.size());
    for (int i = 0; i < locs.size(); ++i) {
      Particle *p = parts[i];
      vector3df loc = locs[i];
      quaternion rlocq(loc.X, loc.Y, loc.Z, 0);
      quaternion thetaInv = theta;
      thetaInv.makeInverse();
      rlocq = thetaInv*rlocq*theta;
      vector3df rloc(rlocq.X, rlocq.Y, rlocq.Z);
      p->x = x + rloc;

      p->v = v;
      if (w.getLengthSQ() > 0.0) {
        vector3df tangent = w.crossProduct(rloc);
        tangent.normalize();
        vector3df norm = rloc - rloc.dotProduct(w)*w / w.getLengthSQ();
        p->v += norm.getLength() * w.getLength() * tangent;
      }
    }
  }

  void dumpIntoVoxels(Voxels &v) {
    for (Particle *p : parts) {
      int xi = (int)((p->x.X-v.xbase) / v.size);
      int yi = (int)((p->x.Y-v.ybase) / v.size);
      int zi = (int)((p->x.Z-v.zbase) / v.size);
      // assert(xi < SIZE);
      // assert(yi < SIZE);
      // assert(zi < SIZE);
      v.voxels[vector3di(xi,yi,zi)].push_back(p);
      cout << "Dumped part into : " << xi << "," << yi << endl;
    }
  }

  void addPart(vector3df l) {
    Particle *p = new Particle();
    p->parent = this;
    p->index = locs.size();
    parts.push_back(p);
    locs.push_back(l);
  }

  void draw(double z, void (*pt)(double,double,double),
            void (*ln)(double,double,double,double,double,double)) {
    for (Particle *p : parts) {
      p->draw(z, pt, ln);
    }
  }
};

#endif

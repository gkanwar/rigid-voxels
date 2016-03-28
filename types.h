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

enum CollType {
  PART = 1,
  PLANE = PART << 1,
};

class CollObj {
public:
  vector3df pos;
  virtual CollType getType() = 0;
};

struct Collision {
  // TODO: Support multi-part collision
  CollObj *o1;
  CollObj *o2;

  void applyForces();
private:
  void applyPartPart();
  void applyPlanePart();
};

struct Voxels {
  std::map< vector3di, vector<CollObj*> > voxels;

  double xbase=0.0, ybase=0.0, zbase=0.0;
  double size=PART_D;

  void findCollisions(vector<Collision> &out) {
    for (auto &kv : voxels) {
      if (kv.second.size() > 1) {
        // Collision!
        //cout << "Found collision: " << kv.first << endl;
        for (int i = 0; i < kv.second.size(); ++i) {
          for (int j = i+1; j < kv.second.size(); ++j) {
            CollObj *o1 = kv.second[0];
            CollObj *o2 = kv.second[1];

            Collision c;
            c.o1 = kv.second[0];
            c.o2 = kv.second[1];
            out.push_back(c);
          }
        }
      }
      else if (kv.second.size() == 1) {
        CollObj *o1 = kv.second[0];
        if (o1->getType() != PART) {
          // TODO: do planes need to check adjacent?
          continue;
        }
        // Search adjacent
        for (int i = -1; i <= 1; ++i) {
          for (int j = -1; j <= 1; ++j) {
            for (int k = -1; k <= 1; ++k) {
              if (i == 0 && j == 0 && k == 0) continue;
              auto loc = vector3di(kv.first.X+i, kv.first.Y+j, kv.first.Z+k);
              if (voxels.count(loc)) {
                for (CollObj *o2 : voxels[loc]) {
                  vector3df d = o1->pos - o2->pos;
                  double dSq = d.getLengthSQ();
                  if (dSq < PART_D*PART_D) {
                    // Collision!
                    Collision c;
                    c.o1 = o1;
                    c.o2 = o2;
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

struct Particle : CollObj {
  vector3df v;
  Obj *parent;
  int index;

  // Draw debugging
  vector3df l;
  bool last = false;
  void draw(double z, void (*pt)(double,double,double),
            void (*ln)(double,double,double,double,double,double)) {
    pt(pos.X, pos.Y, pos.Z);
    if (last) {
      ln(pos.X,pos.Y,pos.Z,l.X,l.Y,l.Z);
    }
    last = true;
    l = pos;
  }

  CollType getType() {
    return PART;
  }
};

struct Plane : CollObj {
  double width;
  double height;
  vector3df norm;
  vector3df right;

  CollType getType() {
    return PLANE;
  }

  void dumpIntoVoxels(Voxels &vox) {
    vector3df up = norm.crossProduct(right);

    for (int i = -1 * (width/2/vox.size) - 1; i <= width/2/vox.size; i++) {
      for (int j = -1 * (height/2/vox.size) -1; j <= height/2/vox.size; j++) {

        int cx = pos.X + up.X*i*vox.size + right.X*j*vox.size;
        int cy = pos.Y + up.Y*i*vox.size + right.Y*j*vox.size;
        int cz = pos.Z + up.Z*i*vox.size + right.Z*j*vox.size;

        int xi = (int)((cx-vox.xbase)/vox.size);
        int yi = (int)((cy-vox.ybase)/vox.size);
        int zi = (int)((cz-vox.zbase)/vox.size);

        vector3di check = vector3di(xi,yi,zi);

        for (int dx = -1; dx <= 1; dx++) {
          for (int dy = -1; dy <= 1; dy++) {
            for (int dz = -1; dz <= 1; dz++) {
              check.X = xi + dx;
              check.Y = yi + dy;
              check.Z = zi + dz;

              // Planes don't collide with each other so we only fill in voxels
              // when it would cause a collision
              if (vox.voxels[check].size() > 0) {
                vox.voxels[check].push_back(this);
              }
            }
          }
        }
      }
    }
  }

  void addToScene(irr::scene::ISceneManager *smgr) {
    vector3df up = norm.crossProduct(right);

    double size = PART_D/2;

    for (int i = -1 * (width/2/size) - 1; i <= width/2/size; i++) {
      for (int j = -1 * (height/2/size) -1; j <= height/2/size; j++) {
        double cx = pos.X/size + up.X*i + right.X*j;
        double cy = pos.Y/size + up.Y*i + right.Y*j;
        double cz = pos.Z/size + up.Z*i + right.Z*j;

        cx *= size;
        cy *= size;
        cz *= size;

        smgr->addSphereSceneNode(PART_D/4)->setPosition(vector3df(cx,cy,cz));
      }
    }
  }
};


struct Obj {
  vector<Particle*> parts;
  vector< vector3df > locs;

  vector3df pos; // Linear pos
  vector3df v; // Linear velocity
  quaternion theta; // Angular pos
  vector3df w; // Angular velocity

  vector3df f; // Step force
  vector3df t; // Torque

  bool fixed = false;

  // Integrate steps
  void integrateForce(double ts) {
    if (fixed) return;
    //cout << "Integrating force " << f << "; " << t << endl;
    v += f*ts;
    w += t*ts;
  }

  void integrateVel(double ts) {
    if (fixed) return;
    pos += v*ts;
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
      p->pos =pos + rloc;

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
      int xi = (int)((p->pos.X-v.xbase) / v.size);
      int yi = (int)((p->pos.Y-v.ybase) / v.size);
      int zi = (int)((p->pos.Z-v.zbase) / v.size);
      // assert(xi < SIZE);
      // assert(yi < SIZE);
      // assert(zi < SIZE);
      v.voxels[vector3di(xi,yi,zi)].push_back(p);
      //cout << "Dumped part into : " << xi << "," << yi << "," << zi << endl;
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

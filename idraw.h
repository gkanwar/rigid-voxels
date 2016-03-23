#ifndef IDRAW_H
#define IDRAW_H

#include <irrlicht/irrlicht.h>

#include "types.h"

// Pull in irr::* namespaces
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;


namespace {
IrrlichtDevice *dev;
IVideoDriver *driver;
ISceneManager *smgr;
IGUIEnvironment *guienv;

struct IrrObj {
  Obj* parent;
  IMeshSceneNode* node;
  vector<IMeshSceneNode*> parts;

  void updatePos() {
    node->setPosition(vector3df(parent->x, parent->y, 0));
    node->setRotation(vector3df(0, 0, parent->theta*180.0/M_PI));
    
    assert(parent->parts.size() == parts.size());
    for (int i = 0; i < parts.size(); ++i) {
      IMeshSceneNode* mn = parts[i];
      Particle* part = parent->parts[i];
      mn->setPosition(vector3df(part->x, part->y, 0));
    }
  }
};

vector<IrrObj> objects;
} // anonymous namespace

namespace idraw {

int init() {
  dev = createDevice(EDT_OPENGL, dimension2d<u32>(640, 480), 16, false, false, false, 0);
  if (!dev) return 1;
  dev->setWindowCaption(L"RigidVoxels");

  driver = dev->getVideoDriver();
  smgr = dev->getSceneManager();
  guienv = dev->getGUIEnvironment();

  // Just some sample text
  guienv->addStaticText(L"RigidVoxels test", rect<s32>(10,10,260,22), true);

  // TODO: Lighting
  // smgr->addLightSceneNode(0/*parent*/, vector3df(0,0,10.0)/*pos*/);
  // smgr->addLightSceneNode(0/*parent*/, vector3df(3,3,100)/*pos*/);
  
  smgr->addCameraSceneNode(0/*parent*/, vector3df(0,0,10.0)/*position*/,
                           vector3df(0,0,0)/*lookat*/);

  return 0;
}

void cleanup() {
  dev->drop();
}

int addObj(Obj* obj) {
  IrrObj io;
  io.parent = obj;
  IAnimatedMesh *mesh = smgr->getMesh("queen.obj");
  io.node = smgr->addMeshSceneNode(mesh, 0 /*parent*/, -1/*id*/,
                                   vector3df(0,0,0)/*position*/,
                                   vector3df(0,0,0)/*rotation*/, 
                                   vector3df(0.1,0.1,0.1)/*scale*/);
  for (Particle *p : obj->parts) {
    IMeshSceneNode *mn = smgr->addSphereSceneNode(PART_D/2/*radius*/);
    io.parts.push_back(mn);
  }
  io.updatePos();
  objects.push_back(io);
}

int step() {
  for (IrrObj &io : objects) {
    io.updatePos();
  }

  if (dev->run()) {
    driver->beginScene(true, true, SColor(255, 0, 0, 255)); // blue bg for debugging
    smgr->drawAll();
    guienv->drawAll();
    driver->endScene();
    return 0;
  }

  return 1; // Window is closed
}

} // namespace idraw

#endif

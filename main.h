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

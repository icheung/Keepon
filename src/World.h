/*
 * World.h
 *
 *  Created on: Feb 19, 2009
 *      Author: njoubert
 * (edited by jima)
 */

#ifndef WORLD_H_
#define WORLD_H_

#include "global.h"
#include "UCB/SceneInfo.h"
#include "Primitives.h"
#include "Viewport.h"

/**
 * The World forms a container for lights and primitives
 * in our scene.
 */
class World {
public:
  World();
  ~World();
  
  void loadScene(string filename);
  
  // tests a ray for intersection with the world, and fills in t, n, and m with info from the surface it hit
  bool intersect(Ray & r, double & t, vec3 &n, MaterialInfo &m);
  
  inline void setAmbientLight(vec3 ambient) { _ambientLight = ambient; }
  inline const vec3& getAmbientLight() { return _ambientLight; }
  
  inline vector<Light>::iterator getLightsBeginIterator(int type) {
    return _lights[type].begin();
  }
  inline vector<Light>::iterator getLightsEndIterator(int type) {
    return _lights[type].end();
  }

  int sphereCount() { return _spheres.size(); }
  inline Sphere *getSphere(int i) { return &_spheres[i]; }
  
  inline Viewport& getViewport() { return _view; }

  void insertSphere(Sphere s) { _spheres.push_back(s); }

  void insertViewport(Viewport v) { _view = v; }

  void insertLight(Light l) { _lights[l.getLightInfo().type].push_back(l); }
  
private:
  
  vector<Sphere> _spheres; // for now, all the geometry is spheres
  vector<Light> _lights[3]; // keep a separate array for each sort of light except ambient
  vec3 _ambientLight; // ambient lights just sum to a world-wide value
  
  Viewport _view;
};

#endif /* WORLD_H_ */


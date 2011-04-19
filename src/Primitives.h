/*
 * Primitive.h
 *
 *  Created on: Feb 19, 2009
 *      Author: njoubert
 * (edited by jima)
 */

#ifndef PRIMITIVE_H_
#define PRIMITIVE_H_

#include "global.h"
#include "UCB/SceneInfo.h"

struct Light {
    Light() {}
Light(vec3 pos, vec3 dir, LightInfo l) : _l(l), _pos(pos), _dir(dir) {}
    
    inline const LightInfo& getLightInfo() { return _l; }
    inline const vec3& getPosition() { return _pos; }
    inline const vec3& getDirection() { return _dir; }
    
private:
    LightInfo _l;
    vec3 _pos, _dir;
};

class Sphere {
public:
    Sphere(vec4 center, double radius, MaterialInfo m);
    Sphere(vec4 center, double radius, MaterialInfo m, mat4 m2w);
    
    double intersect(Ray & ray);
    vec4 calculateNormal(vec4 & position);
    inline MaterialInfo& getMaterial() { return _m; }
    vec4 getPosition() { return _p; }
    mat4 getM2W() { return _modelToWorld; }
    mat4 getW2M() { return _worldToModel; }
    
private:
    vec4 _p;
    double _r;
    MaterialInfo _m;
    
    // these won't be needed until you want to stretch the spheres:
    mat4 _modelToWorld;
    mat4 _worldToModel;
};

#endif /* PRIMITIVE_H_ */

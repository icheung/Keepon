/*
 * World.cpp
 *
 *  Created on: Feb 19, 2009
 *      Author: njoubert
 * (edited by jima)
 */

#include "World.h"
#include <limits>

World::World() {
    loadScene("scene.scd");
}

World::~World() {}

bool World::intersect(Ray & r, double & bestT, vec3 &outn, MaterialInfo &outm) {
    bestT = numeric_limits<double>::infinity();  
    vector<Sphere>::iterator i;
    
    for (i = _spheres.begin(); i < _spheres.end(); i++) {
        double currentT = i->intersect(r);
        if (currentT < bestT) {
            bestT = currentT;
            vec4 p = r.getPos(bestT);
            
            // compute normal in object space
            vec4 tempNormal = i->calculateNormal(p);
            tempNormal = i->getW2M().transpose() * tempNormal;
            tempNormal[VW] = 0;
            outn = vec3(tempNormal, VW).normalize();
            outm = i->getMaterial();
        }
    }
    
    return bestT < numeric_limits<double>::infinity();
}

void World::loadScene(string filename) {
    // for as4, you can optionally hard-code the scene.  For as5 and as6 it must be loaded from a file.
    
    /*
    vec4 eye(0.0, 0.0, 0, 1.0);
    vec4 LL(-1.0, -1.0, -3.0, 1.0);
    vec4 UL(-1.0, 1.0, -3.0, 1.0);
    vec4 LR(1.0, -1.0, -3.0, 1.0);
    vec4 UR(1.0, 1.0, -3.0, 1.0);
    
    _view = Viewport(eye, LL, UL, LR, UR, IMAGE_WIDTH, IMAGE_HEIGHT);
    
    _lights[LIGHT_DIRECTIONAL].push_back(
        Light(0, vec3(.5,.5,-.5), 
              LightInfo(LIGHT_DIRECTIONAL, vec3(.4, .8, 1.2))));
    _lights[LIGHT_POINT].push_back(
        Light(vec3(0,0,-14), 0,
              LightInfo(LIGHT_POINT, vec3(1.39, .2, .2))));
    _ambientLight = vec3(.5,.2,.2);
    
    _spheres.push_back(Sphere(vec4(1.5,-1.5,-10.0,1.0), 1.0,
                              MaterialInfo(vec3(.5, .9, .4), 
                                           .5, .5, .3, 4, .5)));
    _spheres.push_back(Sphere(vec4(0.0,4.0,-25.0), 2.5,
                              MaterialInfo(vec3(.9, .4, .5),
                                           .4, .2, .5, 20, 0)));
    _spheres.push_back(Sphere(vec4(-2.5,-1.5,-17.0), 2.0,
                              MaterialInfo(vec3(.4, .5, .9),
                                           .1, .5, .5, 150, 1)));
    */
}




/*
 * Viewport.cpp
 *
 *  Created on: Feb 19, 2009
 *      Author: njoubert
 */

#include "Viewport.h"

Viewport::Viewport(vec4 eye, vec4 LL, vec4 UL, vec4 LR, vec4 UR, int width, int height) {
    _eye = eye;
    _LL = LL;
    _UL = UL;
    _LR = LR;
    _UR = UR;
    _pixelsWide = width;
    _pixelsHigh = height;
	_raysPerPixel = 1;
	_incPP = (int)sqrt((float)_raysPerPixel);
	_viewToWorld = identity3D();
}


void Viewport::resetSampler() {
    _i = _j = 1.0 / (_incPP + 1); //Starts off at the center of a pixel.
    _percentage = -1;
}

bool Viewport::getSample(vec2 &p, Ray &r) {
    _i += 1.0/_incPP;
    if (_i >= _pixelsWide) {
        _i = 0.5;
        _j += 1.0/_incPP;
    }
    if (_j >= _pixelsHigh) {
        resetSampler();
        return false;
    }

    p = vec2(_i, _j);

	// friendly progress counter
    int soFar = int( 100*((floor(_i)+1)*(floor(_j)+1) / (double) (_pixelsWide * _pixelsHigh)) );
    if (soFar > _percentage+24) {
    	_percentage = soFar;
    	cout << soFar << "% Done." << endl;
    }

	double u = _i / (double) _pixelsWide;
    double v = _j / (double) _pixelsHigh;
    vec4 onscreen(((1-u)*((1-v)*_LL[0] + v*_UL[0]) + u*((1-v)*_LR[0] + v*_UR[0])),
				  ((1-u)*((1-v)*_LL[1] + v*_UL[1]) + u*((1-v)*_LR[1] + v*_UR[1])),
				  ((1-u)*((1-v)*_LL[2] + v*_UL[2]) + u*((1-v)*_LR[2] + v*_UR[2])),1);
	r = Ray(_eye, onscreen, 0.1);

    return true;
}



/*
 * Primitive.cpp
 *
 *  Created on: Feb 19, 2009
 *      Author: njoubert
 */

#include "Primitives.h"

#include <limits>


Sphere::Sphere(vec4 center, double radius, MaterialInfo m) {
    _p = center;
    _r = radius;
    _m = m;
    _modelToWorld = mat4(vec4(1,0,0,0), vec4(0,1,0,0),
                         vec4(0,0,1,0), vec4(0,0,0,1));
    _worldToModel = mat4(vec4(1,0,0,0), vec4(0,1,0,0),
                         vec4(0,0,1,0), vec4(0,0,0,1));
}

Sphere::Sphere(vec4 center, double radius, MaterialInfo m, mat4 transform) {
    _p = center;
    _r = radius;
    _m = m;
    _modelToWorld = transform;
    mat4 temp = _modelToWorld;
    _worldToModel = temp.inverse();
}

//Checks for intersection with the given ray
double Sphere::intersect(Ray & ray) {
	/*
	// MY ALGORITHM ROCKS.
    vec4 e = _worldToModel * ray.start();
    vec4 d = _worldToModel * ray.direction();
    vec4 center = _worldToModel * _p;
    //double r = _r;
    
    double a = (d * d);
    double b = (2 * d * (e - center));
    double c = ((e - center) * (e - center)) - (_r * _r);
    
    double discriminant = ((b * b) - 4 * a * c);
    if (discriminant < 0) {
        return numeric_limits<double>::infinity(); //no hit!
    } else {
        double t1 = ((-1 * b) + sqrt(discriminant)) / (2 * a);
        double t2 = ((-1 * b) - sqrt(discriminant)) / (2 * a);
        double minimum = MIN(t1 ,t2);
        double maximum = MAX(t1, t2);

		// Return the smallest non-negative t.
        if (maximum > 0) {
            return ((minimum > 0) ? minimum : maximum);
        } else {
            return numeric_limits<double>::infinity();
        }
    }*/

	vec3 d(_worldToModel * ray.direction(), VW);
	vec3 e(_worldToModel * ray.start(), VW);
	vec4 center(_worldToModel * _p);
	vec3 ec = vec3(vec4(e,1)-center, VW);
	double dec = (d * ec);
	double desc = dec*dec - (d*d)*(ec*ec - _r*_r);
	if (desc < 0) {
	    return numeric_limits<double>::infinity(); //no hit!
	}
	desc = sqrt(desc);
	double t1 = (-1*dec - desc) / (d*d);
	double t2 = (-1*dec + desc) / (d*d);
	double minimum = MIN(t1 ,t2);
    double maximum = MAX(t1, t2);
    if (maximum > 0)
        return ((minimum > 0) ? minimum : maximum);
    else
        return numeric_limits<double>::infinity();
}

//Calculates the normal for the given position on this sphere.
vec4 Sphere::calculateNormal(vec4 & position) {
    vec4 convertedPosition = _worldToModel * position;
    vec4 convertedCenter = _worldToModel * _p;
    return ((convertedPosition - convertedCenter) / _r);
    //return ((position - _p) / _r);
}

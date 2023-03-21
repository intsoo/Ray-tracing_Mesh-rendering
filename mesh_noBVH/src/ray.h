/*
 * ===================================================
 *
 *       Filename:  ray.h
 *    Description:  Ray Tracing In One Weekend (RTIOW): Final Code 
 *        Created:  2022/07/13
 * 
 * ===================================================
 */


// Preprocessors
#pragma once

#include "vec3.h"


// Classes

class Ray 
{
public:
    Ray() {}
  	Ray(const Point3& origin, const Point3& direction, double time = 0.0)
		: orig(origin), dir(direction), tm(time)
	{}

	Point3 origin() const { return orig; }
	Vec3 direction() const { return dir; }
	double time() const { return tm; }

	Point3 at(double t) const {
		return orig + t * dir;
	}

public:
	Point3 orig;
	Vec3 dir;
	double tm;
};

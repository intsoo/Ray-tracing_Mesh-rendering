/*
 * ===================================================
 *
 *       Filename:  hittable.h
 *    Description:  Ray Tracing In One Weekend (RTIOW): Final Code 
 *        Created:  2022/07/13
 * 
 * ===================================================
 */


// Preprocessors
#pragma once

#include "ray.h"
#include "utility.h"


class Material;

// Classes
struct HitRecord 
{
	Point3 p;
	Vec3 normal;
	shared_ptr<Material> mat_ptr;
	double t;
	bool is_front_face;

	inline void set_face_normal(const Ray& r, const Vec3& outward_normal) {
		is_front_face = dot(r.direction(), outward_normal) < 0;
		normal = is_front_face ? outward_normal : -outward_normal;
	}
};


class Hittable 
{
	public:
		virtual bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const = 0;
};

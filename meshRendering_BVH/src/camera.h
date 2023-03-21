/*
 * ===================================================
 *      
 *       Filename:  camera.h
 *    Description:  Ray Tracing In One Weekend (RTIOW): Final Code 
 *        Created:  2022/07/13
 * 
 * ===================================================
 */



// Preprocessors
#pragma once

#include "utility.h"


// Classes

class Camera {
public:
	Camera(
		Point3 lookfrom,
		Point3 lookat,
		Vec3   vup,	
		double vfov, // vertical field-of-view in degrees
		double aspect_ratio,
		double aperture,
		double focus_dist,
		double _time0 = 0,
		double _time1 = 0

		) {
		double theta = degrees_to_radians(vfov);
		double h = tan(theta/2);
		double viewport_height = 2.0 * h;
		double viewport_width = aspect_ratio * viewport_height;		

		Vec3 w = unit_vector(lookfrom - lookat);
		Vec3 u = unit_vector(cross(vup, w));
		Vec3 v = cross(w, u);

		origin = lookfrom;
		horizontal = focus_dist * viewport_width * u;
		vertical = focus_dist * viewport_height * v;
		lower_left_corner = origin - horizontal/2 - vertical/2 - focus_dist*w;
		lens_radius = aperture / 2;
		time0 = _time0;
		time1 = _time1;

	}

	Ray get_ray(double s, double t) const {
		Vec3 rd = lens_radius * random_in_unit_disk();
		Vec3 offset = u * rd.x() + v * rd.y();
		
		return Ray(
			origin + offset,
			lower_left_corner + s*horizontal + t*vertical - origin - offset,
			random_double(time0, time1)
		);

	}

private:
	Point3 origin;
	Point3 lower_left_corner;
	Vec3 horizontal;
	Vec3 vertical;
	Vec3 u, v, w;
	double lens_radius;
	double time0, time1;  // shutter open/close times

};

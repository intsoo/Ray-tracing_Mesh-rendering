/*
 * ===================================================
 *
 *       Filename:  aabb.h
 *    Description:  Ray Tracing: The Next Week (RTTNW): ~BVH 
 *        Created:  2022/07/13
 * 
 * ===================================================
 */


// Preprocessors
# pragma once

#include "utility.h"

#include <iostream>
using namespace std;

// Classes
class Aabb {
public:
	// Constructors
	Aabb() {}
	Aabb(const Point3& a, const Point3& b) { minimum = a; maximum = b;}

	// Getters
	Point3 min() const { return minimum; }
	Point3 max() const { return maximum; }
	Point3 mid() const { return (minimum + maximum) * 0.5f; }

	bool hit(const Ray& r, double t_min) const;

private:
	Point3 minimum;
	Point3 maximum;
};


// Function Definitions
bool Aabb::hit(const Ray& r, double t_min) const 
{
	for (int a = 0; a < 3; a++) 
	{
		double t0 = fmin((minimum[a] - r.origin()[a]) / r.direction()[a], 
			(maximum[a] - r.origin()[a]) / r.direction()[a]);
		double t1 = fmax((minimum[a] - r.origin()[a]) / r.direction()[a], 
			(maximum[a] - r.origin()[a]) / r.direction()[a]);
		
		// intersection time
		/*
		t_min = fmax(t0, t_min);
		t_max = fmin(t1, t_max);
		*/		
		if(t_min < t0)  // If the current AABB is further than the closest hit point so far 
		{
			//cout << "* Current box is FURTHER than the recorded hit point *" << endl;
			return false;
		}
		if (t1 <= t0)  // If not hit
		{
			//cout << "* Current box is NOT hit *" << endl;
			return false;
		}
	}
	//cout << "* Current box is hit *" << endl;
	return true;  // hit
}

/*
inline bool aabb::hit(const ray& r, double t_min, double t_max) const {
    	for (int a = 0; a < 3; a++) {
		auto invD = 1.0f / r.direction()[a];
		auto t0 = (min()[a] - r.origin()[a]) * invD;
		auto t1 = (max()[a] - r.origin()[a]) * invD;
		if (invD < 0.0f)
	    		std::swap(t0, t1);
		t_min = t0 > t_min ? t0 : t_min;
		t_max = t1 < t_max ? t1 : t_max;
		if (t_max <= t_min)
	    		return false;
    	}
    	return true;
}
*/

inline Aabb computeSurroundingBox(Aabb box0, Aabb box1) {
	// Compute minimum and maximum points of AABB
	Point3 box_min(fmin(box0.min().x(), box1.min().x()),
			fmin(box0.min().y(), box1.min().y()),
			fmin(box0.min().z(), box1.min().z()));

	Point3 box_max(fmax(box0.max().x(), box1.max().x()),
		fmax(box0.max().y(), box1.max().y()),
		fmax(box0.max().z(), box1.max().z()));

	return Aabb(box_min, box_max);
}

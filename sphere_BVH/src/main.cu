/*
 * ===================================================
 *
 *       Filename:  main.cu
 *    Description:  Ray Tracing In One Weekend (RTIOW): ~BVH 
 *        Created:  2022/07/13
 * 
 * ===================================================
 */


// Preprocessors
#include "moving_sphere.h"
#include "material.h"
#include "utility.h"
#include "color.h"
#include "hittable_list.h"
#include "sphere.h"
#include "camera.h"
#include "bvh.h"

#include "mkPpm.h"
#include "mkCuda.h"
#include "mkClockMeasure.h"

#include <iostream>

#define MAX_SIZE 500

unsigned char *array;


// Functions
// 1. random_scene: Implements the 3D World.
hittable_list create_scene(int n) 
{
    hittable_list world;
	int count = 0;
	auto ground_material = make_shared<lambertian>(color(0.5f, 0.5f, 0.5f));
	//auto ground_material = make_shared<lambertian>(color(0.5f, 0.5f, 0.0f));

    world.add(make_shared<sphere>(++count, point3(0,-999,0), 999, ground_material));

    for (int a = -n; a < n; a++) {
		for (int b = -n; b < n; b++) {

			// Choose material type by random.
			float choose_mat = (a * 11 + b)/121;

			// Create new objects.
			//auto choose_mat = random_float();  // by random
	    	//point3 center(a + 0.9*random_float(), 0.2, b + 0.9*random_float());
	    	point3 center(a, 0.2f, b);

	    	if ((center - point3(4, 0.2f, 0)).length() > 0.9f) {
			    shared_ptr<material> sphere_material;  // material type

				// diffuse (lambertian)
				if (choose_mat < 0.8f) {
		    		color albedo = color::random() * color::random();
		    		sphere_material = make_shared<lambertian>(albedo);
					
					point3 center2 = center + vec3(0, random_float(0,0.5f), 0);
		    		world.add(make_shared<sphere>(++count, center, 0.2f, sphere_material));

				// metal
				} else if (choose_mat < 0.95f) {
		    		color albedo = color::random(0.5f, 1);
		    		float fuzz = random_float(0, 0.5f);
		    		
					sphere_material = make_shared<metal>(albedo, fuzz);
		    		world.add(make_shared<sphere>(++count, center, 0.2f, sphere_material));

				// dielectric
				} else {
					float refract_idx = 1.5f;
		    		sphere_material = make_shared<dielectric>(refract_idx);
		    		world.add(make_shared<sphere>(++count, center, 0.2f, sphere_material));
				}
	    	}
		}
    }

	auto material1 = make_shared<dielectric>(0.5);
	world.add(make_shared<sphere>(++count, point3(0, 1.3, 0), 1.3, material1));
	//world.add(make_shared<sphere>(++count, point3(3, 1, -0.5), 1.0, material1));

    auto material2 = make_shared<lambertian>(color(0.4, 0.2, 0.1));
    world.add(make_shared<sphere>(++count, point3(-4, 1.3, 0), 1.3, material2));

	auto material3 = make_shared<metal>(color(0.7, 0.6, 0.5), 0.0);
	world.add(make_shared<sphere>(++count, point3(4, 1.3, 0), 1.3, material3));
	//world.add(make_shared<sphere>(++count, point3(3, 1, 2), 1.0, material3));

/*
	auto material4 = make_shared<metal>(color(0.5, 0.7, 0.5), 0.1);
	//world.add(make_shared<sphere>(++count, point3(3, 1, -0.5), 1.0, material4));

	auto material5 = make_shared<lambertian>(color(1.0, 0.0, 0.6));
	//world.add(make_shared<sphere>(++count, point3(5, 0.5, 0), 0.5, material5));

	auto material6 = make_shared<dielectric>(0.5);
	//world.add(make_shared<sphere>(++count, point3(5, 0.3, 1.3), 0.3, material6));
*/
	return world;
	
	// Constructing BVH
/*
	hittable_list world_bvh;
	world_bvh.add(make_shared<bvh_node>(world, 0, 1));
	printf("\n\n================================== BVH CONSTURCTION COMPLETED ==================================\n\n\n");

	return world_bvh;
*/

}


// 2. ray_color: calculates color of the current ray intersection point.
color ray_color(const ray& r, const hittable& world, int depth) 
{    
	hit_record rec;

	// Limit the number of child rays.
	if (depth <= 0)	return color(0, 0, 0);  // No light approaches the current point.

	// If the ray hits an object: Hittable Object
	if (world.hit(r, 0.001, infinity, rec)) {
		ray scattered;
		color attenuation;

		if (rec.mat_ptr->scatter(r, rec, attenuation, scattered))  // Compute how the ray scatters on the intersection point.
			return attenuation * ray_color(scattered, world, depth-1);
		return color(0,0,0);
	}

	// If the ray hits no object: Background
	vec3 unit_direction = unit_vector(r.direction());
	auto t = 0.5 * (unit_direction.y() + 1.0);
	return (1.0 - t) * color(1.0, 1.0, 1.0) + t * color(0.5, 0.7, 1.0);
	//return (1.0 - t) * color(1.0, 1.0, 1.0) + t * color(0.7, 0.7, 1.0);
}


// 3. render: renders an output image.
void render(int image_height, int image_width, int samples_per_pixel, int depth, unsigned char* image, const camera& cam, const hittable& world) 
{
	float r, g, b;

	// RT18
	//PRINT PIXEL VALUES OF THE OUTPUT IMAGE: printf("------------------- IMAGE -------------------\n");

	for (int j = 0; j < image_height; ++j) {
	   	for (int i = 0; i < image_width; ++i) {
			int idx = (j * image_width + i) * 3;
			color pixel_color(0, 0, 0);

			for (int s = 0; s < samples_per_pixel; ++s) {
				auto u = (i + random_float()) / (image_width - 1);
				auto v = ((image_height-j-1) + random_float()) / (image_height - 1);

				ray cur_ray = cam.get_ray(u, v);

				// RT17: FOR DEBUGGING
				/*
				printf("(RENDER) Pixel (%lf, %lf): Ray Direction = (%lf, %lf, %lf)\n\n", 
				u, v, 
				(cur_ray.direction()).e[0], (cur_ray.direction()).e[1], (cur_ray.direction()).e[2]);
				*/

				pixel_color += ray_color(cur_ray, world, depth);

				r = pixel_color.x();
				g = pixel_color.y();
				b = pixel_color.z();

				// Antialiasing
				float scale = 1.0 / samples_per_pixel;
				r = sqrt(scale * r);
				g = sqrt(scale * g);
				b = sqrt(scale * b);		
				
				//printf("[%dx%d s:%d] %lf %lf %lf\n", j, i, s, pixel_color[0], pixel_color[1], pixel_color[2]);
				//printf("[%d] %f %f %f\n", s, pixel_color[0], pixel_color[1], pixel_color[2]);
			}
		
			image[idx] = (256 * clamp(r, 0.0f, 0.999f));
			image[idx+1] = (256 * clamp(g, 0.0f, 0.999f));
			image[idx+2] = (256 * clamp(b, 0.0f, 0.999f));

			// RT18 - PRINT PIXEL VALUES OF THE OUTPUT IMAGE:
			//printf("  R:%d, G:%d, B:%d\n", image[idx], arimageray[idx+1], image[idx+2]);
		}
    }
}



// 4. main
int main() {

	// Execution time
	mkClockMeasure *ckCpu = new mkClockMeasure("CPU CODE");
	ckCpu->clockReset();

   	// Image
	float aspect_ratio = 16.0 / 9.0;
   	int image_width = 100;  //400
	int image_height = (int)(image_width / aspect_ratio);
	int samples_per_pixel = 1;    
	const int max_depth = 50;

	// Output Image Array
	array = (unsigned char *)malloc(sizeof(unsigned char) * image_width * image_height * 3);

	// Camera
	point3 lookfrom(13,2,3);
	point3 lookat(0,0,0);
	vec3 vup(0,1,0);
	auto dist_to_focus = 20.0;
	auto aperture = 0.1;
	camera cam(lookfrom, lookat, vup, 20, aspect_ratio, aperture, dist_to_focus, 0.0, 1.0);

	// Objects
	int n = 0;
	int object_num = (n+n)*(n+n)+4;
	

	// 1. Create a 3D world or scene.
	ckCpu->clockResume();
    hittable_list world = create_scene(n);
	ckCpu->clockPause();
    ckCpu->clockPrint("Creat World");

	// 2. Render an image.
	ckCpu->clockReset();
	ckCpu->clockResume();

	render(image_height, image_width, samples_per_pixel, max_depth, array, cam, world);

	// RT18 - PRINT PIXEL VALUES OF THE OUTPUT IMAGE: 
	//printf("---------------------------------------------\n");

	ckCpu->clockPause();
	ckCpu->clockPrint("Rendering");

	// 3. Create a PPM image file. 
	ppmSave("img.ppm", array, image_width, image_height, object_num, samples_per_pixel);

	return 0;
}

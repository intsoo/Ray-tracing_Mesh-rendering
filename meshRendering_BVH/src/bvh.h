/*
 * ===================================================
 *
 *     Project Name:  RT_mesh_BVH
 *        File Name:  bvh.h
 *      Description:  
 *					
 *          Created:  2023/02/16
 * 
 * ===================================================
 */


// Preprocessors
#pragma once

#include <algorithm>

#define MAX_SIZE 1000000
#define IMG_WIDTH 20


// Structures
typedef struct BvhNode 
{
	int nodeIdx;
	int first;
	int last;
//	int leftIdx;   // 2n+1 (n: current index)
//	int rightIdx;  // 2n+2
	Aabb box;
/*
	// 1. hit: Test whether ray hits the node.
	bool hit(Ray &r, float t_min, float t_max, HitRecord &rec) {
		// DEBUGGING
		int x_idx = threadIdx.x + blockIdx.x * blockDim.x;
		int y_idx = threadIdx.y + blockIdx.y * blockDim.y;
		int pixelIdx = yIdx * IMG_WIDTH + xIdx;

		// No hit
		if (!box.hit(r, t_min, t_max)) {
			printf("BVH NODE *NOT* HIT\n");  // DEBUGGING
			return false;
		}
		// Hit
		// DEBUGGING
		Vec3 pm = box.min(), pM = box.max();
		printf("------------------------------ THD %d: NODE %d HIT! ------------------------------\n  -%-8s: (%d ~ %d)\n  -%-8s: (%.1lf,%.1lf,%.1lf) ~ (%.1lf,%.1lf,%.1lf)\n--------------------------------------------------------------------------\n", 
		pixelIdx, nodeIdx, "Objects", first, last, 
		"AABB", pm.e[0], pm.e[1], pm.e[2], pM.e[0], pM.e[1], pM.e[2]);
		
		return true;	
	}
*/
	// 2. boounding_box: Return the AABB of the node.
	bool getAabb(Aabb *output_box) 
	{
		*output_box = box;
		return true;
	}

	// 3. print_info: Print node info.
	bool printInfo() 
	{
		Vec3 p_min = box.min(), p_max = box.max();
		printf("Node %d [AABB] (%.1lf, %.1lf, %.1lf) ~ (%.1lf, %.1lf, %.1lf)\n", nodeIdx, p_min.e[0], p_min.e[1], p_min.e[2], p_max.e[0], p_max.e[1], p_max.e[2]);
	}

} BvhNode;


/*
// 4. Print node info.
__device__ bool print_info_bvhNode(BvhNode &node) {
	Vec3 pm = node.box.min(), pM = node.box.max();
	printf("Node %d [AABB] (%.1lf, %.1lf, %.1lf) ~ (%.1lf, %.1lf, %.1lf)\n", node.nodeIdx, pm.e[0], pm.e[1], pm.e[2], pM.e[0], pM.e[1], pM.e[2]);
}
*/
// 5. box_compare: Compare which box is closer to the origin in respect to the chosen axis.
inline bool box_compare(Face &f1, Face &f2, int axis) 
{
	Aabb box_a;
    Aabb box_b;
    	
	if (!f1.getAabb(box_a) || !f2.getAabb(box_b))  // If the boxes exists,
		printf("No bounding box in BVH node constructor!\n");
    	
		return box_a.min().e[axis] < box_b.min().e[axis];
}
/*
// 6. print_bvh: Print the result of BVH.
__device__ inline void print_bvh(
	BvhNode *bvh, 
	Sphere *obj_list,
	int obj_num)
{
	int node_num = 2*obj_num-1;
	for(int idx = 0; idx < node_num; idx++) {
		BvhNode cur_node = bvh[idx];
		Aabb cur_box = cur_node.box;	

	//	BvhNode left_node = bvh[2*idx+1];
	//	BvhNode right_node = bvh[2*idx+2];

		int obj_span = cur_node.last - cur_node.first + 1;

		printf("------------------------------ Node %d INFO (obj # = %d) ------------------------------\n  -%-8s: (%.1lf,%.1lf,%.1lf) ~ (%.1lf,%.1lf,%.1lf)\n----------------------------------------------------------------------------\n",
			cur_node.nodeIdx, obj_span, "AABB", 
			cur_box.min().e[0], cur_box.min().e[1], cur_box.min().e[2],
			cur_box.max().e[0], cur_box.max().e[1], cur_box.max().e[2]);
	}
*/
	
/*
	printf("\n\n========================================== CHECK BVH ===========================================\n\n\n");
	for(int i=0; i<count; i++) {
		bvh_node *cur_node = d_bvh[i];
		object_span = cur_node->last - cur_node->first + 1; 
		int nIdx = cur_node->nodeIdx;
		point3 pm = cur_node->box.min(), pM = cur_node->box.max();

		printf("[%d] Node %d -> \n", i, nIdx);
		printf("------------------------------ NODE %d (obj# = %d) -------------------------------\n", nIdx, object_span);
		printf("  -%-8s: (%d ~ %d)\n", "Objects", cur_node->first, cur_node->last);
		printf("  -%-8s: (%.1lf,%.1lf,%.1lf) ~ (%.1lf,%.1lf,%.1lf)\n", "AABB", pm.e[0], pm.e[1], pm.e[2], pM.e[0], pM.e[1], pM.e[2]);

		if(object_span <= 2) {
			sphere *leftN = (sphere *)cur_node->left, *rightN = (sphere *)cur_node->right;

			printf("  -%-8s: (%.1lf,%.1lf,%.1lf)\n  -%-8s: %.1lf\n  -%-8s: (%.1lf,%.1lf,%.1lf)\n  -%-8s: %.1lf\n----------------------------------------------------------------------------------\n",
			"Left: Center", leftN->center.e[0], leftN->center.e[1], leftN->center.e[2],
			"Left: Radius", leftN->radius,
			"Right: Center", rightN->center.e[0], rightN->center.e[1], rightN->center.e[2],
			"Right: Radius", rightN->radius);
		}
	}
*/
/*
}
*/

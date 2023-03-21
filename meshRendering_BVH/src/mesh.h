/**********************************************************************************
 * <Things to Complete>
 *
 * ~ 2023/02/02
 * 1. Load mesh (object) file: Start with a simple pyramid mesh!
 * 2. Implement triangle-ray intersection test: Debug first
 * 3. Implement mesh-ray intersection test: Debug first
 //////////////////////////////////////////////////////////////////////////////////
 * ~ 2023/02/??
 * 4. Render multiple meshes (Create mesh array or world)
 * 5. Create BVH
 **********************************************************************************/


// 1. 메시 오브젝트 파일 로딩

// 1.1. setter (setVertices, setFaces) 멤버함수 구현 (O)
// => AABB, Material 초기화 부분 제외하고 구현 완료

// 1.2. getter (getVertices, getFaces) 멤버함수 구현 (O)

// 1.3. getter함수 이용해서 mesh의 vertices, faces가 제대로 초기화되었는지 디버깅
// => printMesh() 함수를 구현하여 제대로 로딩이 됨을 확인함

// **** glm 함수 (normalize, cross) 직접 구현하기
// => utility.h에서 이미 구현한, Vec3 연산을 수행하는 inline 함수 unit_vector와 cross를 사용함



// ~2023/02/16 TO-DO
// 1. computeAabb(): 삼각형들을 모두 감싸는 AABB 만드는 함수
// 2. Face::getAabb(): 삼각형의 AABB를 반환하는 함수


// Preprocessors
#pragma once

#include <string>
#include <vector>
#include <array>

#include <fstream>
#include <iostream>
#include <cassert>

#include <float.h>

#include "vec3.h"
#include "ray.h"
#include "hittable.h"

// #BVH
#include "aabb.h"


#define POLYGON 3
#define DIMENSION 3

using namespace std;

struct HitRecord;

// Structures and Classes
struct Vertex
{
    // Stores actual element values
    Vec3 position;
    Vec3 normal;
    
    // Constructors
	Vertex(Vec3& position, Vec3& normal) : position(position), normal(normal) {}
};

struct Face
{
    array<int, POLYGON> vertices;
	array<int, POLYGON> normals;
    Aabb box;

    // Constructors
    Face(const array<int, POLYGON>& vertices, const array<int, POLYGON>& normals);
    
    // Ray-Triangle Intersection Test
    bool hit(const Ray& ray, double *t, Vec3& bary, const Vec3& v0, const Vec3& v1, const Vec3& v2);

};


// Structures
struct BvhNode 
{
	int idx;
	int f_first;  // index of first face(triangle)
	int f_last;  // index of last face(triangle)
    int left;  // left child index
    int right;  // right child index
	Aabb box;


    // Constructor
    BvhNode(int node_idx, int start, int end, int idx_l, int idx_r, Aabb& node_box);

    // Test whether ray hits the node
    bool hit(const Ray &ray, double t_min);

	// Return the AABB of the node
	bool getAabb(double time0, double time1, Aabb *output_box) const;

	// Print node info
	void printInfo() const;

};



class Mesh
{
public:
    // Constructors
	Mesh();
	explicit Mesh(vector<Vertex> vertices, vector<Face> faces);

    // Setters
	void setVertices(vector<Vertex> vertices);
	void setFaces(vector<Face> faces);
	void setMaterial(shared_ptr<Material> material);
    void setBvh(vector<BvhNode> bvh);  // #BVH

    // Computing sizes
	int getNumVertices() const;
	int getNumFaces() const;

    // Getters
	const vector<Vertex>& getVertices() const;
	const vector<Face>& getFaces() const;
	const shared_ptr<Material>& getMaterial() const;
//	const AABB& getBoundingBox() const override;
	Vec3 getVertexPos(int face, int v) const;
    Vec3 getVertexNorm(int face, int v) const;
    Vec3 getPointNorm(int face, double u, double v) const;

    // #BVH
    Aabb getAabb(int face) const;
//    bool compareAabbs(int f1, int f2, int axis) const; 

    bool hit(const Ray& ray, double min_t, HitRecord& rec);

    // #BVH
    // BVH construction
    void setAabb(int face, Aabb& input_box);
    void setAllAabbs();
    Aabb computeAabb(int start, int end);

    void printAabbs() const;
    void constructBvh();
    void printBvh() const;

    // BVH Traversal
    bool searchBvh(const Ray& ray, HitRecord &rec);


private:

//	void updateBoundingBox();
    vector<Vertex> m_vertices;  // stores actual values of its position and normal vector elements
    vector<Face> m_faces;  // stores indices of its vertices and normals
//	AABB m_boundingBox;
    shared_ptr<Material> m_material;
    
    // #BVH
    vector<BvhNode> m_bvh;  // pointer of root node

};


// Function Prototypes
void printMesh(string filename, Mesh& mesh);
bool loadObjFile(string filename, Mesh& mesh);
bool loadObjFile(string filename, vector<Vertex>& vertices, vector<Face>& faces);
bool testRayTriangleHit(const Ray& ray, double *t, Vec3& bary, const Vec3& v0, const Vec3& v1, const Vec3& v2);
Vec3 getBarycentric(Vec3 &p, const Vec3& v0, const Vec3& v1, const Vec3& v2); 
inline bool compareAabbs(Aabb &b1, Aabb &b2, int axis);




// Function Definitions

Face::Face(const array<int, POLYGON>& vertices, const array<int, POLYGON>& normals) :
    vertices(vertices),
    normals(normals) 
{

}

bool Face::hit(const Ray& ray, double *t, Vec3& bary, const Vec3& v0, const Vec3& v1, const Vec3& v2)
{
    double u, v, t_temp = 0.0f;
/*
    // #DEBUGGING /////////////////////////////////////////////////////////////////
    cout << "----------------------------------------------------------------------------\n";
    cout << "  - Ray: O=(" << ray.origin() << "), D=" << ray.direction() << ")" << endl;
    cout << "  - V0: P=(" << v0 << ")" << endl;
    cout << "  - V1: P=(" << v1 << ")" << endl;
    cout << "  - V2: P=(" << v2 << ")" << endl;
    cout << endl;
    ///////////////////////////////////////////////////////////////////////////////
*/

    // edges
//    Vec3 e1 = v1.position - v0.position;  // v0v1
//    Vec3 e2 = v2.position - v0.position;  // v0v2
    Vec3 e1 = v1 - v0;  // v0v1
    Vec3 e2 = v2 - v0;  // v0v2

    // 1. Check whether the ray is parallel to the plane
    // Calculate determinant
    Vec3 pvec = cross(ray.dir, e2);
    double det = dot(e1, pvec);

/*
    // #DEBUGGING /////////////////////////////////////////////////////////////////
    cout << "  - e1=(" << e1 << "), e2=" << e2 << ")" << endl;
    cout << "  - pvec=(" << pvec << endl;
    cout << "  - det = " << det << endl;
    cout << endl;
    ///////////////////////////////////////////////////////////////////////////////
*/  

    if (det <= 0.000001) return false;  // If the determinant is near zero, ray // plane

    // 2. Do the intersection test using Barycentric coordinates (u, v, w)
    // 2.1. Calculate U paramter and test bounds
    double inv_det = 1.0f / det;  // inverse determinant
    Vec3 tvec = ray.orig - v0;  // distance from vertex to ray origin
    u = dot(tvec, pvec) * inv_det;  // U paramter
    
/*
    // #DEBUGGING /////////////////////////////////////////////////////////////////
    cout << "  - tvec=(" << tvec << ")" << endl;
    cout << "  - u = " << u << endl;
    cout << endl;
    ///////////////////////////////////////////////////////////////////////////////
*/

    if (u < 0.0f || u > 1.0f ) return false; 

    // 2.2. Calculate V paramter and test bounds
    Vec3 qvec = cross(tvec, e1);
    v = dot(ray.dir, qvec) * inv_det;  // V paramter

/*
    // #DEBUGGING /////////////////////////////////////////////////////////////////
    cout << "  - qvec=(" << qvec << ")" << endl;
    cout << "  - v = " << v << endl;
    cout << endl;
    ///////////////////////////////////////////////////////////////////////////////
*/ 

    if (v < 0.0f || u + v > 1.0f) return false;

    // Ray intersects triangle
    // 3. Record intersection time
    t_temp = dot(e2, qvec) * inv_det;

/*
    // #DEBUGGING /////////////////////////////////////////////////////////////////
    cout << "  - tvec=(" << tvec << ")" << endl;
    cout << "  - u = " << u << endl;
    cout << endl;
    ///////////////////////////////////////////////////////////////////////////////
*/

    //cout << "** Triangle Hit at: t = " << t_temp << " **" << endl;

    *t = t_temp;

    return true;
}



BvhNode::BvhNode(int node_idx, int start, int end, int idx_l, int idx_r, Aabb& node_box) :
    idx(node_idx),
    f_first(start),
    f_last(end),
    left(idx_l),
    right(idx_r),
    box(node_box)
{

}

bool BvhNode::getAabb(double time0, double time1, Aabb *output_box) const
{
    *output_box = box;
    return true;
}


// 1. hit: Test whether ray hits the node.
bool BvhNode::hit(const Ray& ray, double t_min)
{

    // Do ray-AABB intersection test
    if (!box.hit(ray, t_min))  // not hit
    {
        //cout << "* Current node is NOT hit *" << endl;
        return false;
    }

    // hit
    // #DEBUGGING ///////////////////////////////////////////
    
    Point3 p_min = box.min(), p_max = box.max();
    /*
    cout << "------------------------------ Node " << idx << " (obj # = " << (f_last-f_first+1) << ") Hit ------------------------------\n"
        << "  - Faces: " << f_first << "~" << f_last << endl
        << "  - AABB: " << box.min() << " ~ " << box.max() << endl
        << "  - Left: N" << left << ", Right: N" << right << endl;
    */
    /////////////////////////////////////////////////////////

    return true;	
}

void BvhNode::printInfo() const
{
    Vec3 pm = box.min(), pM = box.max();
    printf("Node %d [AABB] (%.1lf, %.1lf, %.1lf) ~ (%.1lf, %.1lf, %.1lf)\n", idx, pm.e[0], pm.e[1], pm.e[2], pM.e[0], pM.e[1], pM.e[2]);
}




//////////수정해라///////////
Mesh::Mesh() :
//	m_boundingBox({0.0, 0.0, 0.0}, { 0.0, 0.0, 0.0 }),
	m_material(nullptr)
{

}

Mesh::Mesh(vector<Vertex> vertices, vector<Face> faces) :
	m_vertices(std::move(vertices)),
	m_faces(std::move(faces)),
//	m_boundingBox({ 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }),
	m_material(nullptr)
{
//  updateBoundingBox();

}


//////////////////////
// Setting values of private members
void Mesh::setVertices(vector<Vertex> vertices)
{
	m_vertices = std::move(vertices);  // Data transfer

//	updateBoundingBox();
}

void Mesh::setFaces(vector<Face> faces)
{
	m_faces = std::move(faces);  // Data transfer
}

void Mesh::setMaterial(shared_ptr<Material> material)
{
	m_material = std::move(material);
}


// #BVH
void Mesh::setBvh(vector<BvhNode> bvh)
{
    m_bvh = std::move(bvh);
}


// Sizes 
int Mesh::getNumVertices() const
{
	return static_cast<int>(m_vertices.size());
}

int Mesh::getNumFaces() const
{
	return static_cast<int>(m_faces.size());
}


// Getting values of private members
const vector<Vertex>& Mesh::getVertices() const
{
	return m_vertices;
}

const vector<Face>& Mesh::getFaces() const
{
	return m_faces;
}


// Get the position of the target vertex
Vec3 Mesh::getVertexPos(int face, int v) const
{
    // Prevent index out of range
	assert(face >= 0 && face < getNumFaces());
	assert(v >= 0 && v < 3);

    // Find the index of the target vertex vector
	const int v_idx = m_faces[face].vertices[v];

//  cout << "v_idx = " << v_idx << endl;

	assert(v_idx >= 0 && v_idx < getNumVertices());

	return m_vertices[v_idx].position;  // x y z values
}

// Get normal vector of the target vertex
Vec3 Mesh::getVertexNorm(int face, int v) const
{
	assert(face >= 0 && face < getNumFaces());
	assert(v >= 0 && v < 3);

	const int v_idx = m_faces[face].normals[v];

    // Check whether the target vertex exits
	if (v_idx >= 0 && v_idx < getNumVertices())
	{
		return m_vertices[v_idx].normal;  // Return its normal
	}
	else
	{
		// Compute its normal (equal to its face normal) manually
		const Vec3 v0 = getVertexPos(face, 0);
		const Vec3 v1 = getVertexPos(face, 1);
		const Vec3 v2 = getVertexPos(face, 2);

//		return glm::normalize(glm::cross(v1 - v0, v2 - v0));  // normal of the given face (v0v1 X v0v2)
		return unit_vector(cross(v1 - v0, v2 - v0));  // normal of the given face (v0v1 X v0v2)
	}
}

// Compute normal vector of an arbitrary point inside the face using interpolation
Vec3 Mesh::getPointNorm(int face, double u, double v) const
{
    // Compute vertex normals of the target face
	const Vec3 n_v0 = getVertexNorm(face, 0);
	const Vec3 n_v1 = getVertexNorm(face, 1);
	const Vec3 n_v2 = getVertexNorm(face, 2);

	const Vec3 normal_vector = (1.0 - u - v) * n_v0 + u * n_v1 + v * n_v2;  // interpolation of the three vertex normals

	return unit_vector(normal_vector);
}

// #BVH
Aabb Mesh::getAabb(int face) const
{
    const Aabb f_box = m_faces[face].box;

    return f_box;
}


void printMesh(string filename, Mesh& mesh) {

    int num_v = mesh.getNumVertices();
    int num_f = mesh.getNumFaces();

    vector<Vertex> m_vertices = mesh.getVertices();
    vector<Face> m_faces = mesh.getFaces();

    cout << "======================================" << "Printing Mesh Information " << filename << "======================================" << endl;

    // Vertices
    cout << "<Vertex Positions>" << endl;
    for(int v = 0; v < num_v; v++) {
        Vertex cur_v = m_vertices[v];
        cout << v+1 << ": v " << cur_v.position << endl;
    }
    cout << endl;

    cout << "<Vertex Normals>" << endl;
    // Normals
    for(int v = 0; v < num_v; v++) {
        Vertex cur_v = m_vertices[v];
        cout << v+1 << ": v " << cur_v.normal << endl;
    }
    cout << endl;

    cout << "<Faces>" << endl;
    for(int f = 0; f < num_f; f++)
    {
        Face cur_f = m_faces[f];
    //    cout << "Face " << f << ": V " << cur_f.vertices[0]+1 << " " << cur_f.vertices[1]+1 << " " << cur_f.vertices[2]+1 << endl;
        cout << "Face " << f << endl;
        for(int v = 0; v < cur_f.vertices.size(); v++) {
        
            //Vec3 cur_v = m_vertices[cur_f.vertices[v]].position;
            //Vec3 cur_n = m_vertices[cur_f.vertices[v]].normal;
            Vec3 cur_v = mesh.getVertexPos(f, v);
            Vec3 cur_n = mesh.getVertexNorm(f, v);
            cout << "Vertex " << v << ": P=(" << cur_v << "), N=(" << cur_n << ")" << endl;
        }
        cout << endl;
    }    
    cout << "=================================================================================================" << endl << endl;
}


// loadObjFile: Load a mesh (.obj) file
bool loadObjFile(string filename, Mesh& mesh) 
{
    vector<Vertex> m_vertices;
    vector<Face> m_faces;

    bool success = loadObjFile(filename, m_vertices, m_faces);
    
    // Initialize mesh
    mesh.setVertices(m_vertices);
    mesh.setFaces(m_faces);
    // #BVH
//    mesh.setAllAabbs();  // Initialize AABB of each face(triangle)

    return success;
}

bool loadObjFile(string filename, vector<Vertex>& vertices, vector<Face>& faces) 
{
    // Open the object file
    // ifstream file(filename);
    ifstream file;
    file.open(filename);

    if(file.fail())  // If it fails to read the file (== !file.is_open())
    {  
        // printf("Cannot open the object file.\n");
        cout << "======================================" << "Cannot open the object file " << filename << "======================================" << endl;
        return false;
    }

    cout << "======================================" << "Opening file " << filename << "======================================" << endl;

    // raw data
    vector<Vec3> raw_vertices;          
    vector<Vec3> raw_normals;
    vector<int> v_elements;
    vector<int> n_elements;

    // Read the file line by line and store its data
    string line;
    string line_type;
    string line_rest;

    // parsing
    while(getline(file, line)) 
    {
        // Determine the line type
        int cur_pos = 0;
        int pos = line.find(" ", cur_pos);
        int len;

        line_type = line.substr(0, pos);  // substring before space
        line_rest = line.substr(pos+1, line.length());  // substring after space

        //cout << line_type << " ";
        
        // 1) Vertex line
        if(line_type == "v")  // v x y z
        { 
            double e[DIMENSION];  // x y z
            
            for(int i=0; i<DIMENSION; i++) 
            {
                pos = line_rest.find(" ", cur_pos);  // index of the first " ", starting from cur_pos
                len = pos - cur_pos;  // length of the current element
                e[i] = stof(line_rest.substr(cur_pos, len));
                cur_pos = pos + 1;  // Move on to the next element

                //cout << e[i] << " ";  // debugging
            }
            raw_vertices.emplace_back(e[0], e[1], e[2]);  // add the current vertex data to the collection
        }
        else if(line_type == "vn")  // vn x y z
        {
            double e[DIMENSION];  // x y z
            
            for(int i=0; i<DIMENSION; i++) 
            {
                pos = line_rest.find(" ", cur_pos);  // index of the first " ", starting from cur_pos
                len = pos - cur_pos;  // length of the current element
                e[i] = stof(line_rest.substr(cur_pos, len));
                cur_pos = pos + 1;  // Move on to the next element

                //cout << e[i] << " ";  // debugging
            }
            raw_normals.emplace_back(e[0], e[1], e[2]);  // add the current vertex data to the collection
        }
        else if(line_type == "f")  // f v1 v2 v3 vn1 vn2 vn3
        {
            bool has_only_vertices = false;
        
            int v_e[POLYGON];
            int n_e[POLYGON];
            
            // Vertex indices
            for(int i=0; i<POLYGON && pos >= 0; i++) 
            {
                // v
                pos = line_rest.find("/", cur_pos);  // index of the first " ", starting from cur_pos
                if(pos == string::npos) 
                {
                    has_only_vertices = true;
                    pos = line_rest.find(" ", cur_pos);  // If there is no delimeter "/" found
                }

                len = pos - cur_pos;  // length of the current element
                v_e[i] = stoi(line_rest.substr(cur_pos, len));
                cur_pos = pos + 1;  // Move on to the next element

                if(has_only_vertices) 
                {
                    //cout << v_e[i] << " ";  // debugging
                    continue;
                }

                // vt
                pos = line_rest.find("/", cur_pos);
                cur_pos = pos + 1;

                // vn
                pos = line_rest.find(" ", cur_pos);  // index of the first " ", starting from cur_pos
                len = pos - cur_pos;  // length of the current element
                n_e[i] = stoi(line_rest.substr(cur_pos, len));
                cur_pos = pos + 1;

                //cout << v_e[i] << "//" << n_e[i] << " ";  // debugging
               
            }

            v_elements.push_back(v_e[0] - 1);
            v_elements.push_back(v_e[1] - 1);
            v_elements.push_back(v_e[2] - 1);

            n_elements.push_back(n_e[0] - 1);
            n_elements.push_back(n_e[1] - 1);
            n_elements.push_back(n_e[2] - 1);

        }

/*
            while((pos = line_rest.find(" ", cur_pos)) != string::npos) {  // while " " is found in line
                int len = pos - cur_pos;  // length of the current element
                double e = stof(line_rest.substr(cur_pos, len));  // Store the current element
                
                cout << e << endl;  // debugging
                cur_pos = pos + 1;  // Move on to the next element
            }
*/
    //cout << endl;
    }

    // Close the file
    file.close();

    /////////////////////////////////////////////////////////////////////////////
    // Refine the raw data
    
    // the numbers of mesh vertices and mesh normals
    int v_num = raw_vertices.size();
    int n_num = raw_normals.size();
    int max_num = std::max(v_num, n_num);

    // the total numbers of vertices and normals that make up mesh faces
    int max_f_num = std::max(v_elements.size(), n_elements.size());

    int f_v_num = v_elements.size();
    int f_n_num = n_elements.size();


    // 1) Vertex Data
    // Initialize vertex vector
    vertices.clear();
    vertices.reserve(raw_vertices.size());  // Resize the vector

    // Generate the vector
    for(int i = 0; i < max_num; i++) 
    {
        Vec3 cur_v;
        Vec3 cur_n;

        // Check index range before data transfer
		if (i < v_num)  cur_v = raw_vertices[i];
		if (i < n_num)  cur_n = raw_normals[i];
	        
        vertices.emplace_back(cur_v, cur_n);  // Add new vertex to the vector
    }


    // 2) Face Data
    // Initialize face vector
    faces.clear();
	faces.reserve(max_num/POLYGON);  // Resize the vector (3 vertices compose one face(triangle))

    // Generate the vector
    for (int i = 0; i < max_f_num; i += POLYGON)  // max_num != max_f_num
	{
//        int cur_v_idx[POLYGON] = {0, 0, 0};
//        int cur_n_idx[POLYGON] = {-1, -1, -1};

        array<int, POLYGON> cur_v_idx = { {0, 0, 0} };
        array<int, POLYGON> cur_n_idx = { {0, 0, 0} };

        // Check index range before data transfer
		if (i+2 < f_v_num)  // three vertices of the triangle
		{
			cur_v_idx[0] = v_elements[i];
			cur_v_idx[1] = v_elements[i+1];
			cur_v_idx[2] = v_elements[i+2];
		}

		if (i+2 < f_n_num)  // three normals at the vertices
		{
			cur_n_idx[0] = n_elements[i];
			cur_n_idx[1] = n_elements[i+1];
			cur_n_idx[2] = n_elements[i+2];
		}

		faces.emplace_back(cur_v_idx, cur_n_idx);  // Add new face to the vector
	}

    cout << "=================================================================================================" << endl << endl;
    
    return true;  // successful file loading
}


bool Mesh::hit(const Ray& ray, double min_t, HitRecord& rec)
{
    //cout << "\n\n================================== MESH HIT TEST ==================================\n" << endl;

/*
	// First intersect ray with AABB to quickly discard non-intersecting rays
	if (!m_boundingBox.hit(this.boundingBox(), ray))
	{
		return false;
	}
*/
	bool is_hit = false;

	rec.t = std::numeric_limits<double>::max();

	// Iterate over all triangles in the mesh
    int f_num = getNumFaces();
	for (int f = 0; f < f_num; f++)
	{
		const Vec3& v0 = getVertexPos(f, 0);
		const Vec3& v1 = getVertexPos(f, 1);
		const Vec3& v2 = getVertexPos(f, 2);

		// distance between origin and hit point
		double t_temp;
		// Output barycentric coordinates of the intersection point
		Vec3 bary;

//        cout << "Face[" << f << "]" << endl;

		// Intersection test
//		if (testRayTriangleHit(ray, &t_temp, bary, v0, v1, v2))  // Check if ray intersects the triangle
		if (m_faces[f].hit(ray, &t_temp, bary, v0, v1, v2))  // Check if ray intersects the triangle

        {
//            cout << "  t: " << t_temp << ", p: (" << ray.orig+t_temp*ray.dir << ")" << endl;  // debugging

            if(t_temp >= min_t && t_temp < rec.t)  // Update hit record only if hit earlier than the recorded one
            {
                rec.t = t_temp;   // intersection time
                rec.p = ray.orig + rec.t * ray.dir;  // intersection point

                bary = getBarycentric(rec.p, v0, v1, v2);

                rec.normal = getPointNorm(f, bary.x(), bary.y());  // normal at the intersection point
                rec.is_front_face = dot(ray.direction(), rec.normal) < 0;
                rec.mat_ptr = m_material;
            
                //cout << "[Update Hit Record] t: " << rec.t << ", p: (" << rec.p << ")" << endl;  // debugging

                is_hit = true;
            }
		}
//        cout << endl;
	}
    //cout << "\n\n===============================================================================\n" << endl;
    return is_hit;
}

//bool rayTriangleHit(const Ray& ray, HitRecord &rec, const Vertex& v0, const Vertex& v1, const Vertex& v2) const {
bool testRayTriangleHit(const Ray& ray, double *t, Vec3& bary, const Vec3& v0, const Vec3& v1, const Vec3& v2)
{
    double u, v, t_temp = 0.0f;

    // edges
//    Vec3 e1 = v1.position - v0.position;  // v0v1
//    Vec3 e2 = v2.position - v0.position;  // v0v2
    Vec3 e1 = v1- v0;  // v0v1
    Vec3 e2 = v2- v0;  // v0v2

    // 1. Check whether the ray is parallel to the plane
    // Calculate determinant
    Vec3 pvec = cross(ray.dir, e2);
    double det = dot(e1, pvec);

    if (det <= 0.0001) return false;  // If the determinant is near zero, ray // plane

    // 2. Do the intersection test using Barycentric coordinates (u, v, w)
    // 2.1. Calculate U paramter and test bounds
    double inv_det = 1.0f / det;  // inverse determinant
    Vec3 tvec = ray.orig - v0;  // distance from vertex to ray origin
    u = dot(tvec, pvec) * inv_det;  // U paramter
    
    if (u < 0.0f || u > 1.0f ) return false; 

    // 2.2. Calculate V paramter and test bounds
    Vec3 qvec = cross(tvec, e1);
    v = dot(ray.dir, qvec) * inv_det;  // V paramter
    
    if (v < 0.0f || u + v > 1.0f) return false;

    // Ray intersects triangle
    // 3. Record intersection time
    t_temp = dot(e2, qvec) * inv_det;
    *t = t_temp;
//    bary = Vec3{u, v, 1-u-v};
/*
    // 3. Update hit record
    if (t_temp > 0 && t_temp < rec.t)  // If the current hit time is earlier than the recorded one
    {  

        rec.t = t_temp;   // intersection time
        rec.p = ray.orig + rec.t * ray.dir;  // intersection point
        rec.normal = mesh.normal(f, u, v);  // normal at the intersection point
        rec.is_front_face = dot(ray.direction(), rec.normal) < 0;
        rec.mat_ptr = mesh.m_material;


//      이해가 안 되는 방법...
//        Vec3 bary = getBarycentric(rec.p);
//        rec.normal = mesh.normal(f, bary.u, bary.v);
  
        return true;
    }

    return false;
*/
//  cout << "  *Face " << "hit*" << endl;

    return true;
}

Vec3 getBarycentric(Vec3 &p, const Vec3& v0, const Vec3& v1, const Vec3& v2) 
{
    // edges
    Vec3 e1 = v1 - v0;  // v0v1
    Vec3 e2 = v2 - v0;  // v0v2
    
    Vec3 v2_ = p - v0;
    
    // lengths
    double d00 = dot(e1, e1);
    double d01 = dot(e1, e2);
    double d11 = dot(e2, e2);
    double d20 = dot(v2_, e1);
    double d21 = dot(v2_, e2);

    double d = d00 * d11 - d01 * d01;

    double v = (d11 * d20 - d01 * d21) / d;
    double w = (d00 * d21 - d01 * d20) / d;
    double u = 1 - v - w;
    
    return Vec3(u, v, w);
}
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////



// #BVH #CONSTRUCTION
///////////////////////////////////////////////////////////////////////////////////////////////////

// Set the AABB value of target triangle
void Mesh::setAabb(int face, Aabb& input_box)
{
    // Prevent index out of range
	assert(face >= 0 && face < getNumFaces());

    m_faces[face].box = input_box;
}


// Initialize AABB of each triangle in mesh
void Mesh::setAllAabbs()
{    
    // Check if the face array is empty
    //assert(m_faces.empty());
    
    // for all faces in mesh
    for(int f = 0; f < getNumFaces(); f++)
    {
        Aabb box;
        Vec3 cur_v;
        Vec3 box_min, box_max;  // minimum, maximum points of AABB (of current AABB)
        double x_min = FLT_MAX, y_min = FLT_MAX, z_min = FLT_MAX;
        double x_max = -FLT_MAX, y_max = -FLT_MAX, z_max = -FLT_MAX;

        // for all vertices in the face
        for (int v = 0; v < 3; v++)
        {
            // Get the current vertex position
            cur_v = getVertexPos(f, v);

            // Compute the minimum, maximum points of the current face
            x_min = (x_min > cur_v.x()) ? cur_v.x() : x_min;
            y_min = (y_min > cur_v.y()) ? cur_v.y() : y_min;
            z_min = (z_min > cur_v.z()) ? cur_v.z() : z_min;

            x_max = (x_max < cur_v.x()) ? cur_v.x() : x_max;
            y_max = (y_max < cur_v.y()) ? cur_v.y() : y_max;
            z_max = (z_max < cur_v.z()) ? cur_v.z() : z_max;
        }
        box_min = Vec3{x_min, y_min, z_min};
        box_max = Vec3{x_max, y_max, z_max};
        box = Aabb{box_min, box_max};

        // #DEBUGGING
        //cout << "face " << f << ": Min = (" << box_min << ", Max = (" << box_max << ")" << endl;

        setAabb(f, box);
    }
}


// Compute AABB that bounds triangles in a given range
Aabb Mesh::computeAabb(int start, int end)
{
    // Check if the face array is empty
    //assert(m_faces.empty());

    // Compute AABB bounding the triangles
    Aabb box;
    Vec3 cur_v;  // current vertex
    Vec3 box_min, box_max;  // minimum, maximum points of current AABB
    double x_min = FLT_MAX, y_min = FLT_MAX, z_min = FLT_MAX;
    double x_max = -FLT_MAX, y_max = -FLT_MAX, z_max = -FLT_MAX;
    
    for(int f = start; f <= end; f++) 
    {
        // for all vertices in the face
        for (int v = 0; v < 3; v++)
        {
            // Get the current vertex position
            cur_v = getVertexPos(f, v);

            // Compute the minimum, maximum points of the current face
            x_min = (x_min > cur_v.x()) ? cur_v.x() : x_min;
            y_min = (y_min > cur_v.y()) ? cur_v.y() : y_min;
            z_min = (z_min > cur_v.z()) ? cur_v.z() : z_min;

            x_max = (x_max < cur_v.x()) ? cur_v.x() : x_max;
            y_max = (y_max < cur_v.y()) ? cur_v.y() : y_max;
            z_max = (z_max < cur_v.z()) ? cur_v.z() : z_max;
        }
    }
    box_min = Vec3{x_min, y_min, z_min};
    box_max = Vec3{x_max, y_max, z_max};

    return Aabb{box_min, box_max};
}

// #BVH #DEBUGGING
void Mesh::printAabbs() const
{
    // Get all faces(triangles)
    int num_f = getNumFaces();
    vector<Face> m_faces = getFaces();

    cout << "======================================" << "Printing Mesh AABB Information ======================================" << endl;

    // Print AABB information of each face
    cout << "<Faces>" << endl << endl;
    for(int f = 0; f < num_f; f++)
    {
        Face cur_f = m_faces[f];
        cout << "Face " << f << endl;

        Point3 box_min = cur_f.box.min();
        Point3 box_max = cur_f.box.max();

        cout << "Minimum P = (" << box_min << "), Maximum P = (" << box_max << ")" << endl;
        cout << endl;
    }    
    cout << "=================================================================================================" << endl << endl;
}


// Construct BVH that includes all triangles in the mesh
void Mesh::constructBvh()
{
    int obj_num = getNumFaces();
	int node_num = 2 * obj_num - 1;

    //cout << "FACES # = " << obj_num << endl;
    //cout << "NODES # = " << node_num << endl;


    vector<BvhNode> bvh;

	// Root node
    Aabb cur_box = computeAabb(0, obj_num-1);
    bvh.emplace_back(0, 0, obj_num-1, -1, -1, cur_box);

	// Intermediate nodes
    int count = 0;  // number of nodes created so far
	for(int idx = 0; idx < node_num; idx++) {

        // Pop a node from the stack
        BvhNode* cur_node = &bvh[idx];  // current node
        
        int f_start = cur_node->f_first;
		int f_end = cur_node->f_last;
		int f_span = f_end - f_start;

        // Compute AABB of the current node
        //Aabb cur_box = computeAabb(f_start, f_end);


        // #DEBUGGING
        /*
        cout << "Node " << idx << endl;
        cout << "  - Faces: " << f_start << "~" <<  f_end << endl;
        cout << "  - AABB: (" << cur_node->box.min() << ") ~ (" << cur_node->box.max() << ")" << endl; 
        */


        // Choose an axis
        //int axis = 0;
        int axis = random_int(0, 2);  

        
        // If the current node bounds "two or more" triangles
        if(f_span >= 1)
        {
            int f_mid = f_start + f_span/2;  // split index

            // indices of left, right child nodes
            //int l_idx = 2*idx+1; 
            //int r_idx = l_idx+1;
            int l_idx = count+1;
            int r_idx = count+2;


            count += 2;
            

            // Sort the triangles
            for(int i = f_start; i < f_end; i++)
            {
                for(int j = f_start; j < f_end-i; j++)
                {
                    Aabb box_a = getAabb(j+1);
                    Aabb box_b = getAabb(j);
                    
                    if(compareAabbs(box_a, box_b, axis)) 
                    {
                        Face temp = m_faces[j];
                        m_faces[j] = m_faces[j+1];
                        m_faces[j+1] = temp;
                    }
                }
            }
            // Create left, right child nodes
            if(l_idx > 0 && l_idx < node_num)  
            {
                //cur_node->left = l_idx;  // Update current node's left child index
                bvh[idx].left = l_idx;

                cur_box = computeAabb(f_start, f_mid);
                bvh.emplace_back(l_idx, f_start, f_mid, -1, -1, cur_box);

				//bvh[l_idx] = BvhNode{f_start, f_mid};                 
			} 
            else 
            {
                cout << "BVH Node " << l_idx << "cannot be created" << endl;
            }
            if(r_idx > 0 && r_idx < node_num) 
            {
                // 왜 루트 노드만 오른쪽 자식 노드 인덱스가 자꾸 -1이 되는 걸까...
                //cur_node->right = r_idx;  // Update current node's right child index
                bvh[idx].right = r_idx;

                cur_box = computeAabb(f_mid+1, f_end);
                bvh.emplace_back(r_idx, f_mid+1, f_end, -1, -1, cur_box);
                
                //bvh[r_idx] = BvhNode{f_mid+1, f_end};
			} 
            else 
            {
                cout << "BVH Node " << r_idx << "cannot be created" << endl;
            }
        }
/*
        // Print the current sorting result #DEBUGGING 
        cout << "Mesh Faces" << endl;
        for(int f=0; f<obj_num; f++) 
        {
            cout << "  - Face " << f << ": ";
            for (auto element : m_faces[f].vertices) 
            {
               cout << element+1 << " ";
            } 
            cout << endl;
        }
*/
    }
    
    //m_bvh = std::move(bvh);
    setBvh(bvh);

    //cout << "\n\n================================== BVH CONSTURCTION COMPLETED ==================================\n\n" << endl;
}


// Print the BVH
/*
void Mesh::printBvh() const
{
	int node_num = 2 * getNumFaces() - 1;
	
    // Print all nodes
    for(int n = 0; n < node_num; n++) 
    {
		BvhNode cur_node = m_bvh[n];
		Aabb cur_box = cur_node.box;	

		BvhNode left_node = m_bvh[2*n+1];
		BvhNode right_node = m_bvh[2*n+2];

		int obj_num = cur_node.f_last - cur_node.f_first + 1;

		cout << "------------------------------ Node " << cur_node.idx << " INFO (obj # = " << obj_num << ") ------------------------------\n"
            << "  - Faces: " << cur_node.f_first << "~" << cur_node.f_last << endl
            << "  - AABB: " << cur_box.min() << " ~ " << cur_box.max() << endl;
	
        // If leaf node
        if(obj_num < 2)
        {
            // Print all triangles of the leaf
            //cout << "------------------------------ Node " << cur_node.idx << " FACE INFO (obj # = " << obj_num << ") ------------------------------" << endl;
            int f_start = cur_node.f_first;
            int f_end = cur_node.f_last;

            for(int f = f_start; f <= f_end; f++) 
            {
                cout << "  - Face[" << f << "] ";
                Face cur_f = m_faces[f];
                
                // index values of its vertices
                cout << "Vs: ";
                for (auto element : cur_f.vertices) {
                    cout << element+1 << " ";
                }
                cout << endl;
            }
            cout << endl;
        }   
    }
    

}
*/


void Mesh::printBvh() const
{
    vector<int> stack;  // stack (storing node index)

    cout << "\n\n================================== PRINT BVH ==================================\n" << endl;

    // root node
    stack.push_back(0);  // Push the root node index into the stack

	while(!stack.empty())  // while the stack is not empty
	{
        // #DEBUGGING
        // Print the current stack
        cout << "[Stack] ";
        for (auto element : stack) {
            cout << "N" << element << " ";
        }
        cout << endl;

        // Pop one node from the stack top
        BvhNode cur_node = m_bvh[stack.back()];
        stack.pop_back();
        Aabb cur_box = cur_node.box;	
        int f_num = cur_node.f_last - cur_node.f_first + 1;

        // Print the current stack
        cout << "[Stack] ";
        for (auto element : stack) {
            cout << "N" << element << " ";
        }
        cout << endl;

        // Print the current node info
		cout << "------------------------------ Node " << cur_node.idx << " INFO (obj # = " << f_num << ") ------------------------------\n"
            << "  - Faces: " << cur_node.f_first << "~" << cur_node.f_last << endl
            << "  - Left: N" << cur_node.left << ", Right: N" << cur_node.right << endl
            << "  - AABB: (" << cur_box.min() << ") ~ (" << cur_box.max() << ")" << endl;

		// Check the node type		
        // 1) leaf node
        if (f_num == 1) {  
			Face cur_f = m_faces[cur_node.f_first];  // face(triangle) in the current node
				
            // Print the face info
            cout << "  - Leaf Node: ";
            cout << "F[" << cur_node.f_first << "]" << endl;          
            for (auto element : cur_f.vertices) {
                cout << "V" << element+1 << " ";  // vertex indices
            }
            // vertex position
            const Vec3& v0 = m_vertices[cur_f.vertices[0]].position;
            const Vec3& v1 = m_vertices[cur_f.vertices[1]].position;
            const Vec3& v2 = m_vertices[cur_f.vertices[2]].position;

            cout << " = (" << v0 << "), (" << v1 << "), (" << v2 << ")" << endl;
        }

        // 2) intermediate node
		else if (f_num > 1) {  
            stack.push_back(cur_node.right);  // (or child_r->idx)
            stack.push_back(cur_node.left);  // (or child_l->idx)
		}
    }
    cout << "\n\n===============================================================================\n" << endl;
}


/*
bool Mesh::compareAabbs(int f1, int f2, int axis) const
{
	Aabb box_a = getAabb(f1);
    Aabb box_b = getAabb(f2);
    	
    return box_a.mid().e[axis] < box_b.mid().e[axis];
}
*/

inline bool compareAabbs(Aabb &b1, Aabb &b2, int axis) 
{   	
//    return b1.min().e[axis] < b2.min().e[axis];
    return b1.mid().e[axis] < b2.mid().e[axis];

}

///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// #BVH #TRAVERSAL
bool Mesh::searchBvh( 
//	const int STACK_SIZE, 
	const Ray& ray,
//	double t_min, double t_max,
	HitRecord &rec) 
{
    vector<int> stack;  // stack (storing node index)
	bool is_hit = false;
    double t_min = FLT_MAX; // current closest hit time

    //cout << "\n\n================================== BVH TRAVERSAL ==================================\n" << endl;

    rec.t = FLT_MAX;

    // root node
//    BvhNode* cur_node = &m_bvh[0];
    stack.push_back(0);  // Push the root node index into the stack

	while(!stack.empty())  // while the stack is not empty
	{
        // #DEBUGGING
        /*
        // Print the stack
        cout << endl << "[Stack] ";
        for (auto element : stack) {
            cout << "N" << element << " ";
        }
        cout << endl;
        */


        // Pop one node from the stack top
        BvhNode cur_node = m_bvh[stack.back()];
        stack.pop_back();


        // Do the ray-node intersection test
        if(!cur_node.hit(ray, t_min))
        {
            // not hit
            //cout << "Node " << cur_node.idx << " is NOT hit" << endl;  // #DEBUGGING
            
            ////#mistake #실수!!!!! 현재 노드랑 만나지 않는다고 아예 BVH 테스트 결과를 false로 반환해버리면 안됨....ㅠㅠ
            //return false;
            continue;
        }

        // hit
        //cout << "Node " << cur_node.idx << " is hit" << endl;  // #DEBUGGING

		// Check the node type
		int f_num = cur_node.f_last - cur_node.f_first + 1;  // # of objects in the current node
		
        // 1) leaf node
        if (f_num == 1) {  
			Face cur_f = m_faces[cur_node.f_first];  // face(triangle) in the current node

            /*	
            // #DEBUGGING /////////////////////////////////////
            // Print the current node info
            cout << "  - Face[" << cur_node.f_first << "] ";          
            cout << "Vs: ";
            for (auto element : cur_f.vertices) {
                cout << element+1 << " ";
            }
            cout << endl;
			///////////////////////////////////////////////////
            */

            // Get face(triangle) info
/*
            const Vec3& v0 = m_vertices[cur_f.vertices[0]].position;
            const Vec3& v1 = m_vertices[cur_f.vertices[1]].position;
            const Vec3& v2 = m_vertices[cur_f.vertices[2]].position;
*/
            //cout << "Triangle Vs: (" << v0 << "), (" << v1 << "), (" << v2 << ")" << endl;

            const Vec3& v0 = getVertexPos(cur_node.f_first, 0);
            const Vec3& v1 = getVertexPos(cur_node.f_first, 1);
            const Vec3& v2 = getVertexPos(cur_node.f_first, 2);


            // distance between origin and hit point
            double t_temp;

            // Output barycentric coordinates of the intersection point
            Vec3 bary;
		

		    // Intersection test
			// Do the ray-triangle intersection test
			//if(cur_f->hit(ray, (is_hit_first)?rec.t:t_min, rec)) {  // If the current object is hit
    		//if(cur_f->hit(ray, t_min, rec)) {  // If the current object is hit
    		if (cur_f.hit(ray, &t_temp, bary, v0, v1, v2))  // Check if ray intersects the triangle            
            //if (testRayTriangleHit(ray, &t_temp, bary, v0, v1, v2))  // Check if ray intersects the triangle            

            {  
                //cout << "Triangle Hit at t = " << t_temp << endl;
                // Update hit record only if hit earlier than the recorded one
                if(t_temp >= 0.000001 && t_temp < t_min) 
                {
                    rec.t = t_temp;   // intersection time
                    rec.p = ray.orig + rec.t * ray.dir;  // intersection point

                    bary = getBarycentric(rec.p, v0, v1, v2);

                    rec.normal = getPointNorm(cur_node.f_first, bary.x(), bary.y());  // normal at the intersection point
                    rec.is_front_face = dot(ray.direction(), rec.normal) < 0;
                    rec.mat_ptr = m_material;

                    //cout << "[Update Hit Record] t: " << rec.t << ", p: (" << rec.p << ")" << endl;  // debugging
                
                    // Update temporary variables
                    t_min = rec.t; 
                    is_hit = true;
              
                }      
			} 
		}

		// 2) intermediate node
		else if (f_num > 1) {  
            //cout << "Internal Node" << endl;
            const BvhNode child_r = m_bvh[cur_node.right];  // right child
			const BvhNode child_l = m_bvh[cur_node.left];  // left child


            // #mistake #실수 
            // 현재 노드와 레이가 만난다면, 자식 노드들은 바로 스택에 넣는다 (자식 노드까지 충돌 테스트하고 넣는 게 아니다! 테스트를 이중으로 하게 되므로...)
            /*
			// Do the intersection test of the child nodes
            bool hit_r = child_r.hit(ray, t_min); 
			bool hit_l = child_l.hit(ray, t_min);

            if(hit_r)   stack.push_back(cur_node.right);  // (or child_r->idx)
            if(hit_l)   stack.push_back(cur_node.left);  // (or child_l->idx)
            */

            // 
            stack.push_back(cur_node.right);  // (or child_r->idx)
            stack.push_back(cur_node.left);  // (or child_l->idx)
		}
	}	
    //cout << "\n\n===============================================================================\n" << endl;

	return is_hit;
}

///////////////////////////////////////////////////////////////////////////////////////////////////


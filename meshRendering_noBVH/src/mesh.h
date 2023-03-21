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




// Preprocessors
#pragma once

#include <string>
#include <vector>
#include <array>

#include <fstream>
#include <iostream>
#include <cassert>

#include "vec3.h"
#include "ray.h"
#include "hittable.h"

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
    
	Vertex(Vec3& position, Vec3& normal) : position(position), normal(normal) {}
};

struct Face
{
    // Stores indices
//    int* vertices;
//   int* normals;

    array<int, POLYGON> vertices;
	array<int, POLYGON> normals;

    Face(const array<int, POLYGON>& vertices, const array<int, POLYGON>& normals) :
        vertices(vertices),
        normals(normals) 
    {}

//    Face(int vertices[], int normals[]) : vertices(vertices), normals(normals) {}
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

    bool hit(const Ray& ray, double min_t, HitRecord& rec);


private:

//	void updateBoundingBox();
    vector<Vertex> m_vertices;  // stores actual values of its position and normal vector elements
    vector<Face> m_faces;  // stores indices of its vertices and normals
//	AABB m_boundingBox;
    shared_ptr<Material> m_material;

};


// Function Prototypes
void printMesh(string filename, Mesh& mesh);
bool loadObjFile(string filename, Mesh& mesh);
bool loadObjFile(string filename, vector<Vertex>& vertices, vector<Face>& faces);
bool testRayTriangleHit(const Ray& ray, double *t, Vec3& bary, const Vec3& v0, const Vec3& v1, const Vec3& v2);
Vec3 getBarycentric(Vec3 &p, const Vec3& v0, const Vec3& v1, const Vec3& v2); 


// Function Definitions
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


// loadObjFile: Load a mesh (.obj) file
bool loadObjFile(string filename, Mesh& mesh) 
{
    vector<Vertex> m_vertices;
    vector<Face> m_faces;

    bool success = loadObjFile(filename, m_vertices, m_faces);
    
    mesh.setVertices(m_vertices);
    mesh.setFaces(m_faces);

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

        cout << line_type << " ";
        
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

                cout << e[i] << " ";  // debugging
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

                cout << e[i] << " ";  // debugging
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
                    cout << v_e[i] << " ";  // debugging
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

                cout << v_e[i] << "//" << n_e[i] << " ";  // debugging
               
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
    cout << endl;
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
        array<int, POLYGON> cur_n_idx = { {-1, -1, -1} };

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
		if (testRayTriangleHit(ray, &t_temp, bary, v0, v1, v2))  // Check if ray intersects the triangle
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
            
//                cout << "  *Update Hit Record* t: " << rec.t << ", p: (" << rec.p << ")" << endl;  // debugging

                is_hit = true;
            }
		}
//        cout << endl;
	}
    return is_hit;
}

//bool rayTriangleHit(const Ray &ray, HitRecord &rec, const Vertex& v0, const Vertex& v1, const Vertex& v2) const {
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

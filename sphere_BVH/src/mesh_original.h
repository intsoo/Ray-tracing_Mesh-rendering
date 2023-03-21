/*
 * ===================================================
 *
 *       Filename:  mesh.h
 *    Description:  Further Improvements of <Ray Tracing In One Weekend>
 *        Created:  2023/01/04
 * 
 * ===================================================
 */

// Preprocessors
#include <array>
#include <filesystem>
#include <string>
#include <vector>
#include <glm/glm.hpp>

#include <fstream>
#include <iostream>
#include <sstream>

#include <glm/gtx/intersect.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include "MathUtils.h"

#include "AABB.h"
#include "Ray.h"
#include "Material.h"

using std::shared_ptr;
using std::make_shared;

struct MeshVertex
{
	Vec3 position;
	Vec3 normal;

	MeshVertex(const Vec3& position, const Vec3& normal) : position(position), normal(normal) {}
};

struct MeshFace
{
	int m_vertices[3];
	int m_normals[3];

	MeshFace(int* vertices, int* normals) : m_vertices(vertices), m_normals(normals) {}
    
};

class Mesh
{
public:

    // Constructors
    Mesh::Mesh() : m_boundingBox({0.0, 0.0, 0.0}, { 0.0, 0.0, 0.0 }), m_material(nullptr) {}
    Mesh::Mesh(std::vector<MeshVertex> vertices, std::vector<MeshFace> faces) :
        m_vertices(std::move(vertices)),
        m_faces(std::move(faces)),
        m_boundingBox({ 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }),
        m_material(nullptr) 
    {
        updateBoundingBox();
    }

    // Setter Functions
	void setVertices(std::vector<MeshVertex> vertices) const override;
	void setFaces(std::vector<MeshFace> faces) const override;
	void setMaterial(std::shared_ptr<Material> material) const override;

    // Getter Functions
	const std::vector<MeshVertex>& vertices() const override;
	const std::vector<MeshFace>& faces() const override;
	const std::shared_ptr<Material>& material() const override;
	const AABB& boundingBox() const override;

    // Compute sizes
	int numVertices() const override;  // the number of mesh vertices
	int numFaces() const override;  // the number of mesh faces

    // Return selected values 
	[[nodiscard]] Vec3 vertex(int idx_face, int idx_vertex) const override;  // 3D coordinates of target vertex
	[[nodiscard]] Vec3 normal(int idx_face, int idx_vertex) const override;  // 3D coordinates of normal at given vertex
	[[nodiscard]] Vec3 normal(int idx_face, double u, double v) const override;  // normal at the point inside the triangle (face), interpolated with barycentric coordinates

	void applyTransformation(const Mat4& transformation) const override;  // Apply the given transformation
    void updateBoundingBox();


private:
	std::vector<MeshVertex> m_vertices;
	std::vector<MeshFace> m_faces;
    shared_ptr<material> m_material;
	aabb m_boundingBox;
};


// 1. Member Functions

// Setters
void Mesh::setVertices(std::vector<MeshVertex> vertices) const {
	m_vertices = std::move(vertices);

	updateBoundingBox();
}

void Mesh::setFaces(std::vector<MeshFace> faces) const {
	m_faces = std::move(faces);
}

void Mesh::setMaterial(std::shared_ptr<Material> material) const {
	m_material = std::move(material);
}


// Getters
const std::vector<MeshVertex>& Mesh::vertices() const {
	return m_vertices;
}

const std::vector<MeshFace>& Mesh::faces() const {
	return m_faces;
}

const std::shared_ptr<Material>& Mesh::material() const {
	return m_material;
}

const AABB& Mesh::boundingBox() const {
	return m_boundingBox;
}


// Compute sizes
int Mesh::numVertices() const {
	return (int)(m_vertices.size());
}

int Mesh::numFaces() const {
	return (int)(m_faces.size());
}


// Return selected values
Vec3 Mesh::vertex(int idx_face, int idx_vertex) const {

    // Check whether the given indices are out of range
	assert(idx_face >= 0 && idx_face < numFaces());
	assert(idx_vertex >= 0 && idx_vertex < 3);

    // Get 3D coordinates of the target vertex
	auto idx = m_faces[idx_face].vertices[idx_vertex];
	assert(idx >= 0 && idx < numVertices());

	return m_vertices[idx].position;
}

// Compute normal at the given vertex
Vec3 Mesh::normal(int idx_face, int idx_vertex) const {
	assert(idx_face >= 0 && idx_face < numFaces());
	assert(idx_vertex >= 0 && idx_vertex < 3);

    // Get 3D coordinates of the target normal
    auto idx = m_faces[idx_face].normals[idx_vertex];

    // If the normal is already defined in the mesh
	if (idx >= 0 && idx < numVertices()) {
		return m_vertices[idx].normal;
	}
    // If not, compute the normal manually
	else {
        // Get 3D coordinates of the three vertices making up the given triangle (face)
	    auto v0 = vertex(idx_face, 0);
		auto v1 = vertex(idx_face, 1);
		auto v2 = vertex(idx_face, 2);

		return glm::normalize(glm::cross(v1 - v0, v2 - v0));  // normal of a plane == cross product of two vectors on the plane
	}
}

// Compute normal at a point located inside the given triangle (face)
Vec3 Mesh::normal(int idx_face, double u, double v) const
{
    // Compute normals at the three vertices making up the triangle (face)
	auto n0 = normal(idx_face, 0);
	auto n1 = normal(idx_face, 1);
	auto n2 = normal(idx_face, 2);

    // Compute the normal at the given point by using interpolation with barycentric coordinates 
	auto norm_vec = (1.0 - u - v) * n0 + u * n1 + v * n2;  

	return glm::normalize(norm_vec);  
}


void Mesh::applyTransformation(const Mat4& transformation) const {
	const auto normal_transform = glm::inverseTranspose(transformation);

	// Transform all vertices and normals
    int vert_num = m_vertices.size();
	for (int i = 0; i < vert_num; i++)
	{
		m_vertices[i].position = mapPoint(transformation, m_vertices[i].position);
		m_vertices[i].normal = mapVector(normalTransformation, m_vertices[i].normal);
	}

	updateBoundingBox();
}

void Mesh::updateBoundingBox() const {
	if (!m_vertices.empty())
	{
		Vec3 a = m_vertices.front().position;
		Vec3 b = m_vertices.front().position;

		for (const auto& v : m_vertices)
		{
			a = glm::min(a, v.position);
			b = glm::max(b, v.position);
		}

		m_boundingBox = AABB(a, b);
	}
}


// 2. Other Functions
// Loading meshes

int indexOfNumberLetter(const std::string& str, int offset)
{
	const auto length = static_cast<int>(str.length());

	for (int i = offset; i < length; ++i)
	{
		if ((str[i] >= '0' && str[i] <= '9') || str[i] == '-' || str[i] == '.') return i;
	}

	return length;
}

int lastIndexOfNumberLetter(const std::string& str)
{
	const auto length = static_cast<int>(str.length());

	for (int i = length - 1; i >= 0; --i)
	{
		if ((str[i] >= '0' && str[i] <= '9') || str[i] == '-' || str[i] == '.') return i;
	}

	return length;
}

std::vector<std::string> split(const std::string& s, char delim)
{
	std::vector<std::string> elems;

	std::stringstream ss(s);
	std::string item;
	while (getline(ss, item, delim))
	{
		elems.push_back(item);
	}

	return elems;
}

// Load a mesh file
bool loadMesh(const std::filesystem::path& filename, std::vector<MeshVertex>& vertices, std::vector<MeshFace>& faces)
{
	std::ifstream file(filename);

	if (!file.is_open())
	{
		std::cerr << "Mesh::load() - Could not open file " << filename << std::endl;
		return false;
	}

	// Store vertex and normal data while reading
	std::vector<Vec3> raw_vertices;
	std::vector<Vec3> raw_normals;
	std::vector<int> v_elements;
	std::vector<int> n_elements;

	std::string line;
	while (getline(file, line))
	{
        // Vertex
		if (line.substr(0, 2) == "v ")
		{
			// Read position data
			int index1 = indexOfNumberLetter(line, 2);
			int index2 = lastIndexOfNumberLetter(line);
			auto values = split(line.substr(index1, index2 - index1 + 1), ' ');
			raw_vertices.emplace_back(stof(values[0]), stof(values[1]), stof(values[2]));
		}
        // Vertex Normal
		else if (line.substr(0, 3) == "vn ")
		{
			// Read normal data
			int index1 = indexOfNumberLetter(line, 2);
			int index2 = lastIndexOfNumberLetter(line);
			auto values = split(line.substr(index1, index2 - index1 + 1), ' ');
			raw_normals.emplace_back(stof(values[0]), stof(values[1]), stof(values[2]));
		}
        // Face
		else if (line.substr(0, 2) == "f ")
		{
			// Read face data
			int index1 = indexOfNumberLetter(line, 2);
			int index2 = lastIndexOfNumberLetter(line);
			auto values = split(line.substr(index1, index2 - index1 + 1), ' ');
			for (int i = 0; i < static_cast<int>(values.size()) - 2; i++)
			{
				// Split up vertex indices
				auto v1 = split(values[0], '/'); // Triangle fan for ngons
				auto v2 = split(values[i + 1], '/');
				auto v3 = split(values[i + 2], '/');

				// Store position indices
				v_elements.push_back(std::stoul(v1[0]) - 1);
				v_elements.push_back(std::stoul(v2[0]) - 1);
				v_elements.push_back(std::stoul(v3[0]) - 1);

				// Check for normals
				if (v1.size() >= 3 && v1[2].length() > 0)
				{
					n_elements.push_back(std::stoul(v1[2]) - 1);
					n_elements.push_back(std::stoul(v2[2]) - 1);
					n_elements.push_back(std::stoul(v3[2]) - 1);
				}
			}
		}
	}

	file.close();

	// Resize vertices and faces vectors
	vertices.clear();
	vertices.reserve(raw_vertices.size());

    // Store each vertice and normal sequentially
	for (unsigned int i = 0; i < std::max(raw_vertices.size(), raw_normals.size()); i++)
	{
		Vec3 vertex;
		Vec3 normal;

		if (i < raw_vertices.size())
		{
			vertex = raw_vertices[i];
		}

		if (i < raw_normals.size())
		{
			normal = raw_normals[i];
		}

		vertices.emplace_back(vertex, normal);
	}

	faces.clear();
	faces.reserve(std::max(v_elements.size(), n_elements.size()) / 3);

    // Store each triangle (face) sequentially
	for (unsigned int i = 0; i < std::max(v_elements.size(), n_elements.size()); i += 3)
	{
		std::array<int, 3> v = { {0, 0, 0} };
		std::array<int, 3> n = { {-1, -1, -1} };

		if (i + 2 < v_elements.size())  // three vertices of the triangle
		{
			v[0] = v_elements[i];
			v[1] = v_elements[i + 1];
			v[2] = v_elements[i + 2];
		}

		if (i + 2 < n_elements.size())  // three normals at the vertices
		{
			n[0] = n_elements[i];
			n[1] = n_elements[i + 1];
			n[2] = n_elements[i + 2];
		}

		faces.emplace_back(v, n);
	}

	return true;
}


bool loadMesh(const std::filesystem::path& filename, Mesh& mesh)
{
	std::vector<MeshVertex> vertices;
	std::vector<MeshFace> faces;

    auto success = loadMesh(filename, vertices, faces);  // Get vertices and faces of the mesh

	mesh.setVertices(vertices);
	mesh.setFaces(faces);

	return success;
}

bool rayMeshIntersection(
	const Mesh& mesh,
	const Ray& ray,
	double t_min,
	hit_record& rec)
{
	// First, intersect ray with AABB to quickly discard non-intersecting rays
	if (!rayAABBIntersection(mesh.boundingBox(), ray))
	{
		return false;
	}

	const auto& orig = ray.origin();
	const auto& dir = ray.direction();
	bool is_hit = false;
	rec.t = std::numeric_limits<double>::max(); 

	// Iterate over all triangles (faces) in the mesh
    int num_faces = mesh.numFaces();
	for (int f = 0; f < num_faces; f++)
	{
		const auto& v0 = mesh.vertex(f, 0);
		const auto& v1 = mesh.vertex(f, 1);
		const auto& v2 = mesh.vertex(f, 2);

		float t;  // distance between ray origin and hit point
		Vec2 baryPosition;  // barycentric coordinates of the intersection point

		// Check if there is an intersection with this triangle
		if (glm::intersectRayTriangle(orig, dir, v0, v1, v2, baryPosition, t)
			&& t >= t_min && t < rec.t)
		{
            // Update the hit record
			rec.t = t;
			rec.p = ray.at(rec.t);
			rec.normal = mesh.normal(f, baryPosition.x, baryPosition.y);
			rec.front_face = glm::dot(ray.direction(), rec.normal) < 0;
			rec.mat_ptr = mesh.material();

			is_hit = true;
		}
	}
	return is_hit;
}


bool rayMeshesIntersection(
	const std::vector<Mesh>& meshes,
	const Ray& ray,
	double t_min,
	HitRecord& rec)
{
	bool is_hit = false;

	rec.t = std::numeric_limits<double>::max();

	// Iterate over all triangles in the mesh
    int num_faces = meshes.size();
	for (int m = 0; m < num_faces; m++)
	{
		HitRecord cur_rec;

		// Check if there is an intersection with this triangle
		if (rayMeshIntersection(meshes[m], ray, t_min, cur_rec) && cur_rec.t < rec.t)
		{
			rec = cur_rec;

			is_hit = true;
		}
	}
	return is_hit;
}
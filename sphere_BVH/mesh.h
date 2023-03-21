/*
 * ===================================================
 *
 *       Filename:  mesh.h
 *    Description:  Further Improvements of <Ray Tracing In One Weekend>
 *        Created:  2023/01/04
 * 
 * ===================================================
 */

#include "vec3.h"

struct vertex
{
    vec3 position;
    vec3 normal;
}

struct face
{
    int mesh_vertices[3];
    int mesh_normals[3];
}

class mesh
{
    public:
    // Constructors
    mesh::mesh() : mesh_aabb({0.0, 0.0, 0.0}, { 0.0, 0.0, 0.0 }), mesh_material(nullptr) {}
    mesh::mesh(std::vector<vertex> vertices, std::vector<face> faces) :
        mesh_vertices(std::move(vertices)),
        mesh_faces(std::move(faces)),
        mesh_boundingBox({ 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }),
        mesh_material(nullptr) 
    {
        //updateBoundingBox();
    }

    // Mesh Initialization
    void setMesh(
        std::vector<vertex> vertices, 
        std::vector<face> faces, 
        std::shared_ptr<material> material) const override;
    

    private:
	std::vector<vertex> mesh_vertices;
	std::vector<face> mesh_faces;
    shared_ptr<material> mesh_material;
	aabb mesh_aabb;

}


bool loadMesh(const std::filesystem::path& filename, std::vector<vertex>& vertices, std::vector<face>& faces)
{
    std::ifstream file(filename);

    // If the file cannot be opened
	if (!file.is_open())
	{
		std::cerr << "Mesh::load() - Could not open file " << filename << std::endl;
		return false;
	}

	// Read the file and store raw vertex & normal data
	std::vector<vec3> raw_vertices;
	std::vector<Vvec3ec3> raw_normals;
	std::vector<int> v_elements;
	std::vector<int> n_elements;

	std::string line;  // line of the file

	while (getline(file, line))  // Read by line
	{
        // Identify the line type
        // 1) Vertex Line
		if (line.substr(0, 2) == "v ")
		{
			// Read vertex position data
			int index1 = indexOfNumberLetter(line, 2);
			int index2 = lastIndexOfNumberLetter(line);
			auto values = split(line.substr(index1, index2 - index1 + 1), ' ');
			raw_vertices.emplace_back(stof(values[0]), stof(values[1]), stof(values[2]));
		}
        // 2) Vertex Normal Line
		else if (line.substr(0, 3) == "vn ")
		{
			// Read normal data
			int index1 = indexOfNumberLetter(line, 2);
			int index2 = lastIndexOfNumberLetter(line);
			auto values = split(line.substr(index1, index2 - index1 + 1), ' ');
			raw_normals.emplace_back(stof(values[0]), stof(values[1]), stof(values[2]));
		}
        // 3) Face Line
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

    // Close the file
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
	std::vector<vertex> vertices;
	std::vector<face> faces;

    auto success = loadMesh(filename, vertices, faces);  // Get vertices and faces of the mesh

	mesh.setMesh(vertices, faces, ...);

	return success;
}


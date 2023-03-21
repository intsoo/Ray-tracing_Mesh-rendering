/*
 * ===================================================
 *
 *       Filename:  triangle.h
 *    Description:  Further Improvements of <Ray Tracing In One Weekend>
 *        Created:  2023/01/04
 * 
 * ===================================================
 */

Vector3f Triangle::get_barycentric(Vector3f &p) const {
    Vector3f v2_ = p - v0;
    float d00 = glm::dot(e1, e1);
    float d01 = glm::dot(e1, e2);
    float d11 = glm::dot(e2, e2);
    float d20 = glm::dot(v2_, e1);
    float d21 = glm::dot(v2_, e2);
    float d = d00 * d11 - d01 * d01;
    float v = (d11 * d20 - d01 * d21) / d;
    float w = (d00 * d21 - d01 * d20) / d;
    float u = 1 - v - w;
    return Vector3f(u, v, w);
}

bool Triangle::intersect(const Ray &r, SurfaceInteraction &interaction) const {
    
    float u, v, t_temp = 0.0f;

    // First, test whether the ray hits the plane that includes the triangle
    vec3 n_plane = cross(r.direction, e2);  // normal of the plane where both ray and edge2 are included
    float det = dot(e1, n_plane);  // determinant
    if (det ==  0.0f) return false;  // If the ray is parallel to plane that includes the triangle
    
    // If the ray meets the plane
    // Compute the barycentric coordinates of the intersection point
    float inv_det = 1.0f / det;  // inverse number(역수) of determinant
    Vector3f tvec = r.o - v0;  // 
    u = glm::dot(tvec, n_plane) * inv_det;
    if (u < 0.0f || u > 1.0f ) return false;
    Vector3f qvec = glm::cross(tvec, e1);
    v = glm::dot(r.d, qvec) * inv_det;
    if (v < 0.0f || u + v > 1.0f) return false;

    // Compute the intersection time
    t_temp = glm::dot(e2, qvec) * inv_det;
    if (r.t_min < t_temp && t_temp < r.t_max) {
        // Define hit record
        interaction.t = t_temp;
        interaction.p = r.o + interaction.t * r.d;  // intersection point at time t
        Vector3f outward_normal = this->n;
        interaction.Ng = this->n;
        //interaction.set_face_normal(r, outward_normal);

        // Barycentric coords
        Vector3f bary = get_barycentric(interaction.p);
        interaction.Ng = glm::normalize((bary.x * N[0]) + (bary.y * N[1]) + bary.z * N[2]);

        //interaction.AOV = bary;
        Vector2f ST = bary.x * uv[0] + bary.y * uv[1] + bary.z * uv[2];
        interaction.AOV = Vector3f(ST.x, ST.y, 0.0f);
        interaction.AOV = glm::normalize((bary.x * N[0]) + (bary.y * N[1]) + bary.z * N[2]);
        interaction.AOV = bary;

        return true;

    }
    return false;
}

bool TriangleMesh::intersect(const Ray &r, SurfaceInteraction &interaction) const {    
    bool hit_tri = false;
    long tri_idx;
    float t = r.t_max;
    SurfaceInteraction temp;
    long triangles_size = tris.size();
    for (long i = 0; i < triangles_size; ++i) {
        if (tris[i]->intersect(r, temp)) {
            hit_tri = true;
            tri_idx = i;
            r.t_max = temp.t;
            interaction = temp;
        }
    }
    return hit_tri;
}

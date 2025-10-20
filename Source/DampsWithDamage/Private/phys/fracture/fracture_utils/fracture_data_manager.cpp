#include "fracture_data_manager.h"

// style-checked
namespace pe_phys_fracture {

    uint32_t FractureDataManager::add_vertex(const pe::Vector3& p, const pe::Vector3& n) {
        const vertex new_vertex(p, n);
        const auto it = _vertices.find(new_vertex);
        if (it == _vertices.end()) {
            _vertices.push_back(new_vertex);
            return _vertices.size() - 1;
        }
        return (uint32_t)(it - _vertices.begin());
    }

    void FractureDataManager::remove_vertex(uint32_t idx, bool direct) {
        _vertices.erase(_vertices.begin() + idx);
        if (direct) return;
        TriangleHashList new_triangles;
        for (auto tri : _triangles) {
            for (auto& vert_id : tri.vert_ids) {
                if (vert_id > idx) {
                    vert_id--;
                }
            }
            new_triangles.push_back(tri);
        }
        _triangles = new_triangles;
    }

    uint32_t FractureDataManager::add_triangle(uint32_t v1, uint32_t v2, uint32_t v3) {
        const triangle new_tri(v1, v2, v3);
        const auto it = _triangles.find(new_tri);
        if (it == _triangles.end()) {
            _triangles.push_back(new_tri);
            return _triangles.size() - 1;
        }
        return UI(it - _triangles.begin());
    }

    void FractureDataManager::remove_triangle(uint32_t idx, bool direct) {
        _triangles.erase(_triangles.begin() + idx);
        if (direct) return;
        for (auto& tet : _tetrahedrons) {
            for (auto& tri_id : tet.tri_ids) {
                if (tri_id > idx) {
                    tri_id--;
                }
            }
        }
    }

    uint32_t FractureDataManager::add_tetrahedron(uint32_t t1, uint32_t t2, uint32_t t3, uint32_t t4,
                                                  uint32_t v1, uint32_t v2, uint32_t v3, uint32_t v4) {
        pe::Vector3 center;
        pe::Real radius;
        calc_tet_bounding_sphere(_vertices[v1].pos, _vertices[v2].pos,
                                 _vertices[v3].pos, _vertices[v4].pos,
                                 center, radius);
        _tetrahedrons.emplace_back(t1, t2, t3, t4, center, radius);
        return UI(_tetrahedrons.size()) - 1;
    }

    void FractureDataManager::faces_to_triangles() {
        // transform faces into triangles
        clear_triangles();
        const uint32_t face_count = _faces.size();
        for (uint32_t i = 0; i < face_count; i++) {
            const auto vert_count = UI(_faces[i].vert_ids.size());
            for (uint32_t j = 1; j < vert_count - 1; j++) {
                add_triangle(_faces[i].vert_ids[0], _faces[i].vert_ids[j], _faces[i].vert_ids[j + 1]);
            }
        }
    }

    bool FractureDataManager::add_triangle_to_face(uint32_t v1i, uint32_t v2i, uint32_t v3i) {
        const auto v1p = _vertices[v1i].pos;
        const auto v2p = _vertices[v2i].pos;
        const auto v3p = _vertices[v3i].pos;
        const auto n = (v2p - v1p).cross(v3p - v1p).normalized();
        uint32_t vs[] = {v1i, v2i, v3i};

        // use normal to identify a face
        polygon new_face(n);
        const uint32_t face_id = UI(_faces.find(new_face) - _faces.begin());

        // if the face doesn't exist, create a new one
        if (face_id == face_count()) {
            for (auto v : vs) {
                new_face.add_vert(v);
            }
            _faces.push_back(new_face);
            return true;
        }

        // check if a new vertex needs to be added or removed
        const auto count = UI(_faces[face_id].vert_ids.size());
        for (uint32_t i = 0; i < count; i++) {
            const uint32_t u1 = _faces[face_id].vert_ids[i],
                           u2 = _faces[face_id].vert_ids[(i + 1) % count],
                           u3 = _faces[face_id].vert_ids[(i + 2) % count];
            // already contains all 3 vertices: need to remove
            // note: only remove the index, not actually remove the vertex,
            // which will not affect the correctness of the mesh structure.
            // removing vertex costs a lot of time, and should be avoided.
            if ((u1 == v1i || u2 == v2i || u3 == v3i) &&
                (u1 == v2i || u2 == v3i || u3 == v1i) &&
                (u1 == v3i || u2 == v1i || u3 == v2i)) {
                auto face = _faces[face_id];
                face.remove_vert((i + 1) % count);
                _faces.replace(_faces.begin() + face_id, face);
                return true;
            }

            // only contains 2 vertices: need to add
            for (int j = 0; j < 3; j++) {
                const uint32_t v1 = vs[j], v2 = vs[(j + 1) % 3];
                if ((u1 == v1 && u2 == v2) || (u1 == v2 && u2 == v1)) {
                    auto face = _faces[face_id];
                    face.add_vert(vs[(j + 2) % 3], (i + 1) % count);
                    _faces.replace(_faces.begin() + face_id, face);
                    return true;
                }
            }
        }

        // this means the triangle is separated from the face, should be added
        // later. (until this triangle is right close to the border)
        return false;
    }

    void FractureDataManager::import_from_mesh(const pe::Mesh &mesh) {
        // import data from a new triangle or polygonal mesh,
        // and automatically merge triangles into polygonal faces
        clear();

        // add vertices
        const auto vert_count = UI(mesh.vertices.size());
        pe::Array<uint32_t> vert_ids(vert_count);
        for (uint32_t i = 0; i < vert_count; i++) {
            vert_ids[i] = add_vertex(mesh.vertices[i].position, mesh.vertices[i].normal);
        }

        // add faces: need to re-add until all triangles are added to face
        uint32_t tri_count = 0;
        for (auto& face : mesh.faces) {
            tri_count += UI(face.indices.size()) - 2;
        }
        pe::Array<bool> tri_added(tri_count, false);
        int num = 0;
        int iter_num = 0;
        while (num != tri_count) {
            if (++iter_num >= 1000) break;
            int k = 0;
            for (auto& face : mesh.faces) {
                for (uint32_t i = 0; i < face.indices.size() - 2; i++) {
                    !tri_added[k] &&
                    ((tri_added[k++] = add_triangle_to_face(
                          vert_ids[face.indices[0]],
                          vert_ids[face.indices[i + 1]],
                          vert_ids[face.indices[i + 2]]))) &&
                    ++num;
                }
            }
        }
    }

    void FractureDataManager::export_to_mesh(pe::Mesh &mesh) {
        // export the mesh data into a mesh
        faces_to_triangles();
        const uint32_t vert_size = _vertices.size(), face_size = _faces.size();

        mesh.vertices.resize(vert_size);
        mesh.faces.resize(face_size);

        // export vertices
        for (uint32_t i = 0; i < vert_size; i++) {
            mesh.vertices[i].position = _vertices[i].pos;
            mesh.vertices[i].normal = _vertices[i].nor;
        }

        // export triangles
        for (uint32_t i = 0; i < face_size; i++) {
            mesh.faces[i].indices = _faces[i].vert_ids;
            mesh.faces[i].normal = _faces[i].nor;
        }
    }

} // namespace pe_phys_fracture
#include "simple_fracture_solver.h"
#include "phys/shape/convex_mesh_shape.h"
#include "phys/shape/box_shape.h"

// style-checked
namespace pe_phys_fracture {

    void SimpleFractureSolver::cut_mesh(const pe::Mesh& mesh, pe::Array<pe::KV<pe::Mesh, bool>>& new_meshes) {
        const uint32_t point_count = _voronoi->point_count();
        FractureDataManager worker;
        worker.import_from_mesh(mesh);

        // generate the new meshes of each point
        for (uint32_t i = 0; i < point_count; i++) {
            FractureDataManager result;
            cut_one_mesh(worker, i, result);
            new_meshes.emplace_back();
            bool inside = false;
            pe::Vector3 point;
            _voronoi->get_point(i, inside);
            new_meshes.back().second = inside;
            result.export_to_mesh(new_meshes.back().first);
        }
    }

    void SimpleFractureSolver::cut_one_mesh(const FractureDataManager &mesh, uint32_t idx,
                                            FractureDataManager &new_mesh) {
        bool inside = false;
        const auto point = _voronoi->get_point(idx, inside);
        const auto adjacent_point_ids = _voronoi->get_adjacent_points(idx);

        // cut the mesh by each adjacent point
        new_mesh = mesh;
        for (const auto other_id : adjacent_point_ids) {
            auto other = _voronoi->get_point(other_id, inside);
            auto center = (point + other) / 2;
            auto normal = (other - point).normalized();
            FractureDataManager result;
            cut_mesh_by_plane(new_mesh, center, normal, result);
            new_mesh = result;
        }
    }

    void SimpleFractureSolver::cut_mesh_by_plane(FractureDataManager &old_mesh, const pe::Vector3 &p,
                                                 const pe::Vector3 &n, FractureDataManager &new_mesh) {
        const uint32_t face_count = old_mesh.face_count();
        pe::Vector3HashList inter_points(50);

        // cut all the faces
        for (uint32_t face_id = 0; face_id < face_count; face_id++) {
            for (auto& point : cut_face_by_plane(face_id, old_mesh, p, n, new_mesh)) {
                inter_points.push_back(point);
            }
        }
        if (inter_points.size() < 3) {
            return;
        }

        // add a new face of the cutting plane
        polygon new_face(n);
        auto sorted_points = inter_points.to_vector();
        std::sort(sorted_points.begin() + 1, sorted_points.end(),
                  [&](const pe::Vector3& a, const pe::Vector3& b) {
                      return (a - sorted_points[0]).cross(b - sorted_points[0]).dot(n) > 0;
                  });
        for (const auto& v : sorted_points) {
            new_face.add_vert(new_mesh.add_vertex(v, n));
        }
        new_mesh.add_face(new_face);
    }

    pe::Array<pe::Vector3> SimpleFractureSolver::cut_face_by_plane(uint32_t face_id, FractureDataManager &old_mesh,
                                                                   const pe::Vector3 &p, const pe::Vector3 &n,
                                                                   FractureDataManager &new_mesh) {
        auto face = old_mesh.get_face(face_id);
        const auto vert_count = UI(face.vert_ids.size());
        pe::Vector3HashList inter_points(vert_count);
        pe::Array<vertex> vertices(vert_count);
        pe::Array<int> side(vert_count, -1);
        pe::Uint32HashList new_point_ids(vert_count * 2);

        // check the side of each vertex to the cutting plane
        for (uint32_t i = 0; i < vert_count; i++) {
            vertices[i] = old_mesh.get_vertex(face.vert_ids[i]);
            side[i] = is_point_on_plane(p, n, vertices[i].pos) ? 0 : -1;
            side[i] = is_point_upside_plane(p, n, vertices[i].pos) ? 1 : -1;
        }

        // traverse the segments in order, and add at most 2 intersection points
        for (uint32_t i = 0; i < vert_count; i++) {
            uint32_t j = (i + 1) % vert_count;
            side[i] == 0 && inter_points.push_back(vertices[i].pos);
            side[j] == 0 && inter_points.push_back(vertices[j].pos);

            i == 0 && side[i] <= 0 &&
            new_point_ids.push_back(new_mesh.add_vertex(vertices[i].pos, vertices[i].nor));

            if (side[i] * side[j] < 0) {
                pe::Real t;
                pe::Vector3 inter;
                calc_line_plane_intersection(p, n, vertices[i].pos, vertices[j].pos, inter, t);
                auto nor = (vertices[j].nor * t + vertices[i].nor * (1 - t)).normalized();
                new_point_ids.push_back(new_mesh.add_vertex(inter, nor));
                inter_points.push_back(inter);
            }

            j != 0 && side[j] <= 0 &&
            new_point_ids.push_back(new_mesh.add_vertex(vertices[j].pos, vertices[j].nor));
        }

        // some corner cases
        if (new_point_ids.size() < 3) {
            return inter_points.to_vector();
        }

        // add the remaining face downside the cutting plane
        polygon new_face;
        auto v1 = new_mesh.get_vertex(new_point_ids[0]).pos;
        auto v2 = new_mesh.get_vertex(new_point_ids[1]).pos;
        auto v3 = new_mesh.get_vertex(new_point_ids[2]).pos;
        new_face.nor = (v2 - v1).cross(v3 - v1).normalized();
        new_face.vert_ids = new_point_ids.to_vector();
        new_mesh.add_face(new_face);
        return inter_points.to_vector();
    }

    void SimpleFractureSolver::solve(const pe::Array<FractureSource>& sources) {
        if (_fracturable_object == nullptr || sources.empty()) return;

        // generate points
        pe::Array<pe::KV<pe::Vector3, bool>> points;
        pe::Array<pe::Vector3> forces;
        UE_LOG(LogTemp, Warning, TEXT("FractureSourceCount: %ld"), sources.size());
        if (!generatePoints(sources, points, forces)) return;
        UE_LOG(LogTemp, Warning, TEXT("FracturePointsGenerated: %ld"), points.size());

        // retrieve mesh data from different shapes
        pe_phys_shape::Shape* shape = _fracturable_object->getCollisionShape();
        pe::Mesh mesh;
        if (shape->getType() == pe_phys_shape::ShapeType::ConvexMesh) {
            mesh = static_cast<pe_phys_shape::ConvexMeshShape *>(shape)->getMesh();
        } else if (shape->getType() == pe_phys_shape::ShapeType::Box) {
            mesh = static_cast<pe_phys_shape::BoxShape *>(shape)->getMesh();
        } else return;

        const pe::Transform world_trans = _fracturable_object->getTransform();

        // generate new rigidbodies
        pe::Array<pe::KV<pe::Mesh, bool>> fragments;
        _voronoi->triangulate(points);
        cut_mesh(mesh, fragments);
        UE_LOG(LogTemp, Warning, TEXT("FragmentMeshGenerated: %ld"), fragments.size());
        for (int i = 0; i < I(fragments.size()); i++) {
            if (!fragments[i].first.empty()) {
                auto filename = "fragment-" + std::to_string(i) + ".obj";
                auto rb = addMesh(fragments[i].first, fragments[i].second, world_trans, filename);
                pe::Vector3 vel = rb->getLinearVelocity();
                vel += forces[i] / rb->getMass();
                rb->setLinearVelocity(vel);
                _result.push_back(rb);
            }
        }
    }

    void SimpleFractureSolver::reset() {
        _voronoi->reset();
        _result.clear();
    }

} // namespace pe_phys_fracture
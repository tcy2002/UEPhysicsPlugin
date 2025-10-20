#include "fracture_solver.h"
#include "phys/shape/convex_mesh_shape.h"
#include <fstream>
#include <random>
#include <string>
#include "phys/fracture/fracture_utils/fracture_utils.h"

// style-checked
namespace pe_phys_fracture {

    // radius Max cannot be 0
    pe::Vector3 FractureSolver::randomRangeSpherePoints(pe::Real radiusMin, pe::Real radiusMax) {
        static std::default_random_engine e(1314);
        static std::uniform_real_distribution<> d(0., 1.);
        const pe::Real ratio = radiusMin / radiusMax, r2_1 = ratio * ratio - 1;
        const pe::Real theta = d(e) * 2 * PE_PI, alpha = (d(e) * 2 - 1) * PE_PI, roa = PE_SQRT(d(e) * r2_1 + 1) * radiusMax;
        const pe::Real cos_t = cos(theta), sin_t = sin(theta), cos_a = cos(alpha), sin_a = sin(alpha);
        return { roa * cos_t * cos_a, roa * sin_t * cos_a, roa * sin_a };
    }

    // radius Max and height Max cannot be 0
    pe::Vector3 FractureSolver::randomRangeCylinderPoints(pe::Real radius, pe::Real heightMin, pe::Real heightMax) {
        static std::default_random_engine e(COMMON_GetTickCount());
        static std::uniform_real_distribution<> d(0., 1.);
        const pe::Real ratioH = heightMin / heightMax, sq_r_1 = PE_SQRT(ratioH) - 1;
        const pe::Real theta = d(e) * 2 * PE_PI, roa = sqrt(d(e)) * radius, L = (d(e) * sq_r_1 + 1), L2 = L * L * heightMax;
        const pe::Real cos_t = cos(theta), sin_t = sin(theta);
        return { roa * cos_t, roa * sin_t, L2 };
    }

#   define EXPLOSION_RATE R(0.125)
#   define SPHERE_DENSITY 100
#   define CYLINDER_DENSITY 120

    bool FractureSolver::generatePoints(const pe::Array<FractureSource> &sources,
                                        pe::Array<pe::KV<pe::Vector3, bool>> &points, pe::Array<pe::Vector3>& forces) {
        pe_phys_shape::Shape* shape = _fracturable_object->getCollisionShape();
        const pe::Transform world_trans = _fracturable_object->getTransform();
        const pe::Real threshold = _fracturable_object->getThreshold();

        for (auto& src : sources) {
            pe::Vector3 local_impact_pos = world_trans.inverseTransform(src.position);
            pe::Vector3 intensity = src.intensity;
            pe::Real impact_radius = intensity.norm() * threshold;
            if (impact_radius < R(0.01)) continue;
            if (src.type == FractureType::Sphere) {
                int point_count = (int)(impact_radius * impact_radius * impact_radius * SPHERE_DENSITY);
                // 60% points inside the impact radius
                for (int i = 0; i <= (int)std::floor(point_count * 0.6); i++) {
                    auto point = randomRangeSpherePoints(0, impact_radius) + local_impact_pos;
                    if (shape->localIsInside(point)) {
                        points.push_back({ point, true });
                        UE_LOG(LogTemp, Warning, TEXT("FracturePointGeneratedInside: %f, %f, %f"), point.x, point.y, point.z);
                    }
                }
                // 40% points outside the impact radius
                for (int i = (int)std::ceil(point_count * 0.6); i < point_count; i++) {
                    auto point = randomRangeSpherePoints(impact_radius, 2 * impact_radius) + local_impact_pos;
                    if (shape->localIsInside(point)) {
                        points.push_back({ point, false });
                        UE_LOG(LogTemp, Warning, TEXT("FracturePointGeneratedOutside: %f, %f, %f"), point.x, point.y, point.z);
                    }
                }
            } else if (src.type == FractureType::Cylinder) {
                pe::Vector3 direction = (world_trans.getBasis().transposed() * intensity).normalized();
                pe::Matrix3 rot = from_two_vectors(pe::Vector3(0, 1, 0), direction);
                const auto point_count = (int)(impact_radius * impact_radius * impact_radius * CYLINDER_DENSITY);
                // 60% points inside the impact radiusa
                for (int i = 0; i <= (int)std::floor(point_count * 0.6); i++) {
                    auto point = rot * randomRangeCylinderPoints(impact_radius * pe::Real(0.5), 0, impact_radius * pe::Real(2)) + local_impact_pos;
                    if (shape->localIsInside(point)) {
                        points.push_back({ point, true });
                    }
                }
                // 40% points outside the impact radius
                for (int i = (int)std::ceil(point_count * 0.6); i < point_count; i++) {
                    auto point = rot * randomRangeCylinderPoints(impact_radius * pe::Real(0.5), impact_radius * pe::Real(2), impact_radius * pe::Real(3)) + local_impact_pos;
                    if (shape->localIsInside(point)) {
                        points.push_back({ point, false });
                    }
                }
            } else return false;
        }
        if (points.size() <= 2) return false;

        forces.assign(points.size(), pe::Vector3::zeros());
        for (int i = 0; i < (int)points.size(); i++) {
            pe::Vector3 pos = world_trans * points[i].first;
            for (auto& src : sources) {
                const pe::Real expForce = src.intensity.norm();
                const pe::Real dist = (pos - src.position).norm();
                const pe::Vector3 dir = (pos - src.position).normalized();
                forces[i] += dir * (expForce / dist * EXPLOSION_RATE);
            }
        }

        return true;
    }

    pe_phys_object::RigidBody* FractureSolver::addMesh(pe::Mesh& mesh, bool inside, const pe::Transform& trans, const std::string& obj_path) {
        auto rb = new pe_phys_object::RigidBody();
        auto convexMesh = new pe_phys_shape::ConvexMeshShape();
        convexMesh->setMeshPath(obj_path);
        convexMesh->setScale(pe::Vector3::ones());
        const pe::Vector3 offset = convexMesh->setMesh(mesh);
        const pe::Transform offsetTrans(pe::Matrix3::identity(), offset);
        rb->setCollisionShape(convexMesh);
        rb->setTransform(pe::Transform(trans * offsetTrans));
        rb->setMass(convexMesh->getVolume());
        rb->setFrictionCoeff(0.3);
        rb->setRestitutionCoeff(0.8);
        rb->setKinematic(!inside);
        return rb;
    }

} // namespace pe_phys_fracture
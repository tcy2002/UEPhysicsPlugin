#include "contact_result.h"
#include <algorithm>

// style-checked
namespace pe_phys_collision {

    void ContactPoint::getOrthoUnits(pe::Vector3 normal, pe::Vector3 &tangent1, pe::Vector3 &tangent2) {
        normal.normalize();
        if (PE_ABS(normal.z) > R(0.7071)) {
            // choose tangent in y-z plane
            const pe::Real a = normal.y * normal.y + normal.z * normal.z;
            const pe::Real k = R(1.0) / std::sqrt(a);
            tangent1.x = 0;
            tangent1.y = -normal.z * k;
            tangent1.z = normal.y * k;
            tangent2.x = a * k;
            tangent2.y = -normal.x * tangent1.z;
            tangent2.z = normal.x * tangent1.y;
        } else {
            // choose tangent in x-y plane
            const pe::Real a = normal.x * normal.x + normal.y * normal.y;
            const pe::Real k = R(1.0) / std::sqrt(a);
            tangent1.x = -normal.y * k;
            tangent1.y = normal.x * k;
            tangent1.z = 0;
            tangent2.x = -normal.z * tangent1.y;
            tangent2.y = normal.z * tangent1.x;
            tangent2.z = a * k;
        }
    }

    ContactPoint::ContactPoint():
        _world_pos(PE_VEC_MAX),
        _world_normal(pe::Vector3::up()),
        _local_pos_a(pe::Vector3::zeros()),
        _local_pos_b(pe::Vector3::zeros()),
        _distance(PE_REAL_MAX),
        _applied_impulse(pe::Vector3::zeros()) {}

    ContactPoint::ContactPoint(const pe::Vector3& world_pos, const pe::Vector3& world_normal,
                               const pe::Vector3& local_pos_a, const pe::Vector3& local_pos_b, pe::Real distance):
        _world_pos(world_pos),
        _world_normal(world_normal),
        _local_pos_a(local_pos_a),
        _local_pos_b(local_pos_b),
        _distance(distance),
        _applied_impulse(pe::Vector3::zeros()) {
        _tangents.resize(4);
        getOrthoUnits(world_normal, _tangents[0], _tangents[1]);
        _tangents[2] = -_tangents[0];
        _tangents[3] = -_tangents[1];
    }

    ContactResult::ContactResult():
        _friction_coeff(0),
        _restitution_coeff(0),
        _point_size(0),
        _swap_flag(false) {}

    void ContactResult::setObjects(pe_phys_object::RigidBody* object_a,
                                   pe_phys_object::RigidBody* object_b) {
        _object_a = object_a;
        _object_b = object_b;
        _friction_coeff = std::sqrt(object_a->getFrictionCoeff() * object_b->getFrictionCoeff());
        _restitution_coeff = object_a->getRestitutionCoeff() * object_b->getRestitutionCoeff();
    }

    void ContactResult::addContactPoint(const pe::Vector3& world_normal,
                                        const pe::Vector3& world_pos, pe::Real depth) {
        if (_object_a == nullptr || _object_b == nullptr) {
            return;
        }

        pe::Vector3 point_a = world_pos;
        pe::Vector3 point_b = world_pos;
        pe::Vector3 n = world_normal;
        pe::Vector3 point = world_pos;

        if (_swap_flag) {
            n = -n;
            point_b = world_pos + world_normal * depth;
            point = point_b;
        } else {
            point_a = world_pos + world_normal * depth;
        }

        const pe::Vector3 local_pos_a = _object_a->getTransform().inverseTransform(point_a);
        const pe::Vector3 local_pos_b = _object_b->getTransform().inverseTransform(point_b);

        // find the same closest point
        const int cp_idx = getExistingClosestPoint(local_pos_b);
        if (cp_idx >= 0) {
            // if found, update the contact point info when the new depth is bigger
            if (depth < _points[cp_idx].getDistance()) {
                _points[cp_idx] = ContactPoint(point, n, local_pos_a, local_pos_b, depth);
            }
        } else {
            // otherwise, find an empty slot and replace it
            bool found = false;
            for (int i = 0; i < PE_CONTACT_CACHE_SIZE; i++) {
                if (!_points[i].isValid()) {
                    _points[i] = ContactPoint(point, n, local_pos_a, local_pos_b, depth);
                    _points[i].setAppliedImpulse(pe::Vector3::zeros());
                    found = true;
                    break;
                }
            }
            // if no empty slot found, replace point with the minimum (abs) depth
            if (!found) {
                auto max_dist = PE_REAL_MIN;
                int idx = -1;
                for (int i = 0; i < PE_CONTACT_CACHE_SIZE; i++) {
                    if (_points[i].getDistance() > max_dist) {
                        max_dist = _points[i].getDistance();
                        idx = i;
                    }
                }
                if (idx >= 0) {
                    _points[idx] = ContactPoint(point, n, local_pos_a, local_pos_b, depth);
                    _points[idx].setAppliedImpulse(pe::Vector3::zeros());
                }
            }
        }
    }

    void ContactResult::sortContactPoints() {
        std::sort(_points, _points + PE_CONTACT_CACHE_SIZE,
                  [](const ContactPoint& a, const ContactPoint& b) {
            return a.getDistance() < b.getDistance();
        });
        for (int i = 0; i < PE_CONTACT_CACHE_SIZE; i++) {
            if (!_points[i].isValid()) {
                _point_size = i;
                return;
            }
        }
        _point_size = PE_CONTACT_CACHE_SIZE;
    }

    void ContactResult::clearContactPoints() {
        for (int i = 0; i < PE_CONTACT_CACHE_SIZE; i++) {
            _points[i].invalidate();
        }
        _point_size = 0;
        _swap_flag = false;
    }

    int ContactResult::getExistingClosestPoint(const pe::Vector3 &local_pos_b) const {
        pe::Real min_dist = getSameContactPointDistanceThreshold();
        min_dist *= min_dist;

        int nearest = -1;
        for (int i = 0; i < PE_CONTACT_CACHE_SIZE; i++) {
            if (!_points[i].isValid()) continue;
            pe::Vector3 diff = _points[i].getLocalPosB() - local_pos_b;
            const pe::Real dist = diff.dot(diff);
            if (dist < min_dist) {
                min_dist = dist;
                nearest = i;
            }
        }
        return nearest;
    }

    pe::Real ContactResult::getSameContactPointDistanceThreshold() const {
        const pe::Real a_scale = _object_a->getAABBScale();
        const pe::Real b_scale = _object_b->getAABBScale();
        return PE_MIN(a_scale, b_scale) * PE_DIST_TH;
    }

} // pe_phys_collision
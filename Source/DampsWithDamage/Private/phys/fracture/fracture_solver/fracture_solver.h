#pragma once

#include "phys/object/rigidbody.h"
#include "phys/object/fracturable_object.h"

namespace pe_phys_fracture {

    enum FractureType {
        Sphere, Cylinder
    };

    struct FractureSource {
        FractureType type;
        pe::Vector3 position = pe::Vector3::zeros();
        pe::Vector3 intensity = pe::Vector3::ones();
    };

    class FractureSolver {
        COMMON_MEMBER_PTR_SET_GET(pe_phys_object::FracturableObject, fracturable_object, FracturableObject);

    protected:
        pe::Array<pe_phys_object::RigidBody*> _result;

        static pe::Vector3 randomRangeSpherePoints(pe::Real radiusMin, pe::Real radiusMax);
        static pe::Vector3 randomRangeCylinderPoints(pe::Real radius, pe::Real heightMin, pe::Real heightMax);
        static pe_phys_object::RigidBody* addMesh(pe::Mesh& mesh, bool inside, const pe::Transform& trans, const std::string& obj_path);

        bool generatePoints(const pe::Array<FractureSource>& sources,
                            pe::Array<pe::KV<pe::Vector3, bool>>& points, pe::Array<pe::Vector3>& forces);

    public:
        FractureSolver() {}
        virtual ~FractureSolver() {}

        virtual void solve(const pe::Array<FractureSource>& sources) = 0;
        pe::Array<pe_phys_object::RigidBody*>& getFragments() { return _result; }
        virtual void reset() { _result.clear(); }
    };

} // namespace pe_phys_fracture
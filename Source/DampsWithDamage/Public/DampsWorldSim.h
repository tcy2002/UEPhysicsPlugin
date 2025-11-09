#pragma once

#include "CoreMinimal.h"
#include "interface/world.h"
#include "vehicle/tracked_vehicle/tracked_vehicle.h"
#include "physics/fracture/fracture_solver/simple_fracture_solver.h"

DECLARE_DELEGATE_TwoParams(FOnGetBreakableObjects, pe::Array<int>&, pe::Array<bool>&);
DECLARE_DELEGATE_EightParams(FOnAddConvexMeshObjectCallback, bool, int, const pe::Mesh&, const pe::Transform&, const pe::Real&, const pe::Real&, const pe::Real&, bool);
DECLARE_DELEGATE_OneParam(FOnRemoveObjectCallback, int);

/// <summary>
/// Simulator for Damps physics world.
/// Manage rigid bodies / fracturable objects /
/// simulation steps / gabage collection etc.
/// </summary>
class DampsWorldSim
{
public:
	DampsWorldSim();
	virtual ~DampsWorldSim() = default;

	void Update(pe::Real dt);
    void Reset();

	int AddConvexMeshObject(const pe::Mesh& mesh, const pe::Transform& trans, const pe::Real& mass, const pe::Real& fricCoeff, const pe::Real& restCoeff, bool fixedDof[6], bool isKinematic = false);
	int AddBoxObject(const pe::Vector3& extents, const pe::Transform& trans, const pe::Real& mass, const pe::Real& fricCoeff, const pe::Real& restCoeff, bool fixedDof[6], bool isKinematic = false);
	int AddSphereObject(const pe::Real& radius, const pe::Transform& trans, const pe::Real& mass, const pe::Real& fricCoeff, const pe::Real& restCoeff, bool fixedDof[6], bool isKinematic = false);
    int AddCapsuleObject(const pe::Real& radius, const pe::Real& height, const pe::Transform& trans, const pe::Real& mass, const pe::Real& fricCoeff, const pe::Real& restCoeff, bool fixedDof[6], bool isKinematic = false);

    bool RemoveObject(int id);
	void ClearAllObjects();

    int AddTank(pe_vehicle::TrackedVehicle* tank);
    void RemoveTank(int id);
    void ClearAllTanks();

    void SetObjectTransform(int id, const pe::Transform& transform);
    pe::Transform GetObjectTransform(int id) const;

    void SetObjectKinematic(int id, bool isKinematic);

    void AddFractureSource(const std::string& type, const pe::Vector3& position, const pe::Vector3& intensity);

    FOnGetBreakableObjects OnGetBreakableObjects;
    FOnAddConvexMeshObjectCallback OnAddConvexMeshObject;
    FOnRemoveObjectCallback OnRemoveObject;

private:
    pe_interface::World mWorld; // the physics world
    pe_physics_fracture::SimpleFractureSolver mFractureSolver; // fracture solver

    pe::Map<int, pe_physics_object::RigidBody*> mRigidBodies; // map from global ID to rigid body pointer
    pe::Map<int, pe::Vector3> mRelocVecs; // relocation vectors for each object
    
    pe::Array<pe_physics_fracture::FractureSource> mFractureSources; // fracture sources

    pe::Array<pe_vehicle::TrackedVehicle*> mTanks;
};
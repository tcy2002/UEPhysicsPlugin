#pragma once

#include "phys/phys_general.h"
#include "intf/world.h"
#include "phys/fracture/fracture_solver/simple_fracture_solver.h"

DECLARE_DELEGATE_OneParam(FOnGetBreakableObjects, pe::Array<int>&);
DECLARE_DELEGATE_SevenParams(FOnAddConvexMeshObjectCallback, int, const pe::Mesh&, const pe::Transform&, const pe::Real&, const pe::Real&, const pe::Real&, bool);
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

	void update(pe::Real dt);
    void reset();

	int AddConvexMeshObject(const pe::Mesh& mesh, const pe::Transform& trans, const pe::Real& mass, const pe::Real& fricCoeff, const pe::Real& restCoeff, bool isKinematic = false);
	int AddBoxObject(const pe::Vector3& extents, const pe::Transform& trans, const pe::Real& mass, const pe::Real& fricCoeff, const pe::Real& restCoeff, bool isKinematic = false);
	int AddSphereObject(const pe::Real& radius, const pe::Transform& trans, const pe::Real& mass, const pe::Real& fricCoeff, const pe::Real& restCoeff, bool isKinematic = false);

    bool RemoveObject(int id);
	void clearAllObjects();

    void SetObjectTransform(int id, const pe::Transform& transform);
    pe::Transform GetObjectTransform(int id) const;

    void SetObjectKinematic(int id, bool isKinematic);

    void AddFractureSource(const std::string& type, const pe::Vector3& position, const pe::Vector3& intensity);

    FOnGetBreakableObjects OnGetBreakableObjects;
    FOnAddConvexMeshObjectCallback OnAddConvexMeshObject;
    FOnRemoveObjectCallback OnRemoveObject;

private:
    pe_intf::World mWorld; // the physics world
    pe_phys_fracture::SimpleFractureSolver mFractureSolver; // fracture solver

    pe::Map<int, pe_phys_object::RigidBody*> mRigidBodies; // map from global ID to rigid body pointer
    pe::Map<int, pe::Vector3> mRelocVecs; // relocation vectors for each object
    
    pe::Array<pe_phys_fracture::FractureSource> mFractureSources; // fracture sources
};
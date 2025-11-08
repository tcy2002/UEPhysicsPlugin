#include "DampsWorldSim.h"
#include "physics/shape/convex_mesh_shape.h"
#include "physics/shape/box_shape.h"
#include "physics/shape/sphere_shape.h"

DampsWorldSim::DampsWorldSim() 
{
    mWorld.setGravity(pe::Vector3(0, pe::Real(-9.8), 0));
    mWorld.setSleepLinVel2Threshold(pe::Real(0.01));
    mWorld.setSleepAngVel2Threshold(pe::Real(0.005));
    mWorld.setSleepTimeThreshold(pe::Real(2.0));
}

void DampsWorldSim::update(pe::Real dt)
{
    // First, check whether there are fracture sources to process
    if (!mFractureSources.empty()) {
        pe::Array<int> breakableIds;
        pe::Array<bool> flags;
        if (OnGetBreakableObjects.IsBound()) {
            OnGetBreakableObjects.Execute(breakableIds, flags);
        }
        
        for (int i = 0; i < PE_I(breakableIds.size()); i++) {
            auto b = breakableIds[i];
            auto f = flags[i];
            UE_LOG(LogTemp, Warning, TEXT("SolveFracturableBody: %d"), b);
            pe_physics_object::RigidBody* pRigidBody = mRigidBodies[b];
            static pe_physics_object::FracturableObject fracturableObj;
            fracturableObj.setCollisionShape(pRigidBody->getCollisionShape());
            fracturableObj.setThreshold(pe::Real(1.0));
            fracturableObj.setTransform(pRigidBody->getTransform());

            mFractureSolver.reset();
            mFractureSolver.setFracturableObject(&fracturableObj);
            mFractureSolver.solve(mFractureSources);

            if (mFractureSolver.getFragments().size() > 1) {
                UE_LOG(LogTemp, Warning, TEXT("FragmentsGenerated: %ld"), mFractureSolver.getFragments().size());
                for (auto* frag : mFractureSolver.getFragments()) {
                    mWorld.addRigidBody(frag);
                    UE_LOG(LogTemp, Warning, TEXT("FragmentsGenerated: %ld"), frag->getGlobalId());
                    
                    // if the parent is kinematic, we can also have kinematic fragments
                    if (pRigidBody->isKinematic()) {
                        // do nothing, because kinematic was set in the solver
                    }
                    // if not, all fragments are dynamic
                    else {
                        frag->setKinematic(false);
                    }

                    mRigidBodies[frag->getGlobalId()] = frag;
                    mRelocVecs[frag->getGlobalId()] = pe::Vector3::zeros();

                    // Notify the addition of a new convex mesh object
                    if (OnAddConvexMeshObject.IsBound()) {
                        pe_physics_shape::ConvexMeshShape* pShape = static_cast<pe_physics_shape::ConvexMeshShape*>(frag->getCollisionShape());
                        const pe::Mesh& mesh = pShape->getMesh();
                        OnAddConvexMeshObject.Execute(f, (int)frag->getGlobalId(), mesh, frag->getTransform(), frag->getMass(), frag->getFrictionCoeff(), frag->getRestitutionCoeff(), frag->isKinematic());
                    }
                }

                // Remove the original rigid body
                if (OnRemoveObject.IsBound()) {
                    mWorld.removeRigidBody(pRigidBody);
                    uint32 id = pRigidBody->getGlobalId();
                    delete pRigidBody->getCollisionShape();
                    delete pRigidBody;
                    mRigidBodies.erase(b);
                    mRelocVecs.erase(b);

                    // Notify the removal
                    OnRemoveObject.Execute(id);
                }
            }
        }

        // No matter whether fracture happened, clear the fracture sources for the next step
        mFractureSources.clear();
    }

	// Update the rigid world with the given time step
	mWorld.setDt(dt);
	mWorld.step();
}

void DampsWorldSim::reset()
{
    mWorld.clearRigidBodies();
    clearAllObjects();
    mFractureSources.clear();
}

int DampsWorldSim::AddConvexMeshObject(const pe::Mesh& mesh, const pe::Transform& trans, const pe::Real& mass, const pe::Real& fricCoeff, const pe::Real& restCoeff, bool isKinematic)
{
    auto shape = new pe_physics_shape::ConvexMeshShape();
	pe::Vector3 reloc = shape->setMesh(mesh);
    pe_physics_object::RigidBody* pRigidBody = new pe_physics_object::RigidBody();
    pRigidBody->setCollisionShape(shape);
    pRigidBody->setTransform(pe::Transform(trans.getBasis(), trans.getOrigin() + reloc));
    pRigidBody->setMass(mass);
    pRigidBody->setFrictionCoeff(fricCoeff);
    pRigidBody->setRestitutionCoeff(restCoeff);
    pRigidBody->setKinematic(isKinematic);
    mRigidBodies[pRigidBody->getGlobalId()] = pRigidBody;
    mRelocVecs[pRigidBody->getGlobalId()] = reloc;
    mWorld.addRigidBody(pRigidBody);
    return pRigidBody->getGlobalId();
}

int DampsWorldSim::AddBoxObject(const pe::Vector3& extents, const pe::Transform& trans, const pe::Real& mass, const pe::Real& fricCoeff, const pe::Real& restCoeff, bool isKinematic)
{
    auto shape = new pe_physics_shape::BoxShape(extents);
    pe::Vector3 reloc = pe::Vector3::zeros();
    pe_physics_object::RigidBody* pRigidBody = new pe_physics_object::RigidBody();
    pRigidBody->setCollisionShape(shape);
    pRigidBody->setTransform(trans);
    pRigidBody->setMass(mass);
    pRigidBody->setFrictionCoeff(fricCoeff);
    pRigidBody->setRestitutionCoeff(restCoeff);
    pRigidBody->setKinematic(isKinematic);
    mRigidBodies[pRigidBody->getGlobalId()] = pRigidBody;
    mRelocVecs[pRigidBody->getGlobalId()] = reloc;
    mWorld.addRigidBody(pRigidBody);
    return pRigidBody->getGlobalId();
}

int DampsWorldSim::AddSphereObject(const pe::Real& radius, const pe::Transform& trans, const pe::Real& mass, const pe::Real& fricCoeff, const pe::Real& restCoeff, bool isKinematic)
{
    auto shape = new pe_physics_shape::SphereShape(radius);
    pe::Vector3 reloc = pe::Vector3::zeros();
    pe_physics_object::RigidBody* pRigidBody = new pe_physics_object::RigidBody();
    pRigidBody->setCollisionShape(shape);
    pRigidBody->setTransform(trans);
    pRigidBody->setMass(mass);
    pRigidBody->setFrictionCoeff(fricCoeff);
    pRigidBody->setRestitutionCoeff(restCoeff);
    pRigidBody->setKinematic(isKinematic);
    mRigidBodies[pRigidBody->getGlobalId()] = pRigidBody;
    mRelocVecs[pRigidBody->getGlobalId()] = reloc;
    mWorld.addRigidBody(pRigidBody);
    return pRigidBody->getGlobalId();
}

bool DampsWorldSim::RemoveObject(int id) {
    if (mRigidBodies.find(id) == mRigidBodies.end()) {
        return false;
    }

    mWorld.removeRigidBody(mRigidBodies[id]);
    delete mRigidBodies[id]->getCollisionShape();
    delete mRigidBodies[id];
    mRigidBodies.erase(id);
    mRelocVecs.erase(id);
    return true;
}

void DampsWorldSim::clearAllObjects() {
    mWorld.clearRigidBodies();
    for (auto& kv : mRigidBodies) {
        delete kv.second->getCollisionShape();
        delete kv.second;
    }
    mRigidBodies.clear();
    mRelocVecs.clear();
}

void DampsWorldSim::SetObjectTransform(int id, const pe::Transform& transform) {
    if (mRigidBodies.find(id) == mRigidBodies.end()) {
        return;
    }

    pe::Transform relocated = transform * pe::Transform(pe::Matrix3::identity(), mRelocVecs[id]);
    mRigidBodies[id]->setTransform(relocated);
}

pe::Transform DampsWorldSim::GetObjectTransform(int id) const {
    if (mRigidBodies.find(id) == mRigidBodies.end()) {
        return pe::Transform::identity();
    }
    pe::Transform transformed = mRigidBodies.at(id)->getTransform() * pe::Transform(pe::Matrix3::identity(), -mRelocVecs.at(id));
    const auto& trans = mRigidBodies.at(id)->getTransform();
    return transformed;
}

void DampsWorldSim::SetObjectKinematic(int id, bool isKinematic) {
    if (mRigidBodies.find(id) == mRigidBodies.end()) {
        return;
    }
    mRigidBodies[id]->setKinematic(isKinematic);
}

void DampsWorldSim::AddFractureSource(const std::string& type, const pe::Vector3& position, const pe::Vector3& intensity) {
    UE_LOG(LogTemp, Warning, TEXT("AddFractureSource: %s, %f %f %f | %f %f %f"), *FString(type.c_str()), position.x, position.y, position.z, intensity.x, intensity.y, intensity.z);
    pe_physics_fracture::FractureSource source;
    std::string type_lower = type;
    for (auto& c : type_lower) {
        c = std::tolower(c);
    }
    if (type_lower == "sphere") {
        source.type = pe_physics_fracture::FractureType::Sphere;
    }
    else if (type_lower == "cylinder") {
        source.type = pe_physics_fracture::FractureType::Cylinder;
    }
    else {
        UE_LOG(LogTemp, Error, TEXT("DampsWorldSim::AddFractureSource: Unknown fracture source type '%s', defaulting to 'sphere'."), *FString(type.c_str()));
        source.type = pe_physics_fracture::FractureType::Sphere;
    }
    source.position = position;
    source.intensity = intensity;
    mFractureSources.push_back(source);
}

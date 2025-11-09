#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "DampsWorldSim.h"
#include "DampsUEInstance.generated.h"

namespace pe_vehicle {
	class TrackedVehicle;
}

UCLASS()
class UDampsUEInstance : public UGameInstanceSubsystem, public FTickableGameObject
{
	GENERATED_BODY()

public:
	UDampsUEInstance();
	virtual ~UDampsUEInstance() {}

	////////////////////////////
	/// from UWorldSubsystem
	////////////////////////////
	virtual bool ShouldCreateSubsystem(UObject* Outer) const override { return true; }
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

	////////////////////////////
	/// from FTickableGameObject
	////////////////////////////
	virtual bool IsTickable() const override { return !IsTemplate(); }
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override { RETURN_QUICK_DECLARE_CYCLE_STAT(UDampsUEInstance, STATGROUP_Tickables); }
	
	void RegisterConvexMeshObject(AActor* pActor, const FTransform& transform, const TArray<FVector>& vertices, const TArray<int32>& indices, 
		pe::Real mass, pe::Real friction, pe::Real restitution, bool fixedDof[6], bool isStatic = false);
	void RegisterCubeObject(AActor* pActor, const FTransform& transform, const FVector& halfExtents, 
		pe::Real mass, pe::Real friction, pe::Real restitution, bool fixedDof[6], bool isStatic = false);
    void RegisterSphereObject(AActor* pActor, const FTransform& transform, float radius, 
		pe::Real mass, pe::Real friction, pe::Real restitution, bool fixedDof[6], bool isStatic = false);
    void RegisterCapsuleObject(AActor* pActor, const FTransform& transform, float radius, float height, 
		pe::Real mass, pe::Real friction, pe::Real restitution, bool fixedDof[6], bool isStatic = false);
	void UnregisterObject(AActor* pActor);

	void RegisterTank(pe_vehicle::TrackedVehicle* tank);
	void UnregisterTank(pe_vehicle::TrackedVehicle* tank);

    void MarkObjectAsBreakable(AActor* pActor, float threshold);
	void MarkObjectAsUnbreakable(AActor* pActor);

    UFUNCTION(BlueprintCallable, Category = "DampsUEInstance", meta = (DisplayName = "添加毁伤源"))
    void AddFractureSource(const FString& type, const FVector& position, const FVector& intensity);

private:
	TSharedPtr<DampsWorldSim> m_pDampsWorldSim = nullptr;
	std::map<AActor*, int> m_pActorToWorld;
	std::map<AActor*, float> m_BreakableActors;
    std::map<pe_vehicle::TrackedVehicle*, int> m_pTankToWorld;
    UMaterial* m_pDefaultMaterial = nullptr;

	void AddNewObject(bool flag, int id, const pe::Mesh& mesh, const pe::Transform& trans, const pe::Real& mass, const pe::Real& fric, const pe::Real& rest, bool isKinematic);
	void RemoveObject(int id);
};

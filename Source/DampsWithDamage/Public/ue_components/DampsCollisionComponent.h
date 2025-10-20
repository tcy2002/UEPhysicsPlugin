#pragma once

#include "CoreMinimal.h"
#include "ProceduralMeshComponent.h"
#include "DampsCollisionComponent.generated.h"

class UDampsUEInstance;

UCLASS()
class UDampsCollisionComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UDampsCollisionComponent();
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float Mass = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object", meta = (ClampMin = "0.0", UIMin = "0.0"))
    float Friction = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object", meta = (ClampMin = "0.0", UIMin = "0.0"))
    float Restitution = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object")
    bool bShowCollisionMesh = true;

private:
	void RegisterObjectToDamps_Internal();
    void UnregisterObjectFromDamps_Internal();
	void ShowCollisionMesh_Internal();
	void InitCollisionInfo_Internal();

protected:
	virtual void RegisterObjectToDamps(AActor* Owner, UDampsUEInstance* DampsInstance) {}
    virtual void UnregisterObjectFromDamps(AActor* Owner, UDampsUEInstance* DampsInstance) {}
    virtual void ShowCollisionMesh(AActor* Owner) {}
    virtual void InitCollisionInfo(UStaticMeshComponent* smc) {}
	virtual void InitCollisionInfo(UProceduralMeshComponent* pmc) {}

	UPROPERTY()
	FTransform CollisionTransform = FTransform::Identity;

	bool bShouldRegister = true;

	friend class UDampsUEInstance;
};

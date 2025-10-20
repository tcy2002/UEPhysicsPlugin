#pragma once

#include "CoreMinimal.h"
#include "DampsCollisionComponent.h"
#include "DampsSphereCollisionComponent.generated.h"

UCLASS(ClassGroup=(Custom), meta = (BlueprintSpawnableComponent))
class UDampsSphereCollisionComponent : public UDampsCollisionComponent
{
	GENERATED_BODY()

public:
	UDampsSphereCollisionComponent() = default;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float Radius = 0.0f;

private:
	virtual void RegisterObjectToDamps(AActor* Owner, UDampsUEInstance* DampsInstance) override;
	virtual void UnregisterObjectFromDamps(AActor* Owner, UDampsUEInstance* DampsInstance) override;
	virtual void ShowCollisionMesh(AActor* Owner) override;
	virtual void InitCollisionInfo(UStaticMeshComponent* smc) override;
	virtual void InitCollisionInfo(UProceduralMeshComponent* pmc) override;
};

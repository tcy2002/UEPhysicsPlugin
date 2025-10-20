#pragma once

#include "CoreMinimal.h"
#include "DampsCollisionComponent.h"
#include "DampsCubeCollisionComponent.generated.h"

UCLASS(ClassGroup=(Custom), meta = (BlueprintSpawnableComponent))
class UDampsCubeCollisionComponent : public UDampsCollisionComponent
{
	GENERATED_BODY()

public:
	UDampsCubeCollisionComponent() = default;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object")
	FVector HalfExtents = FVector::ZeroVector;

private:
	virtual void RegisterObjectToDamps(AActor* Owner, UDampsUEInstance* DampsInstance) override;
	virtual void UnregisterObjectFromDamps(AActor* Owner, UDampsUEInstance* DampsInstance) override;
	virtual void ShowCollisionMesh(AActor* Owner) override;
	virtual void InitCollisionInfo(UStaticMeshComponent* smc) override;
	virtual void InitCollisionInfo(UProceduralMeshComponent* pmc) override;
};

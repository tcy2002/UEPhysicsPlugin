#pragma once

#include "CoreMinimal.h"
#include "DampsCollisionComponent.h"
#include "DampsConvexMeshCollisionComponent.generated.h"

UCLASS(ClassGroup=(Custom), meta = (BlueprintSpawnableComponent))
class UDampsConvexMeshCollisionComponent : public UDampsCollisionComponent
{
	GENERATED_BODY()

public:
	UDampsConvexMeshCollisionComponent() = default;
	
	UPROPERTY()
	TArray<FVector> vertices;

	UPROPERTY()
	TArray<int32> indices;

private:
	virtual void RegisterObjectToDamps(AActor* Owner, UDampsUEInstance* DampsInstance) override;
	virtual void UnregisterObjectFromDamps(AActor* Owner, UDampsUEInstance* DampsInstance) override;
	virtual void ShowCollisionMesh(AActor* Owner) override;
	virtual void InitCollisionInfo(UStaticMeshComponent* smc) override;
	virtual void InitCollisionInfo(UProceduralMeshComponent* pmc) override;
};

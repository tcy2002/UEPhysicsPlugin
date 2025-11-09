#pragma once

#include "CoreMinimal.h"
#include "DampsCollisionComponent.h"
#include "DampsCapsuleCollisionComponent.generated.h"

UCLASS(ClassGroup=(Custom), meta = (BlueprintSpawnableComponent))
class UDampsCapsuleCollisionComponent : public UDampsCollisionComponent
{
	GENERATED_BODY()

public:
	UDampsCapsuleCollisionComponent() = default;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object", meta = (ClampMin = "0.0", UIMin = "0.0", DisplayName = "半径"))
	float Radius = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object", meta = (ClampMin = "0.0", UIMin = "0.0", DisplayName = "高度（不包含半球部分）"))
	float Height = 0.0f;

private:
	virtual void RegisterObjectToDamps(AActor* Owner, UDampsUEInstance* DampsInstance) override;
	virtual void UnregisterObjectFromDamps(AActor* Owner, UDampsUEInstance* DampsInstance) override;
	virtual void ShowCollisionMesh(AActor* Owner) override;
	virtual void InitCollisionInfo(UStaticMeshComponent* smc) override;
	virtual void InitCollisionInfo(UProceduralMeshComponent* pmc) override;
};

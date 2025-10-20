#pragma once

#include "CoreMinimal.h"
#include "DampsConvexMeshCollisionComponent.generated.h"

class UDampsUEInstance;

UCLASS(ClassGroup=(Custom), meta = (BlueprintSpawnableComponent))
class UDampsConvexMeshCollisionComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UDampsConvexMeshCollisionComponent();
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float Mass = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object", meta = (ClampMin = "0.0", UIMin = "0.0"))
    float Friction = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object", meta = (ClampMin = "0.0", UIMin = "0.0"))
    float Restitution = 0.5f;

	UPROPERTY()
	TArray<FVector> vertices;

	UPROPERTY()
	TArray<int32> indices;

private:
	void RegisterObjectToDamps();
    void UnregisterObjectFromDamps();
	void ShowCollisionMesh();
	void InitCollisionInfo();

    bool bShouldRegister = true;

    friend class UDampsUEInstance;
};

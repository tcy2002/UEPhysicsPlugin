#pragma once

#include "CoreMinimal.h"
#include "DampsCubeCollisionComponent.generated.h"

UCLASS(ClassGroup=(Custom), meta = (BlueprintSpawnableComponent))
class UDampsCubeCollisionComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UDampsCubeCollisionComponent();
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object")
	FVector HalfExtents = FVector::ZeroVector;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float Mass = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object", meta = (ClampMin = "0.0", UIMin = "0.0"))
    float Friction = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object", meta = (ClampMin = "0.0", UIMin = "0.0"))
    float Restitution = 0.5f;

private:
	void RegisterObjectToDamps();
    void UnregisterObjectFromDamps();
	void ShowCollisionMesh();
	void InitCollisionInfo();

	UPROPERTY()
	FTransform CollisionTransform = FTransform::Identity;
};

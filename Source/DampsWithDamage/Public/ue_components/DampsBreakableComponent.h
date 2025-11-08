#pragma once

#include "CoreMinimal.h"
#include "DampsBreakableComponent.generated.h"

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class UDampsBreakableComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object", meta = (ClampMin = "0.0", UIMin = "0.0"))
    float BreakThreshold = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object")
    bool bShowCollisionMeshAfterBreak = true;

public:
	UDampsBreakableComponent();
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	void RegisterBreakableObjectToDamps();
	void UnregisterBreakableObjectFromDamps();
};

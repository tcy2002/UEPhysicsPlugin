#pragma once

#include "CoreMinimal.h"
#include "ProceduralMeshComponent.h"
#include "DampsCollisionComponent.generated.h"

class UDampsUEInstance;
class ADampsTankActor;

USTRUCT(BlueprintType)
struct FFixedDof {
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "位置：x"))
    bool bFixedXPosition = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "位置：y"))
    bool bFixedYPosition = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "位置：z"))
    bool bFixedZPosition = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "旋转：x"))
    bool bFixedXRotation = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "旋转：y"))
    bool bFixedYRotation = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "旋转：z"))
    bool bFixedZRotation = false;
};

UCLASS()
class UDampsCollisionComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UDampsCollisionComponent();
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object", meta = (ClampMin = "0.0", UIMin = "0.0", DisplayName = "质量"))
	float Mass = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object", meta = (ClampMin = "0.0", UIMin = "0.0", DisplayName = "摩擦系数"))
    float Friction = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object", meta = (ClampMin = "0.0", UIMin = "0.0", DisplayName = "恢复系数"))
    float Restitution = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object", meta = (DisplayName = "锁定自由度"))
    FFixedDof FixedDof;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damps Object", meta = (DisplayName = "显示碰撞体"))
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
	friend class ADampsTankActor;
};

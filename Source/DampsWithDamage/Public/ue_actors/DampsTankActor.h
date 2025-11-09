#pragma once

#include "CoreMinimal.h"
#include "DampsTankActor.generated.h"

UENUM(BlueprintType)
enum class ESuspensionType : uint8
{
    Front       UMETA(DisplayName = "可转向型"),
    Rear        UMETA(DisplayName = "不可转向型"),
    None        UMETA(DisplayName = "无")
};

USTRUCT(BlueprintType)
struct FWheelInfo
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "车轮索引"))
    int index;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "悬挂类型"))
    ESuspensionType type = ESuspensionType::None;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "是否驱动轮"))
    bool motor;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "半径"))
    float radius;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "质量"))
    float mass;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "摩擦系数"))
    float friction;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "竖直方向偏移"))
    float anchor_offset_y;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "前后方向偏移"))
    float anchor_offset_z;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "悬挂原长"))
    float suspension_rest_length;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "悬挂刚度系数"))
    float suspension_stiffness;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "悬挂阻尼系数"))
    float suspension_damping;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "模型"))
    UStaticMesh* WheelMesh;
};

namespace pe_vehicle {
    class TrackedVehicle;
    class Chassis;
    class Engine;
}

UCLASS(Blueprintable, BlueprintType)
class  ADampsTankActor : public AActor
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "生成姿态和位置"))
    FTransform InitialTransform = FTransform::Identity;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "车轮区域长度"))
    float WheelRegionLength = 200.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "车轮区域宽度"))
    float WheelRegionWidth = 200.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "车轮区域偏移"))
    FVector WheelRegionOffset = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "车轮宽度"))
    float WheelWidth = 120.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "单侧车轮数目"))
    int WheelCountPerSide = 2;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "档位"))
    int Gear = 0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "左侧履带油门"))
    float LeftThrottle = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "右侧履带油门"))
    float RightThrottle = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "刹车"))
    float Brake = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "车轮参数"))
    TArray<FWheelInfo> WheelInfoArray;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "底盘尺寸"))
    FVector ChassisSize = FVector(200.0f, 100.0f, 50.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "底盘质量"))
    float ChassisMass = 1000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "底盘模型"))
    UStaticMesh* ChassisMesh = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "显示车身碰撞体"))
    bool bShowCollider = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "履带片段尺寸"))
    FVector TrackSegmentSize = FVector(50.0f, 20.0f, 10.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (DisplayName = "履带片段模型"))
    UStaticMesh* TrackSegmentMesh = nullptr;

    ADampsTankActor();
    virtual ~ADampsTankActor() override;

protected:
    pe_vehicle::Chassis* mChassis = nullptr;
    pe_vehicle::Engine* mEngine = nullptr;
    pe_vehicle::TrackedVehicle* mTank = nullptr;
    TArray<AActor*> mWheelActors;
    TArray<AActor*> mLeftTrackSegmentActors;
    TArray<AActor*> mRightTrackSegmentActors;
    AActor* mChassisActor = nullptr;
    AActor* mTurretActor = nullptr;
    AActor* mBarrelActor = nullptr;

    UStaticMesh* mDefaultCylinder = nullptr;
    UStaticMesh* mDefaultCube = nullptr;
    UMaterial* mDefaultMaterial = nullptr;

    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;
};

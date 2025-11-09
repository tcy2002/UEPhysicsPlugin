#include "ue_actors/DampsTankActor.h"
#include "vehicle/tracked_vehicle/tracked_vehicle.h"
#include "CoordConvert.h"
#include "DampsUEInstance.h"
#include "Thirdparty/json/json.hpp"
#include "Engine/StaticMeshActor.h"
#include "ue_components/DampsCubeCollisionComponent.h"
#include <fstream>

ADampsTankActor::ADampsTankActor()
{
    PrimaryActorTick.bCanEverTick = true;

    mChassis = new pe_vehicle::Chassis();
    mEngine = new pe_vehicle::Engine(6);
    mTank = new pe_vehicle::TrackedVehicle();

    static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderMesh(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
    if (CylinderMesh.Succeeded()) {
        mDefaultCylinder = CylinderMesh.Object;
    }
    static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
    if (CubeMesh.Succeeded()) {
        mDefaultCube = CubeMesh.Object;
    }
    static ConstructorHelpers::FObjectFinder<UMaterial> MaterialFinder(TEXT("/Engine/BasicShapes/BasicShapeMaterial"));
    if (MaterialFinder.Succeeded()) {
        mDefaultMaterial = (UMaterial*)MaterialFinder.Object;
    }
}

ADampsTankActor::~ADampsTankActor()
{
    if (UWorld* World = GetWorld())
    {
        if (UGameInstance* GameInstance = World->GetGameInstance())
        {
            if (UDampsUEInstance* DampsInstance = GameInstance->GetSubsystem<UDampsUEInstance>())
            {
                DampsInstance->UnregisterTank(mTank);
            }
        }
    }

    delete mTank;
    mTank = nullptr;
}

void ADampsTankActor::BeginPlay()
{
    Super::BeginPlay();

    // create tank instance and register into physics world
    //// chassis
    mChassis->setBoxBase(pe::Transform::identity(), CoordConvert::FVectorToDampsAbsVector3(ChassisSize, true), ChassisMass);

    //// engine
    std::fstream file("D:/ClionProjects/Physics-Engine-clean/Physics-Engine-84da628/examples/vehicle_demo/config/Engine.json");
    nlohmann::json j = nlohmann::json::parse(file);
    mEngine->loadConfigFromJson(j);

    //// tank
    mTank->setChassis(mChassis);
    mTank->setEngine(mEngine);
    mTank->setWheelRegionLength(PE_R(WheelRegionLength / 100));
    mTank->setWheelRegionWidth(PE_R(WheelRegionWidth / 100));
    mTank->setWheelRegionOffset(CoordConvert::FVectorToDampsVector3(WheelRegionOffset, true));
    mTank->setWheelWidth(PE_R(WheelWidth / 100));
    mTank->setWheelCountPerSide(WheelCountPerSide);
    mTank->setGear(Gear);
    mTank->setLeftThrottle(PE_R(LeftThrottle));
    mTank->setRightThrottle(PE_R(RightThrottle));
    mTank->setBrake(PE_R(Brake));
    for (const FWheelInfo& info : WheelInfoArray) {
        pe_vehicle::AxleType type = info.type == ESuspensionType::None ? pe_vehicle::AxleType::AT_NONE :
            (info.type == ESuspensionType::Front ? pe_vehicle::AxleType::AT_FRONT : pe_vehicle::AxleType::AT_REAR);
        mTank->setWheelInfo(info.index, type, info.motor,
            PE_R(info.radius / 100), PE_R(info.mass), PE_R(info.friction),
            PE_R(info.anchor_offset_y / 100), PE_R(info.anchor_offset_z / 100),
            PE_R(info.suspension_rest_length / 100), PE_R(info.suspension_stiffness),
            PE_R(info.suspension_damping));
    }
    mTank->setTransform(CoordConvert::FTransformToDampsTransform(InitialTransform, true));
    if (UWorld* World = GetWorld())
    {
        if (UGameInstance* GameInstance = World->GetGameInstance())
        {
            if (UDampsUEInstance* DampsInstance = GameInstance->GetSubsystem<UDampsUEInstance>())
            {
                DampsInstance->RegisterTank(mTank);
            }
        }
    }

    // create static mesh actor for each wheel
    for (int i = 0; i < WheelInfoArray.Num(); i++) {
        AStaticMeshActor* wheelActor = GetWorld()->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(), FTransform::Identity);
        UStaticMeshComponent* wheelMeshComp = NewObject<UStaticMeshComponent>(wheelActor);
        wheelActor->SetMobility(EComponentMobility::Movable);
        wheelMeshComp->RegisterComponent();
        if (WheelInfoArray[i].WheelMesh) {
            // use the wheel mesh to create a new actor
            wheelMeshComp->SetStaticMesh(WheelInfoArray[i].WheelMesh);
        } else {
            // use the default cylinder mesh to create a new actor
            wheelMeshComp->SetStaticMesh(mDefaultCylinder);
        }
        if (mDefaultMaterial) {
            wheelMeshComp->SetMaterial(0, mDefaultMaterial);
        }
        wheelMeshComp->AttachToComponent(wheelActor->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
        mWheelActors.Add(wheelActor);
    }

    // create static mesh actor for each track segment
    for (int i = 0; i < mTank->getTrackSegmentCount() * 2; i++) {
        AStaticMeshActor* trackSegmentActor = GetWorld()->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(), GetActorTransform());
        UStaticMeshComponent* trackMeshComp = NewObject<UStaticMeshComponent>(trackSegmentActor);
        trackSegmentActor->SetMobility(EComponentMobility::Movable);
        trackMeshComp->RegisterComponent();
        if (TrackSegmentMesh) {
            // use the track segment mesh to create a new actor
            trackMeshComp->SetStaticMesh(TrackSegmentMesh);
        } else {
            // use the default cube mesh to create a new actor
            trackMeshComp->SetStaticMesh(mDefaultCube);
            trackMeshComp->SetWorldScale3D(FVector(TrackSegmentSize.X / 100.0f, TrackSegmentSize.Y / 100.0f, TrackSegmentSize.Z / 100.0f));
        }
        if (mDefaultMaterial) {
            trackMeshComp->SetMaterial(0, mDefaultMaterial);
        }
        trackMeshComp->AttachToComponent(trackSegmentActor->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
        if (i < mTank->getTrackSegmentCount())
            mLeftTrackSegmentActors.Add(trackSegmentActor);
        else
            mRightTrackSegmentActors.Add(trackSegmentActor);
    }

    // create static mesh actor for chassis
    AStaticMeshActor* chassisActor = GetWorld()->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(), GetActorTransform());
    UStaticMeshComponent* chassisMeshComp = NewObject<UStaticMeshComponent>(chassisActor);
    chassisActor->SetMobility(EComponentMobility::Movable);
    chassisMeshComp->RegisterComponent();
    if (ChassisMesh) {
        // use the chassis mesh to create a new actor
        chassisMeshComp->SetStaticMesh(ChassisMesh);
    } else {
        // use the default cube mesh to create a new actor
        chassisMeshComp->SetStaticMesh(mDefaultCube);
        chassisMeshComp->SetWorldScale3D(FVector(ChassisSize.X / 100.0f, ChassisSize.Y / 100.0f, ChassisSize.Z / 100.0f));
    }
    if (mDefaultMaterial) {
        chassisMeshComp->SetMaterial(0, mDefaultMaterial);
    }
    chassisMeshComp->AttachToComponent(chassisActor->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
    UDampsCubeCollisionComponent* chassisCollisionComp = NewObject<UDampsCubeCollisionComponent>(chassisActor);
    chassisCollisionComp->bShouldRegister = false;
    chassisCollisionComp->HalfExtents = ChassisSize / 2.0f;
    chassisCollisionComp->bShowCollisionMesh = bShowCollider;
    chassisCollisionComp->Mass = ChassisMass;
    chassisCollisionComp->Friction = 0.5f;
    chassisCollisionComp->Restitution = 0.5f;
    chassisCollisionComp->RegisterComponent();
    mChassisActor = chassisActor;
}

void ADampsTankActor::Tick(float DeltaTime) {
    mTank->setGear(Gear);
    mTank->setLeftThrottle(PE_R(LeftThrottle));
    mTank->setRightThrottle(PE_R(RightThrottle));
    mTank->setBrake(PE_R(Brake));

    // set wheel transforms
    for (int i = 0; i < mWheelActors.Num(); i++) {
        const int wheelIdx = WheelInfoArray[i].index;
        const pe::Transform wheelTrans = mTank->getWheelTransform(wheelIdx);
        FTransform ueTransform;
        ueTransform.SetLocation(CoordConvert::DampsVector3ToFVector(wheelTrans.getOrigin(), true));
        ueTransform.SetRotation(CoordConvert::DampsQuatToFQuat(pe::Quaternion(wheelTrans.getBasis())));
        FTransform deltaTransform = FTransform::Identity;
        deltaTransform.SetRotation(FQuat::MakeFromEuler(FVector(0.0f, 90.0f, 0.0f)));
        mWheelActors[i]->SetActorTransform(deltaTransform * ueTransform);
    }

    // set track segment transforms
    for (int i = 0; i < mTank->getTrackSegmentCount() * 2; i++) {
        const pe::Transform trans = mTank->getTrackSegmentTransform(i < mTank->getTrackSegmentCount(), i % mTank->getTrackSegmentCount());
        FTransform ueTransform;
        ueTransform.SetLocation(CoordConvert::DampsVector3ToFVector(trans.getOrigin(), true));
        ueTransform.SetRotation(CoordConvert::DampsQuatToFQuat(pe::Quaternion(trans.getBasis())));
        if (i < mTank->getTrackSegmentCount())
            mLeftTrackSegmentActors[i]->SetActorTransform(ueTransform);
        else
            mRightTrackSegmentActors[i % mTank->getTrackSegmentCount()]->SetActorTransform(ueTransform);
    }

    // set chassis transform
    const pe::Transform chassisTrans = mTank->getChassis()->getTransform();
    FTransform ueChassisTransform;
    ueChassisTransform.SetLocation(CoordConvert::DampsVector3ToFVector(chassisTrans.getOrigin(), true));
    ueChassisTransform.SetRotation(CoordConvert::DampsQuatToFQuat(pe::Quaternion(chassisTrans.getBasis())));
    mChassisActor->SetActorTransform(ueChassisTransform);
}

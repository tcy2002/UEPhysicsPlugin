#include "ue_components/DampsCapsuleCollisionComponent.h"
#include "DampsUEInstance.h"
#include "DrawDebugHelpers.h"

void UDampsCapsuleCollisionComponent::RegisterObjectToDamps(AActor* Owner, UDampsUEInstance* DampsInstance)
{
    if (Radius > 1e-5f && Height > 1e-5f)
    {
        FTransform transform = CollisionTransform * Owner->GetTransform();
        bool isStatic = Owner->GetRootComponent()->Mobility == EComponentMobility::Static ||
            Owner->GetRootComponent()->Mobility == EComponentMobility::Stationary;
        bool fixedDof[6] = {
            FixedDof.bFixedXPosition,
            FixedDof.bFixedZPosition, // Note: Y and Z rotation are swapped
            FixedDof.bFixedYPosition,
            FixedDof.bFixedXRotation,
            FixedDof.bFixedZRotation, // Note: Y and Z rotation are swapped
            FixedDof.bFixedYRotation
        };
        DampsInstance->RegisterCapsuleObject(Owner, transform, Radius, Height, Mass, Friction, Restitution, fixedDof, isStatic);
    }
}

void UDampsCapsuleCollisionComponent::UnregisterObjectFromDamps(AActor* Owner, UDampsUEInstance* DampsInstance)
{
    DampsInstance->UnregisterObject(Owner);
}

void UDampsCapsuleCollisionComponent::ShowCollisionMesh(AActor* Owner) {
    // This function is implemented to visualize the collision mesh
    FTransform Transform = CollisionTransform * Owner->GetActorTransform();
    FVector Center = Transform.GetTranslation();
    float CRadius = Radius * Transform.GetScale3D().GetMax();
    float CHeight = Height * Transform.GetScale3D().GetMax();
    FQuat Rotation = Transform.GetRotation();
    DrawDebugCapsule(GetWorld(), Center, CHeight * 0.5f + CRadius, CRadius, Rotation, FColor::Green, false, -1.f, 0, 2.f);
}

void UDampsCapsuleCollisionComponent::InitCollisionInfo(UStaticMeshComponent* smc)
{
    if (UStaticMesh* sm = smc->GetStaticMesh()) {
        if (sm->GetNumLODs() > 0) {
            FStaticMeshLODResources& LODResource = sm->GetRenderData()->LODResources[0];
            FPositionVertexBuffer& VertexBuffer = LODResource.VertexBuffers.PositionVertexBuffer;
            const int32 NumVerts = VertexBuffer.GetNumVertices();
            FVector MinExtents(FLT_MAX);
            FVector MaxExtents(-FLT_MAX);
            for (int32 VertIndex = 0; VertIndex < NumVerts; ++VertIndex) {
                FVector VertexPosition = VertexBuffer.VertexPosition(VertIndex);
                MinExtents.X = FMath::Min(MinExtents.X, VertexPosition.X);
                MinExtents.Y = FMath::Min(MinExtents.Y, VertexPosition.Y);
                MinExtents.Z = FMath::Min(MinExtents.Z, VertexPosition.Z);
                MaxExtents.X = FMath::Max(MaxExtents.X, VertexPosition.X);
                MaxExtents.Y = FMath::Max(MaxExtents.Y, VertexPosition.Y);
                MaxExtents.Z = FMath::Max(MaxExtents.Z, VertexPosition.Z);
            }
            Radius = FMath::Max((MaxExtents.X - MinExtents.X) * 0.5f, (MaxExtents.Y - MinExtents.Y) * 0.5f);
            Height = MaxExtents.Z - MinExtents.Z;
            CollisionTransform.SetLocation((MaxExtents + MinExtents) * 0.5f);
        }
    }
}

void UDampsCapsuleCollisionComponent::InitCollisionInfo(UProceduralMeshComponent* pmc)
{
    const TArray<FProcMeshVertex>& ProcVertices = pmc->GetProcMeshSection(0)->ProcVertexBuffer;
    FVector MinExtents(FLT_MAX);
    FVector MaxExtents(-FLT_MAX);
    for (auto& Vertex : ProcVertices) {
        FVector VertexPosition = Vertex.Position;
        MinExtents.X = FMath::Min(MinExtents.X, VertexPosition.X);
        MinExtents.Y = FMath::Min(MinExtents.Y, VertexPosition.Y);
        MinExtents.Z = FMath::Min(MinExtents.Z, VertexPosition.Z);
        MaxExtents.X = FMath::Max(MaxExtents.X, VertexPosition.X);
        MaxExtents.Y = FMath::Max(MaxExtents.Y, VertexPosition.Y);
        MaxExtents.Z = FMath::Max(MaxExtents.Z, VertexPosition.Z);
    }
    Radius = FMath::Max((MaxExtents.X - MinExtents.X) * 0.5f, (MaxExtents.Y - MinExtents.Y) * 0.5f);
    Height = MaxExtents.Z - MinExtents.Z;
    CollisionTransform.SetLocation((MaxExtents + MinExtents) * 0.5f);
}

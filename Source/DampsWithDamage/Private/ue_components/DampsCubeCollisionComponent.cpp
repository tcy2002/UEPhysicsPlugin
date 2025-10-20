#include "ue_components/DampsCubeCollisionComponent.h"
#include "DampsUEInstance.h"
#include "DrawDebugHelpers.h"

void UDampsCubeCollisionComponent::RegisterObjectToDamps(AActor* Owner, UDampsUEInstance* DampsInstance)
{
    if (HalfExtents.X > 1e-5f && HalfExtents.Y > 1e-5f && HalfExtents.Z > 1e-5f)
    {
        FTransform transform = CollisionTransform * Owner->GetTransform();
        bool isStatic = Owner->GetRootComponent()->Mobility == EComponentMobility::Static ||
            Owner->GetRootComponent()->Mobility == EComponentMobility::Stationary;
        DampsInstance->RegisterCubeObject(Owner, transform, HalfExtents, Mass, Friction, Restitution, isStatic);
    }
}

void UDampsCubeCollisionComponent::UnregisterObjectFromDamps(AActor* Owner, UDampsUEInstance* DampsInstance)
{
    DampsInstance->UnregisterObject(Owner);
}

void UDampsCubeCollisionComponent::ShowCollisionMesh(AActor* Owner) {
    // This function is implemented to visualize the collision mesh
    FTransform Transform = CollisionTransform * Owner->GetActorTransform();
    FVector Center = Transform.GetTranslation();
    FVector Extents = HalfExtents * Transform.GetScale3D();
    FQuat Rotation = Transform.GetRotation();
    DrawDebugBox(GetWorld(), Center, Extents, Rotation, FColor::Green, false, -1.f, 0, 2.f);
}

void UDampsCubeCollisionComponent::InitCollisionInfo(UStaticMeshComponent* smc)
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
            HalfExtents = (MaxExtents - MinExtents) * 0.5f;
            CollisionTransform.SetLocation((MaxExtents + MinExtents) * 0.5f);
        }
    }
}

void UDampsCubeCollisionComponent::InitCollisionInfo(UProceduralMeshComponent* pmc)
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
    HalfExtents = (MaxExtents - MinExtents) * 0.5f;
    CollisionTransform.SetLocation((MaxExtents + MinExtents) * 0.5f);
}

#include "ue_components/DampsConvexMeshCollisionComponent.h"
#include "DampsUEInstance.h"
#include "DrawDebugHelpers.h"

void UDampsConvexMeshCollisionComponent::RegisterObjectToDamps(AActor* Owner, UDampsUEInstance* DampsInstance)
{
    if (vertices.Num() > 0 && indices.Num() > 0)
    {
        FTransform transform = Owner->GetTransform();
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
        DampsInstance->RegisterConvexMeshObject(Owner, transform, vertices, indices, Mass, Friction, Restitution, fixedDof, isStatic);
    }
}

void UDampsConvexMeshCollisionComponent::UnregisterObjectFromDamps(AActor* Owner, UDampsUEInstance* DampsInstance)
{
    DampsInstance->UnregisterObject(Owner);
}

void UDampsConvexMeshCollisionComponent::ShowCollisionMesh(AActor* Owner) {
    // This function is implemented to visualize the collision mesh
    FTransform Transform = Owner->GetActorTransform();
    for (int32 i = 0; i < indices.Num(); i += 3) {
        FVector v0 = Transform.TransformPosition(vertices[indices[i]]);
        FVector v1 = Transform.TransformPosition(vertices[indices[i + 1]]);
        FVector v2 = Transform.TransformPosition(vertices[indices[i + 2]]);
        DrawDebugLine(Owner->GetWorld(), v0, v1, FColor::Green, false, -1.0f, 0, 2.0f);
        DrawDebugLine(Owner->GetWorld(), v1, v2, FColor::Green, false, -1.0f, 0, 2.0f);
        DrawDebugLine(Owner->GetWorld(), v2, v0, FColor::Green, false, -1.0f, 0, 2.0f);
    }
}

void UDampsConvexMeshCollisionComponent::InitCollisionInfo(UStaticMeshComponent* smc)
{
    if (UStaticMesh* sm = smc->GetStaticMesh()) {
        if (sm->GetNumLODs() > 0) {
            FStaticMeshLODResources& LODResource = sm->GetRenderData()->LODResources[0];
            FPositionVertexBuffer& VertexBuffer = LODResource.VertexBuffers.PositionVertexBuffer;
            FIndexArrayView IndexArray = LODResource.IndexBuffer.GetArrayView();
            indices.Empty();
            vertices.Empty();
            for (int32 Idx = 0; Idx < IndexArray.Num(); ++Idx) {
                vertices.Add(VertexBuffer.VertexPosition(IndexArray[Idx]));
                indices.Add(vertices.Num() - 1);
            }
        }
    }
}

void UDampsConvexMeshCollisionComponent::InitCollisionInfo(UProceduralMeshComponent* pmc)
{
    vertices.Empty();
    indices.Empty();
    const TArray<FProcMeshVertex>& ProcVertices = pmc->GetProcMeshSection(0)->ProcVertexBuffer;
    const TArray<uint32>& ProcIndices = pmc->GetProcMeshSection(0)->ProcIndexBuffer;
    for (int32 Idx = 0; Idx < ProcVertices.Num(); ++Idx) {
        vertices.Add(ProcVertices[Idx].Position);
        indices.Add(vertices.Num() - 1);
    }
}

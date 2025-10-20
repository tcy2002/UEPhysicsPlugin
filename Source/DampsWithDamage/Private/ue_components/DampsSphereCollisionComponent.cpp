#include "ue_components/DampsSphereCollisionComponent.h"
#include "DampsUEInstance.h"
#include "DrawDebugHelpers.h"

void UDampsSphereCollisionComponent::RegisterObjectToDamps(AActor* Owner, UDampsUEInstance* DampsInstance)
{
    if (Radius > 1e-5f)
    {
        FTransform transform = CollisionTransform * Owner->GetTransform();
        bool isStatic = Owner->GetRootComponent()->Mobility == EComponentMobility::Static ||
            Owner->GetRootComponent()->Mobility == EComponentMobility::Stationary;
        DampsInstance->RegisterSphereObject(Owner, transform, Radius, Mass, Friction, Restitution, isStatic);
    }
}

void UDampsSphereCollisionComponent::UnregisterObjectFromDamps(AActor* Owner, UDampsUEInstance* DampsInstance)
{
    DampsInstance->UnregisterObject(Owner);
}

void UDampsSphereCollisionComponent::ShowCollisionMesh(AActor* Owner) {
    // This function is implemented to visualize the collision mesh
    FTransform Transform = CollisionTransform * Owner->GetActorTransform();
    FVector Center = Transform.GetTranslation();
    float CRadius = Radius * Transform.GetScale3D().GetMax();
    DrawDebugSphere(Owner->GetWorld(), Center, CRadius, 12, FColor::Green, false, -1.0f, 0, 2.0f);
}

void UDampsSphereCollisionComponent::InitCollisionInfo(UStaticMeshComponent* smc)
{
    if (UStaticMesh* sm = smc->GetStaticMesh()) {
        if (sm->GetNumLODs() > 0) {
            FStaticMeshLODResources& LODResource = sm->GetRenderData()->LODResources[0];
            FPositionVertexBuffer& VertexBuffer = LODResource.VertexBuffers.PositionVertexBuffer;
            const int32 NumVerts = VertexBuffer.GetNumVertices();
            if (NumVerts > 0)
            {
                FVector MeshCenter = FVector::ZeroVector;
                for (int32 VertIndex = 0; VertIndex < NumVerts; ++VertIndex)
                {
                    MeshCenter += VertexBuffer.VertexPosition(VertIndex);
                }
                MeshCenter /= NumVerts;
                float MaxDistanceSq = 0.0f;
                for (int32 VertIndex = 0; VertIndex < NumVerts; ++VertIndex)
                {
                    FVector VertexPosition = VertexBuffer.VertexPosition(VertIndex);
                    float DistanceSq = FVector::DistSquared(MeshCenter, VertexPosition);
                    if (DistanceSq > MaxDistanceSq)
                    {
                        MaxDistanceSq = DistanceSq;
                    }
                }
                Radius = FMath::Sqrt(MaxDistanceSq);
                CollisionTransform.SetLocation(MeshCenter);
            }
        }
    }
}

void UDampsSphereCollisionComponent::InitCollisionInfo(UProceduralMeshComponent* pmc)
{
    const TArray<FProcMeshVertex>& ProcVertices = pmc->GetProcMeshSection(0)->ProcVertexBuffer;
    if (ProcVertices.Num() > 0) {
        FVector MeshCenter = FVector::ZeroVector;
        for (const FProcMeshVertex& Vertex : ProcVertices) {
            MeshCenter += Vertex.Position;
        }
        MeshCenter /= ProcVertices.Num();
        float MaxDistanceSq = 0.0f;
        for (const FProcMeshVertex& Vertex : ProcVertices) {
            float DistanceSq = FVector::DistSquared(MeshCenter, Vertex.Position);
            if (DistanceSq > MaxDistanceSq) {
                MaxDistanceSq = DistanceSq;
            }
        }
        Radius = FMath::Sqrt(MaxDistanceSq);
        CollisionTransform.SetLocation(MeshCenter);
    }
}

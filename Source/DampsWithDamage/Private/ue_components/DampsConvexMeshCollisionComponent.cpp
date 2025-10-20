#include "ue_components/DampsConvexMeshCollisionComponent.h"
#include "DampsUEInstance.h"
#include "DrawDebugHelpers.h"

UDampsConvexMeshCollisionComponent::UDampsConvexMeshCollisionComponent() 
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UDampsConvexMeshCollisionComponent::BeginPlay()
{
	Super::BeginPlay();
	
    if (bShouldRegister) {
        // Read mesh info to init the collision geometry
        InitCollisionInfo();

        // Register the Parent Actor into the DampsUEInstance
        RegisterObjectToDamps();
    }
}

void UDampsConvexMeshCollisionComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);

    // Unregister the Parent Actor from the DampsUEInstance
	UnregisterObjectFromDamps();
}

void UDampsConvexMeshCollisionComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // Optionally visualize the collision mesh
	ShowCollisionMesh();
}

void UDampsConvexMeshCollisionComponent::RegisterObjectToDamps()
{
	if (AActor* Owner = GetOwner())
	{
		if (UWorld* World = Owner->GetWorld())
		{
			if (UGameInstance* GameInstance = World->GetGameInstance())
			{
				if (UDampsUEInstance* DampsInstance = GameInstance->GetSubsystem<UDampsUEInstance>())
				{
                    if (vertices.Num() > 0 && indices.Num() > 0)
                    {
                        FTransform transform = Owner->GetTransform();
                        bool isStatic = Owner->GetRootComponent()->Mobility == EComponentMobility::Static ||
                            Owner->GetRootComponent()->Mobility == EComponentMobility::Stationary;
                        DampsInstance->RegisterConvexMeshObject(Owner, transform, vertices, indices, Mass, Friction, Restitution, isStatic);
                    }
				}
			}
		}
	}
}

void UDampsConvexMeshCollisionComponent::UnregisterObjectFromDamps()
{
    if (AActor* Owner = GetOwner())
    {
        if (UWorld* World = Owner->GetWorld())
        {
            if (UGameInstance* GameInstance = World->GetGameInstance())
            {
                if (UDampsUEInstance* DampsInstance = GameInstance->GetSubsystem<UDampsUEInstance>())
                {
                    DampsInstance->UnregisterObject(Owner);
                }
            }
        }
    }
}

void UDampsConvexMeshCollisionComponent::ShowCollisionMesh() {
    // This function is implemented to visualize the collision mesh
	if (AActor* Owner = GetOwner()) {
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
}

void UDampsConvexMeshCollisionComponent::InitCollisionInfo()
{
	// read the render mesh info
	if (AActor* Owner = GetOwner()) {
        if (UStaticMeshComponent* smc = Cast<UStaticMeshComponent>(Owner->GetComponentByClass(UStaticMeshComponent::StaticClass()))) {
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
	}
}

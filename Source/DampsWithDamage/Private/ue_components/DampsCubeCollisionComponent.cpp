#include "ue_components/DampsCubeCollisionComponent.h"
#include "DampsUEInstance.h"
#include "DrawDebugHelpers.h"

UDampsCubeCollisionComponent::UDampsCubeCollisionComponent() 
{
	PrimaryComponentTick.bCanEverTick = true;

	// Read mesh info to init the collision geometry
	InitCollisionInfo();
}

void UDampsCubeCollisionComponent::BeginPlay()
{
	Super::BeginPlay();
	
	// Register the Parent Actor into the DampsUEInstance
	RegisterObjectToDamps();
}

void UDampsCubeCollisionComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);

    // Unregister the Parent Actor from the DampsUEInstance
	UnregisterObjectFromDamps();
}

void UDampsCubeCollisionComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // Optionally visualize the collision mesh
	ShowCollisionMesh();
}

void UDampsCubeCollisionComponent::RegisterObjectToDamps()
{
	if (AActor* Owner = GetOwner())
	{
		if (UWorld* World = Owner->GetWorld())
		{
			if (UGameInstance* GameInstance = World->GetGameInstance())
			{
				if (UDampsUEInstance* DampsInstance = GameInstance->GetSubsystem<UDampsUEInstance>())
				{
                    if (HalfExtents.X > 1e-5f && HalfExtents.Y > 1e-5f && HalfExtents.Z > 1e-5f)
                    {
                        FTransform transform = CollisionTransform * Owner->GetTransform();
                        bool isStatic = Owner->GetRootComponent()->Mobility == EComponentMobility::Static ||
                            Owner->GetRootComponent()->Mobility == EComponentMobility::Stationary;
                        DampsInstance->RegisterCubeObject(Owner, transform, HalfExtents, Mass, Friction, Restitution, isStatic);
                    }
				}
			}
		}
	}
}

void UDampsCubeCollisionComponent::UnregisterObjectFromDamps()
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

void UDampsCubeCollisionComponent::ShowCollisionMesh() {
    // This function is implemented to visualize the collision mesh
	if (AActor* Owner = GetOwner()) {
        FTransform Transform = CollisionTransform * Owner->GetActorTransform();
        FVector Center = Transform.GetTranslation();
		FVector Extents = HalfExtents * Transform.GetScale3D();
        FQuat Rotation = Transform.GetRotation();
        DrawDebugBox(GetWorld(), Center, Extents, Rotation, FColor::Green, false, -1.f, 0, 2.f);
	}
}

void UDampsCubeCollisionComponent::InitCollisionInfo()
{
	// read the render mesh info
	if (AActor* Owner = GetOwner()) {
        if (UStaticMeshComponent* smc = Cast<UStaticMeshComponent>(Owner->GetComponentByClass(UStaticMeshComponent::StaticClass()))) {
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
	}
}

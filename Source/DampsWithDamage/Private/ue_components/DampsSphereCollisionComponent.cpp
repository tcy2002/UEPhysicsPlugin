#include "ue_components/DampsSphereCollisionComponent.h"
#include "DampsUEInstance.h"
#include "DrawDebugHelpers.h"

UDampsSphereCollisionComponent::UDampsSphereCollisionComponent() 
{
	PrimaryComponentTick.bCanEverTick = true;

	// Read mesh info to init the collision geometry
	InitCollisionInfo();
}

void UDampsSphereCollisionComponent::BeginPlay()
{
	Super::BeginPlay();
	
	// Register the Parent Actor into the DampsUEInstance
	RegisterObjectToDamps();
}

void UDampsSphereCollisionComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);

    // Unregister the Parent Actor from the DampsUEInstance
	UnregisterObjectFromDamps();
}

void UDampsSphereCollisionComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // Optionally visualize the collision mesh
	ShowCollisionMesh();
}

void UDampsSphereCollisionComponent::RegisterObjectToDamps()
{
	if (AActor* Owner = GetOwner())
	{
		if (UWorld* World = Owner->GetWorld())
		{
			if (UGameInstance* GameInstance = World->GetGameInstance())
			{
				if (UDampsUEInstance* DampsInstance = GameInstance->GetSubsystem<UDampsUEInstance>())
				{
                    if (Radius > 1e-5f)
                    {
                        FTransform transform = CollisionTransform * Owner->GetTransform();
                        bool isStatic = Owner->GetRootComponent()->Mobility == EComponentMobility::Static ||
                            Owner->GetRootComponent()->Mobility == EComponentMobility::Stationary;
                        DampsInstance->RegisterSphereObject(Owner, transform, Radius, Mass, Friction, Restitution, isStatic);
                    }
				}
			}
		}
	}
}

void UDampsSphereCollisionComponent::UnregisterObjectFromDamps()
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

void UDampsSphereCollisionComponent::ShowCollisionMesh() {
    // This function is implemented to visualize the collision mesh
	if (AActor* Owner = GetOwner()) {
        FTransform Transform = CollisionTransform * Owner->GetActorTransform();
        FVector Center = Transform.GetTranslation();
		float CRadius = Radius * Transform.GetScale3D().GetMax();
        DrawDebugSphere(Owner->GetWorld(), Center, CRadius, 12, FColor::Green, false, -1.0f, 0, 2.0f);
	}
}

void UDampsSphereCollisionComponent::InitCollisionInfo()
{
	// read the render mesh info
	if (AActor* Owner = GetOwner()) {
        if (UStaticMeshComponent* smc = Cast<UStaticMeshComponent>(Owner->GetComponentByClass(UStaticMeshComponent::StaticClass()))) {
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
	}
}

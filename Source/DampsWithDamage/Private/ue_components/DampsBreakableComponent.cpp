#include "ue_components/DampsBreakableComponent.h"
#include "ue_components/DampsSphereCollisionComponent.h"
#include "ue_components/DampsCubeCollisionComponent.h"
#include "ue_components/DampsConvexMeshCollisionComponent.h"
#include "DampsUEInstance.h"

UDampsBreakableComponent::UDampsBreakableComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UDampsBreakableComponent::BeginPlay()
{
    Super::BeginPlay();

    bool hasCollisionComponent = false;
    if (AActor* Owner = GetOwner())
    {
        if (UDampsCubeCollisionComponent* ccc = Cast<UDampsCubeCollisionComponent>(Owner->GetComponentByClass(UDampsCubeCollisionComponent::StaticClass()))) {
            // Cube collision component found
            UE_LOG(LogTemp, Warning, TEXT("BreakableObjectHasCube"));
            hasCollisionComponent = true;
        }

        if (UDampsConvexMeshCollisionComponent* scc = Cast<UDampsConvexMeshCollisionComponent>(Owner->GetComponentByClass(UDampsConvexMeshCollisionComponent::StaticClass()))) {
            // Sphere collision component found
            UE_LOG(LogTemp, Warning, TEXT("BreakableObjectHasConvexMesh"));
            hasCollisionComponent = true;
        }

        // only support cube and convex mesh collision for breakable objects for now
    }

    if (hasCollisionComponent) {
        UE_LOG(LogTemp, Warning, TEXT("RegisterBreakableObjectToDamps"));
        // Register the Breakable Object into the DampsUEInstance
        RegisterBreakableObjectToDamps();
    }
}

void UDampsBreakableComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Super::EndPlay(EndPlayReason);

    // Unregister the Breakable Object from the DampsUEInstance
    UnregisterBreakableObjectFromDamps();
}

void UDampsBreakableComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UDampsBreakableComponent::RegisterBreakableObjectToDamps()
{
    if (AActor* Owner = GetOwner())
    {
        if (UWorld* World = Owner->GetWorld())
        {
            if (UGameInstance* GameInstance = World->GetGameInstance())
            {
                if (UDampsUEInstance* DampsInstance = GameInstance->GetSubsystem<UDampsUEInstance>())
                {
                    UE_LOG(LogTemp, Warning, TEXT("ComponentMarkObjectAsBreakable"));
                    DampsInstance->MarkObjectAsBreakable(Owner, BreakThreshold);
                }
            }
        }
    }
}

void UDampsBreakableComponent::UnregisterBreakableObjectFromDamps()
{
    if (AActor* Owner = GetOwner())
    {
        if (UWorld* World = Owner->GetWorld())
        {
            if (UGameInstance* GameInstance = World->GetGameInstance())
            {
                if (UDampsUEInstance* DampsInstance = GameInstance->GetSubsystem<UDampsUEInstance>())
                {
                    DampsInstance->MarkObjectAsUnbreakable(Owner);
                }
            }
        }
    }
}

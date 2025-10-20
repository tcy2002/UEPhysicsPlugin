#include "ue_components/DampsCollisionComponent.h"
#include "ProceduralMeshComponent.h"
#include "DampsUEInstance.h"
#include "DrawDebugHelpers.h"

UDampsCollisionComponent::UDampsCollisionComponent() 
{
	PrimaryComponentTick.bCanEverTick = true;

    // Check whether there is already a collision component in the owner actor
    if (AActor* Owner = GetOwner()) {
        TArray<UDampsCollisionComponent*> existingComponents;
        Owner->GetComponents<UDampsCollisionComponent>(existingComponents);
        if (existingComponents.Num() > 0) {
            bShouldRegister = false;
        }
    }
}

void UDampsCollisionComponent::BeginPlay()
{
	Super::BeginPlay();
	
    if (bShouldRegister) {
        // Read mesh info to init the collision geometry
        InitCollisionInfo_Internal();

        // Register the Parent Actor into the DampsUEInstance
        RegisterObjectToDamps_Internal();
    }
}

void UDampsCollisionComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);

    // Unregister the Parent Actor from the DampsUEInstance
	UnregisterObjectFromDamps_Internal();
}

void UDampsCollisionComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (bShowCollisionMesh) {
        // Optionally visualize the collision mesh
        ShowCollisionMesh_Internal();
    }
}

void UDampsCollisionComponent::RegisterObjectToDamps_Internal()
{
    if (AActor* Owner = GetOwner())
    {
        if (UWorld* World = Owner->GetWorld())
        {
            if (UGameInstance* GameInstance = World->GetGameInstance())
            {
                if (UDampsUEInstance* DampsInstance = GameInstance->GetSubsystem<UDampsUEInstance>())
                {
                    RegisterObjectToDamps(Owner, DampsInstance);
                }
            }
        }
    }
}

void UDampsCollisionComponent::UnregisterObjectFromDamps_Internal()
{
    if (AActor* Owner = GetOwner())
    {
        if (UWorld* World = Owner->GetWorld())
        {
            if (UGameInstance* GameInstance = World->GetGameInstance())
            {
                if (UDampsUEInstance* DampsInstance = GameInstance->GetSubsystem<UDampsUEInstance>())
                {
                    UnregisterObjectFromDamps(Owner, DampsInstance);
                }
            }
        }
    }
}

void UDampsCollisionComponent::ShowCollisionMesh_Internal() {
    // This function is implemented to visualize the collision mesh
	if (AActor* Owner = GetOwner()) {
        ShowCollisionMesh(Owner);
	}
}

void UDampsCollisionComponent::InitCollisionInfo_Internal()
{
	// read the render mesh info
	if (AActor* Owner = GetOwner()) {
        if (UStaticMeshComponent* smc = Cast<UStaticMeshComponent>(Owner->GetComponentByClass(UStaticMeshComponent::StaticClass()))) {
            if (smc->GetStaticMesh()) {
                InitCollisionInfo(smc);
            }
        }
        else if (UProceduralMeshComponent* pmc = Cast<UProceduralMeshComponent>(Owner->GetComponentByClass(UProceduralMeshComponent::StaticClass()))) {
            if (pmc->GetNumSections() > 0) {
                InitCollisionInfo(pmc);
            }
        }
	}
}

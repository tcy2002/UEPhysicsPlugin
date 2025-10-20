#include "DampsUEInstance.h"
#include "CoordConvert.h"
#include "Kismet/GameplayStatics.h"
#include "phys/fracture/fracture_utils/fracture_data_manager.h"
#include "ProceduralMeshComponent.h"
#include "ue_components/DampsConvexMeshCollisionComponent.h"

UDampsUEInstance::UDampsUEInstance()
{
	// create DampsWorldSim instance
	m_pDampsWorldSim = MakeShared<DampsWorldSim>();

    // Load default material
    static ConstructorHelpers::FObjectFinder<UMaterial> MaterialFinder(TEXT("/Engine/BasicShapes/BasicShapeMaterial"));
    if (MaterialFinder.Succeeded())
    {
        m_pDefaultMaterial = (UMaterial*)MaterialFinder.Object;
    }
}

void UDampsUEInstance::Initialize(FSubsystemCollectionBase& Collection)
{
    m_pDampsWorldSim->OnGetBreakableObjects.BindLambda([this](pe::Array<int>& breakableIds) {
        for (auto& actor : m_BreakableActors) {
            if (m_pActorToWorld.find(actor.first) != m_pActorToWorld.end()) {
                breakableIds.push_back(m_pActorToWorld[actor.first]);
            }
        }
    });

    m_pDampsWorldSim->OnAddConvexMeshObject.BindUObject(this, &UDampsUEInstance::AddNewObject);
    m_pDampsWorldSim->OnRemoveObject.BindUObject(this, &UDampsUEInstance::RemoveObject);
}

void UDampsUEInstance::Deinitialize()
{
    m_pDampsWorldSim->OnGetBreakableObjects.Unbind();
    m_pDampsWorldSim->OnAddConvexMeshObject.Unbind();
    m_pDampsWorldSim->OnRemoveObject.Unbind();
    m_pDampsWorldSim->reset();
    m_BreakableActors.clear();
    m_pActorToWorld.clear();
}

void UDampsUEInstance::Tick(float DeltaTime)
{
    // Check whether the game is paused
    if (UGameplayStatics::IsGamePaused(GetWorld()))
    {
        return;
    }
    
	// Update the Damps world simulation
	if (m_pDampsWorldSim)
	{
        float step = DeltaTime;
        while (step > 0.0001f) {
            float dt = FMath::Min(step, 0.01f); // Max step size of 20 ms
            m_pDampsWorldSim->update(dt);
            step -= dt;
        }
	}
	
	// Sync the transforms of registered actors
	for (auto& Elem : m_pActorToWorld)
	{
		const pe::Transform& trans = m_pDampsWorldSim->GetObjectTransform(Elem.second);
        FTransform oldTransform = Elem.first->GetActorTransform();
		FTransform transform;
		transform.SetLocation(CoordConvert::DampsVector3ToFVector(trans.getOrigin(), true));
		transform.SetRotation(CoordConvert::DampsQuatToFQuat(pe::Quaternion(trans.getBasis())));
        transform.SetScale3D(oldTransform.GetScale3D()); // Keep the original scale
		Elem.first->SetActorTransform(transform);
	}

    UE_LOG(LogTemp, Log, TEXT("DampsUEInstance Tick: DeltaTime = %f"), DeltaTime);
}

void UDampsUEInstance::RegisterConvexMeshObject(AActor* pActor, const FTransform& transform, const TArray<FVector>& vertices, const TArray<int32>& indices, pe::Real mass, pe::Real friction, pe::Real restitution, bool isStatic)
{
	if (m_pDampsWorldSim && pActor)
	{
        pe::Transform dTransform;
        dTransform.setOrigin(CoordConvert::FVectorToDampsVector3(transform.GetLocation(), true));
        dTransform.setBasis(CoordConvert::FQuatToDampsQuat(transform.GetRotation()).toRotationMatrix());

        pe::Mesh mesh;
        for (const FVector& vertex : vertices)
        {
            pe::Mesh::Vertex v;
            FVector scaledVertex = vertex * transform.GetScale3D();
            v.position = CoordConvert::FVectorToDampsVector3(scaledVertex, true);
            v.normal = pe::Vector3::zeros();
            mesh.vertices.push_back(v);
        }
		for (int i = 0; i < indices.Num(); i += 3) 
		{
			pe::Mesh::Face f;
            f.indices.push_back(indices[i]);
            f.indices.push_back(indices[i + 1]);
            f.indices.push_back(indices[i + 2]);
            f.normal = pe::Vector3::zeros();
            mesh.faces.push_back(f);
		}
        pe::Mesh::perVertexNormal(mesh); // Important
        pe_phys_fracture::FractureDataManager fdm;
        fdm.import_from_mesh(mesh);
        fdm.export_to_mesh(mesh);

        int sim_id = m_pDampsWorldSim->AddConvexMeshObject(mesh, dTransform, mass, friction, restitution, isStatic);
        m_pActorToWorld[pActor] = sim_id;
	}
}

void UDampsUEInstance::RegisterCubeObject(AActor* pActor, const FTransform& transform, const FVector& halfExtents, pe::Real mass, pe::Real friction, pe::Real restitution, bool isStatic)
{
    if (m_pDampsWorldSim && pActor) 
    {
        pe::Transform dTransform;
        dTransform.setOrigin(CoordConvert::FVectorToDampsVector3(transform.GetLocation(), true));
        dTransform.setBasis(CoordConvert::FQuatToDampsQuat(transform.GetRotation()).toRotationMatrix());

        FVector scaledHalfExtents = halfExtents * transform.GetScale3D();
        pe::Vector3 extents = CoordConvert::FVectorToDampsAbsVector3(scaledHalfExtents, true) * pe::Real(2.0);

        int sim_id = m_pDampsWorldSim->AddBoxObject(extents, dTransform, mass, friction, restitution, isStatic);
        m_pActorToWorld[pActor] = sim_id;
    }
}

void UDampsUEInstance::RegisterSphereObject(AActor* pActor, const FTransform& transform, float radius, pe::Real mass, pe::Real friction, pe::Real restitution, bool isStatic)
{
    if (m_pDampsWorldSim && pActor)
    {
        pe::Transform dTransform;
        dTransform.setOrigin(CoordConvert::FVectorToDampsVector3(transform.GetLocation(), true));
        dTransform.setBasis(CoordConvert::FQuatToDampsQuat(transform.GetRotation()).toRotationMatrix());

        float scaledRadius = radius * transform.GetScale3D().GetMax(); // Use the maximum scale component
        scaledRadius *= pe::Real(0.01);

        int sim_id = m_pDampsWorldSim->AddSphereObject(scaledRadius, dTransform, mass, friction, restitution, isStatic);
        m_pActorToWorld[pActor] = sim_id;
    }
}

void UDampsUEInstance::UnregisterObject(AActor* pActor)
{
    if (m_pDampsWorldSim && pActor)
    {
        if (m_pActorToWorld.find(pActor) != m_pActorToWorld.end())
        {
            int sim_id = m_pActorToWorld[pActor];
            m_pDampsWorldSim->RemoveObject(sim_id);
            m_pActorToWorld.erase(pActor);
        }
    }
}

void UDampsUEInstance::MarkObjectAsBreakable(AActor* pActor, float threshold)
{
    if (m_pDampsWorldSim && pActor)
    {
        UE_LOG(LogTemp, Warning, TEXT("InstanceMarkObjectAsBreakable"));
        m_BreakableActors[pActor] = threshold;
    }
}

void UDampsUEInstance::MarkObjectAsUnbreakable(AActor* pActor)
{
    if (m_pDampsWorldSim && pActor)
    {
        m_BreakableActors.erase(pActor);
    }
}

void UDampsUEInstance::AddFractureSource(const FString& type, const FVector& position, const FVector& intensity)
{
    UE_LOG(LogTemp, Warning, TEXT("AddFractureSource: %s, %s, %s"), *type, *position.ToString(), *intensity.ToString());
    if (m_pDampsWorldSim)
    {
        pe::Vector3 dPosition = CoordConvert::FVectorToDampsVector3(position, true);
        pe::Vector3 dIntensity = CoordConvert::FVectorToDampsVector3(intensity, false);
        m_pDampsWorldSim->AddFractureSource(TCHAR_TO_UTF8(*type), dPosition, dIntensity);
    }
}

void UDampsUEInstance::AddNewObject(int id, const pe::Mesh& mesh, const pe::Transform& trans, const pe::Real& mass, const pe::Real& fric, const pe::Real& rest, bool isKinematic) {
    TArray<FVector> vertices;
    TArray<int32> indices;
    for (const auto& v : mesh.vertices) {
        FVector vertex = CoordConvert::DampsVector3ToFVector(v.position, true);
        vertices.Add(vertex);
    }
    for (const auto& f : mesh.faces) {
        for (int i = 0; i < f.indices.size() - 2; ++i) {
            indices.Add(f.indices[0]);
            indices.Add(f.indices[i + 1]);
            indices.Add(f.indices[i + 2]);
        }
    }
    
    FTransform transform;
    transform.SetLocation(CoordConvert::DampsVector3ToFVector(trans.getOrigin(), true));
    transform.SetRotation(CoordConvert::DampsQuatToFQuat(pe::Quaternion(trans.getBasis())));
   
    // Note: Scale is not handled here, assuming uniform scale of 1
    
    // Add a new actor
    // Todo: Custom Material
    AActor* newActor = GetWorld()->SpawnActor<AActor>(AActor::StaticClass(), transform);
    UProceduralMeshComponent* procMeshComp = NewObject<UProceduralMeshComponent>(newActor);
    if (procMeshComp) {
        procMeshComp->CreateMeshSection(0, vertices, indices, TArray<FVector>(), TArray<FVector2D>(), TArray<FColor>(), TArray<FProcMeshTangent>(), false);
        procMeshComp->SetMobility(EComponentMobility::Movable);
        if (m_pDefaultMaterial) {
            procMeshComp->SetMaterial(0, m_pDefaultMaterial);
        }
        newActor->SetRootComponent(procMeshComp);
        procMeshComp->RegisterComponent();
    }
    
    UDampsConvexMeshCollisionComponent* collisionComp = NewObject<UDampsConvexMeshCollisionComponent>(newActor);
    if (collisionComp) {
        collisionComp->bShouldRegister = false;
        collisionComp->vertices = vertices;
        collisionComp->indices = indices;
        collisionComp->Mass = mass;
        collisionComp->Friction = fric;
        collisionComp->Restitution = rest;
        m_pActorToWorld[newActor] = id;
        collisionComp->RegisterComponent();
    }
}

void UDampsUEInstance::RemoveObject(int id) {
    AActor* foundActor = nullptr;
    for (auto& Elem : m_pActorToWorld)
    {
        if (Elem.second == id)
        {
            foundActor = Elem.first;
            m_pActorToWorld.erase(Elem.first);
            break;
        }
    }
    if (m_BreakableActors.find(foundActor) != m_BreakableActors.end()) {
        m_BreakableActors.erase(foundActor);
    }
    foundActor->Destroy();
}

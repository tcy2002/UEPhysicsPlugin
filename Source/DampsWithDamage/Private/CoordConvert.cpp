#include "CoordConvert.h"

pe::Quaternion CoordConvert::DampsToFConvertQuat = pe::Quaternion(pe::Matrix3::fromRotation(pe::Vector3::right(), PE_PI / 2));
pe::Quaternion CoordConvert::DampsToFConvertQuatInverse = CoordConvert::DampsToFConvertQuat.inverse();

pe::Quaternion CoordConvert::FToDampsConvertQuat = pe::Quaternion(pe::Matrix3::fromRotation(pe::Vector3::right(), -PE_PI / 2));
pe::Quaternion CoordConvert::FToDampsConvertQuatInverse = CoordConvert::FToDampsConvertQuat.inverse();

FVector CoordConvert::DampsVector3ToFVector(const pe::Vector3& dVec, const bool& zoom)
{
    /*pe::Vector3 dConvertedVec = DampsToFConvertQuat * dVec;
    FVector fConvertedFVec(dConvertedVec.x(), -dConvertedVec.y(), dConvertedVec.z());
    if (zoom)
        fConvertedFVec *= 100;
    return fConvertedFVec;*/
    FVector dConvertedVec(dVec.x, dVec.z, dVec.y);
    if (zoom)
        dConvertedVec *= 100.0;
    return dConvertedVec;
}

pe::Vector3 CoordConvert::FVectorToDampsVector3(const FVector& fVec, const bool& zoom)
{
    /*pe::Vector3 dVec(fVec.X, -fVec.Y, fVec.Z);
    pe::Vector3 dConvertedVec = FToDampsConvertQuat * dVec;
    if (zoom)
        dConvertedVec *= 0.01;
    return dConvertedVec;*/
    pe::Vector3 dVec(fVec.X, fVec.Z, fVec.Y);
    if (zoom)
        dVec *= 0.01;
    return dVec;
}

FQuat CoordConvert::DampsQuatToFQuat(const pe::Quaternion& dQuat)
{
    pe::Quaternion dConvertedQuat = DampsToFConvertQuat * dQuat * DampsToFConvertQuatInverse;
    return FQuat(-dConvertedQuat.x, dConvertedQuat.y, -dConvertedQuat.z, dConvertedQuat.w);
}

pe::Quaternion CoordConvert::FQuatToDampsQuat(const FQuat& fQuat)
{
    pe::Quaternion dQuat(fQuat.W, -fQuat.X, fQuat.Y, -fQuat.Z);
    return FToDampsConvertQuat * dQuat * FToDampsConvertQuatInverse;
}

FVector CoordConvert::DampsVector3ToAbsFVector(const pe::Vector3& dVec, const bool& zoom)
{
    return DampsVector3ToFVector(dVec, zoom).GetAbs();
}

pe::Vector3 CoordConvert::FVectorToDampsAbsVector3(const FVector& fVec, const bool& zoom)
{
    const pe::Vector3 v = FVectorToDampsVector3(fVec, zoom);
    return pe::Vector3(PE_ABS(v.x), PE_ABS(v.y), PE_ABS(v.z));
}

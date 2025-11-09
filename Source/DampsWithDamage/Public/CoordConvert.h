#pragma once

#include "physics/physics.h"
#include "CoreMinimal.h"

/*
	- UE4 是左手系，默认重力方向是 -z，单位是 cm
	- Damps 是右手系，默认重力方向是 -y，单位是 m
	- 以 Damps 转 UE 为例：
		1. 坐标：将 Damps 下的坐标绕 x 轴旋转 90°，使 Damps 的 y 坐标变为 z 坐标，对齐 UE 的重力方向；
			之后将转化后的 y 坐标变为负值，从 Damps 右手系转为 UE 左手系；
		2. 旋转：从 Damps 到 UE 的坐标转化四元数为 Q_D2U，四元数的转化关系为 Q_U = Q_D2U * Q_D * Q_D2U^-1;
			四元数从右手系转化关系如下（Y 轴取反， 参考 https://blog.csdn.net/weixin_40277515/article/details/89323615）：
			w_L = w_R， x_L = -x_R， y_L = y_R， z_L = -z_R
			即，w 和取反轴不变，其余坐标变为负值
		3. 缩放：从 Damps（m）到 UE（cm）需要 * 100；
*/

class CoordConvert
{
public:
	static FVector DampsVector3ToFVector(const pe::Vector3& dVec, const bool& zoom = false);
	static pe::Vector3 FVectorToDampsVector3(const FVector& fVec, const bool& zoom = false);
	
	static FQuat DampsQuatToFQuat(const pe::Quaternion& dQuat);
	static pe::Quaternion FQuatToDampsQuat(const FQuat& fQuat);
	
	static FVector DampsVector3ToAbsFVector(const pe::Vector3& dVec, const bool& zoom = false);
	static pe::Vector3 FVectorToDampsAbsVector3(const FVector& fVec, const bool& zoom = false);

    static FTransform DampsTransformToFTransform(const pe::Transform& dTrans, const bool& zoom = false);
    static pe::Transform FTransformToDampsTransform(const FTransform& fTrans, const bool& zoom = false);

private:
	static pe::Quaternion DampsToFConvertQuat;
	static pe::Quaternion DampsToFConvertQuatInverse;
	
	static pe::Quaternion FToDampsConvertQuat;
	static pe::Quaternion FToDampsConvertQuatInverse;
};
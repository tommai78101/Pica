#include "physics.h"

/*
 * COMMON.C 
 */

/**
 * @brief Obtain the smaller C3D_FVec vector of the given 2 C3D_FVec vectors.
 * @param[in]    lhs      C3D_FVec vector to compare.
 * @param[in]    rhs      C3D_FVec vector to compare.
 * @return The smallest C3D_FVec vector.
 */
C3D_FVec FVec3_Min(C3D_FVec lhs, C3D_FVec rhs)
{
	return FVec3_New((lhs.x < rhs.x ? lhs.x : rhs.x), (lhs.y < rhs.y ? lhs.y : rhs.y), (lhs.z < rhs.z ? lhs.z : rhs.z));
}

/**
 * @brief Obtain the larger C3D_FVec vector of the given 2 C3D_FVec vectors.
 * @param[in]    lhs      C3D_FVec vector to compare.
 * @param[in]    rhs      C3D_FVec vector to compare.
 * @return The largest C3D_FVec vector.
 */
C3D_FVec FVec3_Max(C3D_FVec lhs, C3D_FVec rhs)
{
	return FVec3_New((lhs.x > rhs.x ? lhs.x : rhs.x), (lhs.y > rhs.y ? lhs.y : rhs.y), (lhs.z > rhs.z ? lhs.z : rhs.z));
}

/**
 * @brief Obtain the C3D_FVec vector containing the absolute values of each element.
 * @param[in]    vector        The original vector.
 * @return The C3D_FVec vector with absolute values of each element. 
 */
C3D_FVec FVec3_Abs(C3D_FVec vector)
{
	return FVec3_New(fabsf(vector.x), fabsf(vector.y), fabsf(vector.z));
}

/**
 * @brief See: http://box2d.org/2014/02/computing-a-basis/
 * @note DO MIND THE ORDER OF THE PARAMETERS!
 *       See: http://box2d.org/2014/02/computing-a-basis/
 *       Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
 *       Then 3*s*s = 1, s = sqrt(1/3) = 0.57735027. This means that at least one component of a
 *       unit vector must be greater or equal to 0.57735027. Can use SIMD select operation.
 * @param[out]  b   A unit vector perpendicular to unit vector, "a".
 * @param[out]  c   A unit vector perpendicular to unit vectors, "a" and "b".
 * @param[in]   a   The unit vector used for calculating the unit vectors, "b" and "c".
 */ 
void FVec3_ComputeBasis(C3D_FVec* b, C3D_FVec* c, const C3D_FVec a)
{
	if (fabsf(a.x) >= 0.57735027f)
		*b = FVec3_New(a.y, -a.x, 0.0f);
	else
		*b = FVec3_New(0.0f, a.z, -a.y);
	*b = FVec3_Normalize(*b);
	*c = FVec3_Cross(a, *b);
}

/**
 * @brief Creates a C3D_Mtx containing the outer product of C3D_FVec vectors, lhs and rhs.
 * @param[out]   out     The resulting C3D_Mtx matrix.
 * @param[in]    lhs     The first C3D_FVec vector.
 * @param[in]    rhs     The second C3D_FVec vector.
 */
void Mtx_OuterProduct(C3D_Mtx* out, C3D_FVec lhs, C3D_FVec rhs)
{
	C3D_FVec a = FVec3_Scale(rhs, lhs.x);
	C3D_FVec b = FVec3_Scale(rhs, lhs.y);
	C3D_FVec c = FVec3_Scale(rhs, lhs.z);
	Mtx_Zeros(out);
	out->r[0].x = a.x;
	out->r[0].y = a.y;
	out->r[0].z = a.z;
	out->r[1].x = b.x;
	out->r[1].y = b.y;
	out->r[1].z = b.z;
	out->r[2].x = c.x;
	out->r[2].y = c.y;
	out->r[2].z = c.z;
	out->r[3].w = 1.0f;
}

/**
 * @brief Integrate the quaternion with a given angular velocity and time step. This will update the dynamic state of a rigid body.
 * @param[in]     quaternion         The quaternion to integrate with.
 * @param[in]     angularVelocity    The angular velocity.
 * @param[in]     deltaTime          The time step intervals to integrate.
 * @return An integrated C3D_FQuat quaternion with respect to the angular velocity and time.   
 */
C3D_FQuat Quat_Integrate(C3D_FQuat quaternion, C3D_FVec angularVelocity, float deltaTime)
{
	C3D_FQuat deltaQuaternion = Quat_New(angularVelocity.x * deltaTime, angularVelocity.y * deltaTime, angularVelocity.z * deltaTime, 0.0f);
	deltaQuaternion = Quat_Multiply(deltaQuaternion, quaternion);
	quaternion.x += deltaQuaternion.x * 0.5f;
	quaternion.y += deltaQuaternion.y * 0.5f;
	quaternion.z += deltaQuaternion.z * 0.5f;
	quaternion.w += deltaQuaternion.w * 0.5f;
	return Quat_Normalize(quaternion);
}

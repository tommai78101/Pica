#include "physics.h"

/*
 * AABB.C 
 */

/**
 * @brief Checks if inner AABB box is within the outer AABB box.
 * @param[in]   outer   Outer AABB box.
 * @param[in]   inner    Inner AABB box to compare with.
 * @return True if outer AABB box contains an inner AABB box. False, if otherwise. 
 */
bool AABB_ContainsAABB(const C3D_AABB* outer, const C3D_AABB* inner) 
{
	return outer->min.x <= inner->min.x &&
	       outer->min.y <= inner->min.y &&
		   outer->min.z <= inner->min.z &&
		   outer->max.x >= inner->max.x &&
		   outer->max.y >= inner->max.y &&
		   outer->max.z >= inner->max.z;
}

/**
 * @brief Checks if C3D_FVec point is within the AABB box.
 * @param[in]   outer   Outer AABB box.
 * @param[in]   inner   C3D_FVec vector position.
 * @return True if vector position is within the AABB box. False, if otherwise.
 */
bool AABB_ContainsFVec3(const C3D_AABB* outer, const C3D_FVec inner) 
{
	return outer->min.x <= inner.x &&
	       outer->min.y <= inner.y &&
		   outer->min.z <= inner.z &&
		   outer->max.x >= inner.x &&
		   outer->max.y >= inner.y &&
		   outer->max.z >= inner.z;
}

/**
 * @brief Obtain the surface area of the AABB box specified.
 * @param[in]  myself   The AABB box for finding the surface area.
 * @return The surface area.
 */
float AABB_GetSurfaceArea(const C3D_AABB* myself) 
{
	float x = myself->max.x - myself->min.x;
	float y = myself->max.y - myself->min.y;
	float z = myself->max.z - myself->min.z;
	return (float) (2.0f) * (x * x + x * z + y * z);
}

/**
 * @brief Unionize two AABB boxes to create 1 bigger AABB box that contains the two AABB boxes.
 * @param[out]   out   The resulting larger AABB box.
 * @param[in]    a     The first AABB box.
 * @param[in]    b     The second AABB box.
 */
void AABB_Combine(C3D_AABB* out, const C3D_AABB* a, const C3D_AABB* b)
{
	out->min = FVec3_New((a->min.x < b->min.x ? a->min.x : b->min.x), (a->min.y < b->min.y ? a->min.y : b->min.y), (a->min.z < b->min.z ? a->min.z : b->min.z));
	out->max = FVec3_New((a->max.x > b->max.x ? a->max.x : b->max.x), (a->max.y > b->max.y ? a->max.y : b->max.y), (a->max.z > b->max.z ? a->max.z : b->max.z));
}

/**
 * @brief Checks if the two AABB boxes intersects each other.
 * @param[in]    a     The first AABB box.
 * @param[in]    b     The second AABB box.
 * @return True if both boxes intersect each other. False, if otherwise.
 */
bool AABB_CollidesAABB(const C3D_AABB* a, const C3D_AABB* b)
{
	if (a->max.x < b->min.x || a->min.x > b->max.x)
		return false;
	if (a->max.y < b->min.y || a->min.y > b->max.y)
		return false;
	if (a->max.z < b->min.z || a->min.z > b->max.z)
		return false;
	return true;
}

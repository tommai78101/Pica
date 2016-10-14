#include "physics.h"

bool AABB_ContainsAABB(const C3D_AABB* outer, const C3D_AABB* inner) 
{
	return outer->min.x <= inner->min.x &&
	       outer->min.y <= inner->min.y &&
		   outer->min.z <= inner->min.z &&
		   outer->max.x >= inner->max.x &&
		   outer->max.y >= inner->max.y &&
		   outer->max.z >= inner->max.z;
}

bool AABB_ContainsFVec3(const C3D_AABB* outer, const C3D_FVec* inner) 
{
	return outer->min.x <= inner->x &&
	       outer->min.y <= inner->y &&
		   outer->min.z <= inner->z &&
		   outer->max.x >= inner->x &&
		   outer->max.y >= inner->y &&
		   outer->max.z >= inner->z;
}

float AABB_GetSurfaceArea(const C3D_AABB* myself) 
{
	float x = myself->max.x - myself->min.x;
	float y = myself->max.y - myself->min.y;
	float z = myself->max.z - myself->min.z;
	return (float) (2.0f) * (x * x + x * z + y * z);
}

void AABB_Combine(C3D_AABB* out, const C3D_AABB* a, const C3D_AABB* b)
{
	out->min = FVec3_New((a->min.x < b->min.x ? a->min.x : b->min.x), (a->min.y < b->min.y ? a->min.y : b->min.y), (a->min.z < b->min.z ? a->min.z : b->min.z));
	out->max = FVec3_New((a->max.x > b->max.x ? a->max.x : b->max.x), (a->max.y > b->max.y ? a->max.y : b->max.y), (a->max.z > b->max.z ? a->max.z : b->max.z));
}

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

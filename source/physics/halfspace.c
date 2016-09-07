#include <c3d/physics.h>

C3D_FVec HS_GetOrigin(C3D_HalfSpace* in)
{
	C3D_FVec result = FVec3_Scale(in->normal, in->distance);
	result.w = 1.0f;
	return result;
}

float HS_GetDistance(C3D_HalfSpace* in, const C3D_FVec* point)
{
	return (FVec3_Dot(in->normal, *point) - in->distance);
}

C3D_FVec HS_Project(C3D_HalfSpace* in, const C3D_FVec* point)
{
	return FVec3_Subtract(*point, FVec3_Scale(in->normal, HS_GetDistance(in, point)));
}

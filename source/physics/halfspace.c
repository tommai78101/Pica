#include <c3d/physics.h>

C3D_FVec HS_GetOrigin(C3D_HalfSpace* in)
{
	C3D_FVec result = FVec3_Scale(in->normal, in->distance);
	result.w = 1.0f;
	return result;
}

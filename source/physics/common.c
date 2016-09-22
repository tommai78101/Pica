#include <c3d/physics.h>

C3D_FVec FVec3_Min(C3D_FVec lhs, C3D_FVec rhs)
{
	return FVec3_New((lhs.x < rhs.x ? lhs.x : rhs.x), (lhs.y < rhs.y ? lhs.y : rhs.y), (lhs.z < rhs.z ? lhs.z : rhs.z));
}

C3D_FVec FVec3_Max(C3D_FVec lhs, C3D_FVec rhs)
{
	return FVec3_New((lhs.x > rhs.x ? lhs.x : rhs.x), (lhs.y > rhs.y ? lhs.y : rhs.y), (lhs.z > rhs.z ? lhs.z : rhs.z));
}

void FVec3_ComputeBasis(const C3D_FVec* a, C3D_FVec* b, C3D_FVec* c)
{
	// See: http://box2d.org/2014/02/computing-a-basis/
	// Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
	// Then 3*s*s = 1, s = sqrt(1/3) = 0.57735027. This means that at least one component of a
	// unit vector must be greater or equal to 0.57735027. Can use SIMD select operation.

	if (fabsf(a->x) >= 0.57735027f)
		*b = FVec3_New(a->y, -a->x, 0.0f);
	else
		*b = FVec3_New(0.0f, a->z, -a->y);

	*b = FVec3_Normalize(*b);
	*c = FVec3_Cross(*a, *b);
}

void Mtx_OuterProduct(C3D_Mtx* out, C3D_FVec* lhs, C3D_FVec* rhs)
{
	C3D_FVec a = FVec3_Scale(*rhs, lhs->x);
	C3D_FVec b = FVec3_Scale(*rhs, lhs->y);
	C3D_FVec c = FVec3_Scale(*rhs, lhs->z);
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

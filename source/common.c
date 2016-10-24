#include "physics.h"

C3D_FVec FVec3_Min(C3D_FVec lhs, C3D_FVec rhs)
{
	return FVec3_New((lhs.x < rhs.x ? lhs.x : rhs.x), (lhs.y < rhs.y ? lhs.y : rhs.y), (lhs.z < rhs.z ? lhs.z : rhs.z));
}

C3D_FVec FVec3_Max(C3D_FVec lhs, C3D_FVec rhs)
{
	return FVec3_New((lhs.x > rhs.x ? lhs.x : rhs.x), (lhs.y > rhs.y ? lhs.y : rhs.y), (lhs.z > rhs.z ? lhs.z : rhs.z));
}

C3D_FVec FVec3_Abs(C3D_FVec vector)
{
	return FVec3_New(fabsf(vector.x), fabsf(vector.y), fabsf(vector.z));
}

void FVec3_ComputeBasis(C3D_FVec* b, C3D_FVec* c, const C3D_FVec a)
{
	if (fabsf(a.x) >= 0.57735027f)
		*b = FVec3_New(a.y, -a.x, 0.0f);
	else
		*b = FVec3_New(0.0f, a.z, -a.y);
	*b = FVec3_Normalize(*b);
	*c = FVec3_Cross(a, *b);
}

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

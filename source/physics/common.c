#include <c3d/physics.h>

C3D_FVec FVec3_Min(C3D_FVec lhs, C3D_FVec rhs)
{
	return FVec3_New((lhs.x < rhs.x ? lhs.x : rhs.x), (lhs.y < rhs.y ? lhs.y : rhs.y), (lhs.z < rhs.z ? lhs.z : rhs.z));
}

C3D_FVec FVec3_Max(C3D_FVec lhs, C3D_FVec rhs)
{
	return FVec3_New((lhs.x > rhs.x ? lhs.x : rhs.x), (lhs.y > rhs.y ? lhs.y : rhs.y), (lhs.z > rhs.z ? lhs.z : rhs.z));
}

void Mtx_Diagonal(C3D_Mtx* out, float x, float y, float z, float w)
{
	Mtx_Identity(out);
	FVec4_Scale(out->r[0], x);
	FVec4_Scale(out->r[1], y);
	FVec4_Scale(out->r[2], z);
	FVec4_Scale(out->r[3], w);
}

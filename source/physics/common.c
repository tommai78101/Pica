#include <c3d/physics.h>

C3D_FVec FVec3_Min(C3D_FVec lhs, C3D_FVec rhs)
{
	return FVec3_New((lhs.x < rhs.x ? lhs.x : rhs.x), (lhs.y < rhs.y ? lhs.y : rhs.y), (lhs.z < rhs.z ? lhs.z : rhs.z));
}

C3D_FVec FVec3_Max(C3D_FVec lhs, C3D_FVec rhs)
{
	return FVec3_New((lhs.x > rhs.x ? lhs.x : rhs.x), (lhs.y > rhs.y ? lhs.y : rhs.y), (lhs.z > rhs.z ? lhs.z : rhs.z));
}

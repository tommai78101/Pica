#include <physics.h>

void Manifold_SetPair(C3D_Manifold* manifold, C3D_Box* boxA, C3D_Box* boxB)
{
	manifold->A = boxA;
	manifold->B = boxB;
	manifold->sensor = boxA->sensor || boxB->sensor;
}

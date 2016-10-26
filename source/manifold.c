#include "physics.h"

/*
 * MANIFOLD.C 
 */

/**
 * @brief Sets the C3D_Manifold object to store the C3D_Box pair, boxA and boxB. The C3D_Manifold object is used for solving collisions.
 * @param[in,out]    manifold       The resulting C3D_Manifold to store the C3D_Box pair, A and B.
 * @param[in]        boxA           The first C3D_Box.
 * @param[in]        boxB           The second C3D_Box.
 */
void Manifold_SetPair(C3D_Manifold* manifold, C3D_Box* boxA, C3D_Box* boxB)
{
	manifold->A = boxA;
	manifold->B = boxB;
	manifold->sensor = boxA->sensor || boxB->sensor;
}

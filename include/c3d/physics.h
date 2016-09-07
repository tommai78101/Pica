#pragma once
#include "types.h"
#include "maths.h"
#include <stdbool.h>

typedef struct C3D_AABB {
	C3D_FVec min;
	C3D_FVec max;
} C3D_AABB;

typedef struct C3D_HalfSpace {
	C3D_FVec normal;
	float distance;
} C3D_HalfSpace;

typedef struct C3D_RaycastData {
	C3D_FVec rayOrigin;
	C3D_FVec direction;
	float deltaTime;
	float timeOfImpact;
	C3D_FVec normal;
} C3D_RaycastData;

/***********************************************
 * AABB Box Collision Helper Functions 
 ***********************************************/

/**
 * @brief Checks if inner AABB box is within the outer AABB box.
 * @param[in]   outer   Outer AABB box.
 * @param[in]   inner    Inner AABB box to compare with.
 * @return True if outer AABB box contains an inner AABB box. False, if otherwise. 
 */
bool AABB_ContainsAABB(const C3D_AABB* outer, const C3D_AABB* inner);

/**
 * @brief Checks if C3D_FVec point is within the AABB box.
 * @param[in]   outer   Outer AABB box.
 * @param[in]   inner   C3D_FVec vector position.
 * @return True if vector position is within the AABB box. False, if otherwise.
 */
bool AABB_ContainsFVec3(const C3D_AABB* outer, const C3D_FVec* inner);

/**
 * @brief Obtain the surface area of the AABB box specified.
 * @param[in]  myself   The AABB box for finding the surface area.
 * @return The surface area.
 */
float AABB_GetSurfaceArea(const C3D_AABB* myself);

/**
 * @brief Unionize two AABB boxes to create 1 bigger AABB box that contains the two AABB boxes.
 * @param[out]   out   The resulting larger AABB box.
 * @param[in]    a     The first AABB box.
 * @param[in]    b     The second AABB box.
 */
void AABB_Combine(C3D_AABB* out, const C3D_AABB* a, const C3D_AABB* b);

/**
 * @brief Checks if the two AABB boxes intersects each other.
 * @param[in]    a     The first AABB box.
 * @param[in]    b     The second AABB box.
 * @return True if both boxes intersect each other. False, if otherwise.
 */
bool AABB_CollidesAABB(const C3D_AABB* a, const C3D_AABB* b);

/**
 * @brief See: http://box2d.org/2014/02/computing-a-basis/
 * @param[in]   a   A unit vector.
 * @param[out]  b   A unit vector perpendicular to unit vector, "a".
 * @param[out]  c   A unit vector perpendicular to unit vectors, "a" and "b".
 */ 
void ComputeBasis(const C3D_FVec* a, C3D_FVec* b, C3D_FVec* c);

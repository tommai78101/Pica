#include "physics.h"

/**
 * @brief To generate contact information, and to solve collisions from a given C3D_Constraint object.
 * @param[in,out]      constraint       A C3D_Constraint object to generate contact information with.
 */
void Constraint_CollisionResponse(C3D_ContactConstraint* constraint)
{
	constraint->manifold.contactsCount = 0;
	Collision_BoxToBox(&constraint->manifold, constraint->A, constraint->B);
	if (constraint->manifold.contactsCount > 0)
	{
		if (constraint->flags & ConstraintFlag_Colliding)
			constraint->flags |= ConstraintFlag_WasColliding;
		else
			constraint->flags |= ConstraintFlag_Colliding;
	}
	else 
	{
		if (constraint->flags & ConstraintFlag_Colliding)
		{
			constraint->flags &= ~ConstraintFlag_Colliding;
			constraint->flags |= ConstraintFlag_WasColliding;
		}
		else
			constraint->flags &= ~ConstraintFlag_WasColliding;
	}
}




#include "physics.h"

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




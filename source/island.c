#include "physics.h"

/*
 * ISLAND.C 
 */

/**
 * @brief Initializes the C3D_Island object.
 * @param[in,out]         island             The resulting C3D_Island object.
 */
void Island_Init(C3D_Island* island)
{
	for (unsigned int i = 0; i < island->contactConstraintCount; i++)
	{
		C3D_ContactConstraint* constraint = island->contactConstraints[i];
		C3D_ContactConstraintState* constraintState = island->contactConstraintStates + i;
		constraintState->centerA = constraint->bodyA->worldCenter;
		constraintState->inertiaA = constraint->bodyA->inverseInertiaWorld;
		constraintState->inverseMassA = constraint->bodyA->inverseMass;
		constraintState->indexBodyA = constraint->bodyA->islandIndex;
		constraintState->centerB = constraint->bodyB->worldCenter;
		constraintState->inertiaB = constraint->bodyB->inverseInertiaWorld;
		constraintState->inverseMassB = constraint->bodyB->inverseMass;
		constraintState->indexBodyB = constraint->bodyB->islandIndex;
		constraintState->restitution = constraint->restitution;
		constraintState->friction = constraint->friction;
		constraintState->normal = constraint->manifold.normal;
		constraintState->tangentVectors[0] = constraint->manifold.tangentVectors[0];
		constraintState->tangentVectors[1] = constraint->manifold.tangentVectors[1];
		constraintState->contactCount = constraint->manifold.contactsCount;
		for (int j = 0; j < constraintState->contactCount; j++)
		{
			C3D_ContactState* state = constraintState->contactStates + j;
			C3D_Contact* contact = constraint->manifold.contacts + j;
			state->radiusContactA = FVec3_Subtract(contact->position, constraintState->centerA);
			state->radiusContactB = FVec3_Subtract(contact->position, constraintState->centerB);
			state->penetration = contact->penetration;
			state->normalImpulse = contact->normalImpulse;
			state->tangentImpulse[0] = contact->tangentImpulse[0];
			state->tangentImpulse[1] = contact->tangentImpulse[1];
		}
	}
}

/**
 * @brief Adds a C3D_Body body object to the C3D_Island object.
 * @param[in,out]      island         The resulting C3D_Island object.
 * @param[in]          body           The C3D_Body object to add to the C3D_Island object. The C3D_Body object will use the previous C3D_Island's total C3D_Body count as its index.
 */
void Island_AddBody(C3D_Island* island, C3D_Body* const body)
{
	assert(island->bodyCount < island->bodyCapacity);
	body->islandIndex = island->bodyCount;
	island->bodies[island->bodyCount++] = body;
}

/**
 * @brief Adds a C3D_ContactConstraint constraint state to the C3D_Island object.
 * @param[in,out]     island                    The resulting C3D_Island object.
 * @param[in]         contactConstraint         The C3D_ContactConstraint constraint state to add to the C3D_Island object.
 */
void Island_AddContactConstraint(C3D_Island* island, C3D_ContactConstraint* const constraint)
{
	assert(island->contactConstraintCount < island->contactConstraintCapacity);
	island->contactConstraints[island->contactConstraintCount++] = constraint;
}

/**
 * @brief Updates all C3D_Body objects' contact points, validate them, and handle sleeping objects.
 * @note From Box2D - Applies damping.
         Ordinary differential equation (ODE): dv/dt + c * v = 0
         Solution: v(t) = v0 * exp(-c * t)
         Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
                    v2 = exp(-c * dt) * v1
         Padé approximation: v2 = v1 * 1 / (1 + c * dt)
 * @param[in,out]     island     The resulting C3D_Island object to update/validate. 
 */
void Island_Solve(C3D_Island* island)
{
	for (unsigned int i = 0; i < island->bodyCount; i++)
	{
		C3D_Body* body = island->bodies[i];
		C3D_VelocityState* velocityState = island->velocityStates + i;
		if (body->flags & BodyFlag_Dynamic)
		{
			Body_ApplyLinearForce(body, FVec3_Scale(island->gravity, body->gravityScale));
			C3D_Mtx rotationMatrix;
			Mtx_Copy(&rotationMatrix, &body->transform.rotation);
			C3D_Mtx transposedRotationMatrix;
			Mtx_Copy(&transposedRotationMatrix, &body->transform.rotation);
			Mtx_Transpose(&transposedRotationMatrix);
			C3D_Mtx temp;
			Mtx_Multiply(&temp, &rotationMatrix, &body->inverseInertiaModel);
			Mtx_Multiply(&body->inverseInertiaWorld, &temp, &transposedRotationMatrix);
			body->linearVelocity = FVec3_Add(body->linearVelocity, FVec3_Scale(FVec3_Scale(body->force, body->inverseMass), island->deltaTime));
			body->angularVelocity = FVec3_Add(body->angularVelocity, FVec3_Scale(Mtx_MultiplyFVec3(&body->inverseInertiaWorld, body->torque), island->deltaTime));
			body->linearVelocity = FVec3_Scale(body->linearVelocity, 1.0f / (1.0f + (island->deltaTime * body->linearDamping)));
			body->angularVelocity = FVec3_Scale(body->angularVelocity, 1.0f / (1.0f + (island->deltaTime * body->angularDamping)));
		}
		velocityState->v = body->linearVelocity;
		velocityState->w = body->angularVelocity;
	}
	C3D_ContactSolver contactSolver;
	Solver_Init(&contactSolver, island);
	Solver_PreSolve(&contactSolver, island->deltaTime);
	for (int i = 0; i < island->iterations; i++)
		Solver_Solve(&contactSolver);
	Solver_Free(&contactSolver);
	for (unsigned int i = 0; i < island->bodyCount; i++)
	{
		C3D_Body* body = island->bodies[i];
		C3D_VelocityState* velocityState = island->velocityStates + i;
		if (body->flags & BodyFlag_Static)
			continue;
		body->linearVelocity = velocityState->v;
		body->angularVelocity = velocityState->w;
		body->worldCenter = FVec3_Add(body->worldCenter, FVec3_Scale(body->linearVelocity, island->deltaTime));
		body->quaternion = Quat_Integrate(body->quaternion, body->angularVelocity, island->deltaTime);
		Mtx_FromQuat(&body->transform.rotation, body->quaternion);
	}
	if (island->allowSleep)
	{
		float minimumSleepTime = FLT_MAX;
		for (unsigned int i = 0; i < island->bodyCount; i++)
		{
			C3D_Body* body = island->bodies[i];
			if (body->flags & BodyFlag_Static)
				continue;
			const float squareLinearVelocity = FVec3_Dot(body->linearVelocity, body->linearVelocity);
			const float squareAngularVelocity = FVec3_Dot(body->angularVelocity, body->angularVelocity);
			const float linearTolerance = C3D_SLEEP_LINEAR;
			const float angularTolerance = C3D_SLEEP_ANGULAR;
			if (squareLinearVelocity > linearTolerance || squareAngularVelocity > angularTolerance)
			{
				minimumSleepTime = 0.0f;
				body->sleepTime = 0.0f;
			}
			else 
			{
				body->sleepTime += island->deltaTime;
				minimumSleepTime = (minimumSleepTime < body->sleepTime ? minimumSleepTime : body->sleepTime);
			}
		}
		if (minimumSleepTime > C3D_SLEEP_TIME)
		{
			for (unsigned int i = 0; i < island->bodyCount; i++)
				Body_SetSleep(island->bodies[i]);
		}
	}
}

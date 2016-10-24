#include "physics.h"

/**
 * @brief Updates all C3D_Body objects' contact points, validate them, and handle sleeping objects.
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

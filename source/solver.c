#include "physics.h"

void Solver_Init(C3D_ContactSolver* solver, C3D_Island* island)
{
	solver->island = island;
	solver->contactConstraintStateCount = island->contactConstraintStateCount;
	solver->contactConstraintStates = island->contactConstraintStates;
	solver->velocityStates = island->velocityStates;
	solver->enableFriction = island->enableFriction;
}

void Solver_Free(C3D_ContactSolver* solver)
{
	for (unsigned int i = 0; i < solver->contactConstraintStateCount; i++)
	{
		C3D_ContactConstraintState* constraintStateIterator = solver->contactConstraintStates + i;
		C3D_ContactConstraint* contactConstraintPointer = solver->island->contactConstraints[i];
		for (int j = 0; j < constraintStateIterator->contactCount; j++)
		{
			C3D_Contact* originalContact = contactConstraintPointer->manifold.contacts + j;
			C3D_ContactState* contactState = constraintStateIterator->contactStates + j;
			originalContact->normalImpulse = contactState->normalImpulse;
			originalContact->tangentImpulse[0] = contactState->tangentImpulse[0];
			originalContact->tangentImpulse[1] = contactState->tangentImpulse[1];
		}
	}
}

void Solver_PreSolve(C3D_ContactSolver* solver, float deltaTime)
{
	for (unsigned int i = 0; i < solver->contactConstraintStateCount; i++)
	{
		C3D_ContactConstraintState* constraintState = solver->contactConstraintStates + i;
		C3D_FVec vAxisA = solver->velocityStates[constraintState->indexBodyA].v;
		C3D_FVec wAxisA = solver->velocityStates[constraintState->indexBodyA].w;
		C3D_FVec vAxisB = solver->velocityStates[constraintState->indexBodyB].v;
		C3D_FVec wAxisB = solver->velocityStates[constraintState->indexBodyB].w;
		for (int j = 0; j < constraintState->contactCount; j++)
		{
			C3D_ContactState* contactState = constraintState->contactStates + j;
			C3D_FVec contactPointA_Cross_normal = FVec3_Cross(contactState->radiusContactA, constraintState->normal);
			C3D_FVec contactPointB_Cross_normal = FVec3_Cross(contactState->radiusContactB, constraintState->normal);
			float netMass = constraintState->indexMassA + constraintState->indexMassB;
			float totalMass[2];
			totalMass[0] = netMass;
			totalMass[1] = netMass;
			netMass += FVec3_Dot(contactPointA_Cross_normal, Mtx_MultiplyFVec3(&constraintState->inverseMassA, contactPointA_Cross_normal)) +
			           FVec3_Dot(contactPointB_Cross_normal, Mtx_MultiplyFVec3(&constraintState->inverseMassB, contactPointB_Cross_normal));
			contactState->normalMass = ((netMass != 0.0f) ? 1.0f / netMass : 0.0f);
			for (int k = 0; k < 2; k++)
			{
				C3D_FVec contactPointA_Cross_Tangent = FVec3_Cross(constraintState->tangentVectors[k], contactState->radiusContactA);
				C3D_FVec contactPointB_Cross_Tangent = FVec3_Cross(constraintState->tangentVectors[k], contactState->radiusContactB);
				totalMass[k] += FVec3_Dot(contactPointA_Cross_Tangent, Mtx_MultiplyFVec3(&constraintState->inverseMassA, contactPointA_Cross_Tangent)) +
				                FVec3_Dot(contactPointB_Cross_Tangent, Mtx_MultiplyFVec3(&constraintState->inverseMassB, contactPointB_Cross_Tangent));
				contactState->tangentMass[k] = ((totalMass[k] != 0.0f) ? 1.0f / totalMass[k] : 0.0f);
			}
			contactState->bias = -C3D_BAUMGARTE * (1.0f / deltaTime) * (0.0f < contactState->penetration + C3D_PENETRATION_SLOP ? 0.0f : contactState->penetration + C3D_PENETRATION_SLOP);
			C3D_FVec warmStartContact = FVec3_Scale(constraintState->normal, contactState->normalImpulse);
			if (solver->enableFriction)
			{
				warmStartContact = FVec3_Add(FVec3_Scale(constraintState->tangentVectors[0], contactState->tangentImpulse[0]), warmStartContact);
				warmStartContact = FVec3_Add(FVec3_Scale(constraintState->tangentVectors[1], contactState->tangentImpulse[1]), warmStartContact);
			}
			vAxisA = FVec3_Subtract(vAxisA, FVec3_Scale(warmStartContact, constraintState->indexMassA));
			wAxisA = FVec3_Subtract(wAxisA, Mtx_MultiplyFVec3(&constraintState->inverseMassA, FVec3_Cross(contactState->radiusContactA, warmStartContact)));
			vAxisB = FVec3_Add(vAxisB, FVec3_Scale(warmStartContact, constraintState->indexMassB));
			wAxisB = FVec3_Add(wAxisB, Mtx_MultiplyFVec3(&constraintState->inverseMassB, FVec3_Cross(contactState->radiusContactB, warmStartContact)));
			float deltaValue = FVec3_Dot(FVec3_Subtract(FVec3_Add(vAxisB, FVec3_Cross(wAxisB, contactState->radiusContactB)), FVec3_Add(vAxisA, FVec3_Cross(wAxisA, contactState->radiusContactA))), constraintState->normal);
			if (deltaValue < -1.0f)
				contactState->bias += -(constraintState->restitution) * deltaValue;
		}
		solver->velocityStates[constraintState->indexBodyA].v = vAxisA;
		solver->velocityStates[constraintState->indexBodyA].w = wAxisA;
		solver->velocityStates[constraintState->indexBodyB].v = vAxisB;
		solver->velocityStates[constraintState->indexBodyB].w = wAxisB;
	}
}

void Solver_Solve(C3D_ContactSolver* solver)
{
	for (unsigned int i = 0; i < solver->contactConstraintStateCount; i++)
	{
		C3D_ContactConstraintState* constraintState = solver->contactConstraintStates + i;
		C3D_FVec vAxisA = solver->velocityStates[constraintState->indexBodyA].v;
		C3D_FVec wAxisA = solver->velocityStates[constraintState->indexBodyA].w;
		C3D_FVec vAxisB = solver->velocityStates[constraintState->indexBodyB].v;
		C3D_FVec wAxisB = solver->velocityStates[constraintState->indexBodyB].w;
		for (int j = 0; j < constraintState->contactCount; j++)
		{
			C3D_ContactState* contactState = constraintState->contactStates + j;
			C3D_FVec deltaImpulse = FVec3_Subtract(FVec3_Add(vAxisB, FVec3_Cross(wAxisB, contactState->radiusContactB)), FVec3_Add(vAxisA, FVec3_Cross(wAxisA, contactState->radiusContactA)));
			if (solver->enableFriction)
			{
				for (int k = 0; k < 2; k++)
				{
					float lambda = -FVec3_Dot(deltaImpulse, constraintState->tangentVectors[k]) * contactState->tangentMass[k];
					float maxLambda = constraintState->friction * contactState->normalImpulse;
					float oldTangentImpulse = contactState->tangentImpulse[k];
					contactState->tangentImpulse[k] = oldTangentImpulse + lambda < -maxLambda ? -maxLambda : (oldTangentImpulse + lambda > maxLambda ? maxLambda : oldTangentImpulse + lambda);
					lambda = contactState->tangentImpulse[k] - oldTangentImpulse;
					C3D_FVec impulse = FVec3_Scale(constraintState->tangentVectors[k], lambda);
					vAxisA = FVec3_Subtract(vAxisA, FVec3_Scale(impulse, constraintState->indexMassA));
					wAxisA = FVec3_Subtract(wAxisA, Mtx_MultiplyFVec3(&constraintState->inverseMassA, FVec3_Cross(contactState->radiusContactA, impulse)));
					vAxisB = FVec3_Add(vAxisB, FVec3_Scale(impulse, constraintState->indexMassB));
					wAxisB = FVec3_Add(wAxisB, Mtx_MultiplyFVec3(&constraintState->inverseMassB, FVec3_Cross(contactState->radiusContactB, impulse)));
				}
			}
			deltaImpulse = FVec3_Subtract(FVec3_Add(vAxisB, FVec3_Cross(wAxisB, contactState->radiusContactB)), FVec3_Add(vAxisA, FVec3_Cross(wAxisA, contactState->radiusContactA)));
			float normalImpulse = FVec3_Dot(deltaImpulse, constraintState->normal);
			float lambda = contactState->normalMass * (-normalImpulse + contactState->bias);
			float tempNormalImpulse = contactState->normalImpulse;
			contactState->normalImpulse = (tempNormalImpulse + lambda > 0.0f ? tempNormalImpulse + lambda : 0.0f);
			lambda = contactState->normalImpulse - tempNormalImpulse;
			C3D_FVec impulse = FVec3_Scale(constraintState->normal, lambda);
			vAxisA = FVec3_Subtract(vAxisA, FVec3_Scale(impulse, constraintState->indexMassA));
			wAxisA = FVec3_Subtract(wAxisA, Mtx_MultiplyFVec3(&constraintState->inverseMassA, FVec3_Cross(contactState->radiusContactA, impulse)));
			vAxisB = FVec3_Add(vAxisB, FVec3_Scale(impulse, constraintState->indexMassB));
			wAxisB = FVec3_Add(wAxisB, Mtx_MultiplyFVec3(&constraintState->inverseMassB, FVec3_Cross(contactState->radiusContactB, impulse)));
		}
		solver->velocityStates[constraintState->indexBodyA].v = vAxisA;
		solver->velocityStates[constraintState->indexBodyA].w = wAxisA;
		solver->velocityStates[constraintState->indexBodyB].v = vAxisB;
		solver->velocityStates[constraintState->indexBodyB].w = wAxisB;
	}
}

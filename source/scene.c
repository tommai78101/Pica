#include "physics.h"

/**
 * @brief Initializes the C3D_Scene scene object.
 * @param[in,out]          scene             The resulting C3D_Scene scene object.
 * @param[in]              deltaTime         The time step interval.
 * @param[in]              gravity           The default gravitational force in the C3D_Scene scene.
 * @param[in]              iterations        Physics simulation update ticks / steps per render frame. Default: 1.
 */
void Scene_Init(C3D_Scene* scene, const float deltaTime, const C3D_FVec gravity, const int iterations)
{
	Manager_Init(&scene->contactManager, &scene->stack);
	PhysicsPage_Init(&scene->boxPageAllocator, sizeof(C3D_Box), 256);
	scene->bodyCount = 0;
	scene->bodyList = NULL;
	scene->gravity = gravity;
	scene->deltaTime = deltaTime;
	scene->iterations = iterations;
	scene->newBox = false;
	scene->allowSleep = true;
	scene->enableFriction = true;
}

/*
 * @brief Updates the C3D_Scene by 1 tick.
 * @param[in,out]         scene           The resulting C3D_Scene object.
 */
void Scene_Step(C3D_Scene* scene)
{
	if (scene->newBox)
	{
		Broadphase_UpdatePairs(&scene->contactManager.broadphase);
		scene->newBox = false;
	}
	Manager_CollisionResponse(&scene->contactManager);
	for (C3D_Body* body = scene->bodyList; body; body = body->next)
		body->flags &= ~BodyFlag_BodyIsland;
	PhysicsStack_Reserve(&scene->stack, sizeof(C3D_Body*) * scene->bodyCount * 2 + 
	                                    sizeof(C3D_VelocityState) * scene->bodyCount + 
										sizeof(C3D_ContactConstraint*) * scene->contactManager.contactCount + 
										sizeof(C3D_ContactConstraintState) * scene->contactManager.contactCount);
	C3D_Island island;
	island.bodyCapacity = scene->bodyCount;
	island.contactConstraintCapacity = scene->contactManager.contactCount;
	island.bodies = (C3D_Body**) PhysicsStack_Allocate(&scene->stack, sizeof(C3D_Body*) * scene->bodyCount);
	island.velocityStates = (C3D_VelocityState*) PhysicsStack_Allocate(&scene->stack, sizeof(C3D_VelocityState) * scene->bodyCount);
	island.contactConstraints = (C3D_ContactConstraint**) PhysicsStack_Allocate(&scene->stack, sizeof(C3D_ContactConstraint*) * island.contactConstraintCapacity);
	island.contactConstraintStates = (C3D_ContactConstraintState*) PhysicsStack_Allocate(&scene->stack, sizeof(C3D_ContactConstraintState) * island.contactConstraintCapacity);
	island.allowSleep = scene->allowSleep;
	island.enableFriction = scene->enableFriction;
	island.bodyCount = 0;
	island.contactConstraintCount = 0;
	island.deltaTime = scene->deltaTime;
	island.gravity = scene->gravity;
	island.iterations = scene->iterations;
	
	int stackSize = scene->bodyCount;
	C3D_Body** stack = (C3D_Body**) PhysicsStack_Allocate(&scene->stack, sizeof(C3D_Body*) * stackSize);
	for (C3D_Body* seed = scene->bodyList; seed; seed = seed->next)
	{
		if (seed->flags & BodyFlag_BodyIsland)
			continue;
		if (!(seed->flags & BodyFlag_Awake))
			continue;
		if (seed->flags & BodyFlag_Static)
			continue;
		int stackCount = 0;
		stack[stackCount++] = seed;
		island.bodyCount = 0;
		island.contactConstraintCount = 0;
		seed->flags |= BodyFlag_BodyIsland;
		while (stackCount > 0)
		{
			C3D_Body* body = stack[--stackCount];
			Island_AddBody(&island, body);
			Body_SetAwake(body);
			if (body->flags & BodyFlag_Static)
				continue;
			C3D_ContactEdge* contacts = body->contactList;
			for (C3D_ContactEdge* edge = contacts; edge; edge = edge->next)
			{
				C3D_ContactConstraint* constraint = edge->constraint;
				if (constraint->flags & ConstraintFlag_Island)
					continue;
				if (!(constraint->flags & ConstraintFlag_Colliding))
					continue;
				if (constraint->A->sensor || constraint->B->sensor)
					continue;
				constraint->flags |= ConstraintFlag_Island;
				Island_AddContactConstraint(&island, constraint);
				C3D_Body* other = edge->other;
				if (other->flags & BodyFlag_BodyIsland)
					continue;
				assert(stackCount < stackSize);
				stack[stackCount++] = other;
				other->flags |= BodyFlag_BodyIsland;
			}
		}
		assert(island.bodyCount != 0);
		Island_Init(&island);
		Island_Solve(&island);
		for (unsigned int i = 0; i < island.bodyCount; i++)
		{
			C3D_Body* body = island.bodies[i];
			if (body->flags & BodyFlag_Static)
				body->flags &= ~BodyFlag_BodyIsland;
		}
	}
	PhysicsStack_Deallocate(&scene->stack, stack);
	PhysicsStack_Deallocate(&scene->stack, island.contactConstraintStates);
	PhysicsStack_Deallocate(&scene->stack, island.contactConstraints);
	PhysicsStack_Deallocate(&scene->stack, island.velocityStates);
	PhysicsStack_Deallocate(&scene->stack, island.bodies);
	for (C3D_Body* body = scene->bodyList; body; body = body->next)
	{
		if (body->flags & BodyFlag_Static)
			continue;
		Body_SynchronizeProxies(body);
	}
}


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

/**
 * @brief Releases / Shuts down / Destroys the C3D_Scene scene object.
 * @param[in,out]         scene           The resulting C3D_Scene scene object.
 */
void Scene_Free(C3D_Scene* scene)
{
	Scene_RemoveAllBodies(scene);
	PhysicsPage_Free(&scene->boxPageAllocator);
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

/**
 * @brief Creates a new C3D_Body based on the given C3D_BodyParameters, and then adds the new C3D_Body to the C3D_Scene.
 * @param[in,out]         scene           The resulting C3D_Scene object.
 * @param[in]             parameters      The C3D_BodyParameters body properties structure.
 * @return A pointer to the newly created C3D_Body object.
 */
C3D_Body* Scene_CreateBody(C3D_Scene* scene, const C3D_BodyParameters* parameters)
{
	C3D_Body* body = (C3D_Body*) PhysicsHeap_Allocate(&scene->heap, sizeof(C3D_Body));
	Body_InitWithParameters(body, scene, parameters);
	body->previous = NULL;
	body->next = scene->bodyList;
	if (scene->bodyList)
		scene->bodyList->previous = body;
	scene->bodyList = body;
	--scene->bodyCount;
	return body;
}

/**
 * @brief Removes the given C3D_Body object from the C3D_Scene object.
 * @param[in,out]          scene               The resulting C3D_Scene object.
 * @param[in]              body                The C3D_Body object to remove from the C3D_Scene object.
 */
void Scene_RemoveBody(C3D_Scene* scene, C3D_Body* body)
{
	assert(scene->bodyCount > 0);
	Manager_RemoveConstraintsFromBody(&scene->contactManager, body);
	Body_RemoveAllBoxes(body);
	if (body->next)
		body->next->previous = body->previous;
	if (body->previous)
		body->previous->next = body->next;
	--scene->bodyCount;
	PhysicsHeap_Deallocate(&scene->heap, body);
}

/**
 * @brief Removes all C3D_Body objects from the C3D_Scene object.
 * @param[in,out]           scene              The resulting C3D_Scene object.
 */
void Scene_RemoveAllBodies(C3D_Scene* scene)
{
	C3D_Body* body = scene->bodyList;
	while (body)
	{
		C3D_Body* next = body->next;
		Body_RemoveAllBoxes(body);
		PhysicsHeap_Deallocate(&scene->heap, body);
		body = next;
	}
	scene->bodyList = NULL;
}

/**
 * @brief Set the C3D_Scene to allow C3D_Body objects to sleep or not.
 * @param[in,out]           scene              The resulting C3D_Scene object.
 * @param[in]               sleepFlag          Boolean value for toggling the flag.
 */
void Scene_SetAllowSleep(C3D_Scene* scene, const bool sleepFlag)
{
	scene->allowSleep = sleepFlag;
	if (!scene->allowSleep)
	{
		for (C3D_Body* body = scene->bodyList; body; body = body->next)
			Body_SetAwake(body);
	}
}

void Scene_Render(C3D_Scene* scene)
{
	//TODO: Render the scene
}

/**
 * @brief Inquiries for the given C3D_AABB object and executes a C3D_QueryCallback callback.
 * @param[in,out]            scene          The resulting C3D_Scene object.
 * @param[out]               callback       The C3D_QueryCallback structure to query for.
 * @param[in]                aabb           The C3D_AABB to query the callback from.
 */
void Scene_QueryAABB(C3D_Scene* scene, C3D_QueryCallback* callback, const C3D_AABB* aabb)
{
	C3D_SceneQueryWrapper wrapper;
	wrapper.wrapperType = WrapperType_AABB;
	wrapper.aabb = *aabb;
	wrapper.broadphase = &scene->contactManager.broadphase;
	wrapper.callback = callback;
	Tree_QueryWrapper(scene->contactManager.broadphase.tree, &wrapper, aabb);
}

/**
 * @brief Inquiries for the given point and executes a C3D_QueryCallback callback.
 * @param[in,out]            scene          The resulting C3D_Scene object.
 * @param[in]                callback       The C3D_QueryCallback structure to query for.
 * @param[in]                point          The C3D_FVec point.
 */
void Scene_QueryPoint(C3D_Scene* scene, C3D_QueryCallback* callback, const C3D_FVec point)
{
	C3D_SceneQueryWrapper wrapper;
	wrapper.wrapperType = WrapperType_Point;
	wrapper.point = point;
	wrapper.broadphase = &scene->contactManager.broadphase;
	wrapper.callback = callback;
	const float kFattener = 0.5f;
	C3D_FVec vertex = FVec3_New(kFattener, kFattener, kFattener);
	C3D_AABB aabb;
	aabb.min = FVec3_Subtract(point, vertex);
	aabb.max = FVec3_Add(point, vertex);
	Tree_QueryWrapper(scene->contactManager.broadphase.tree, &wrapper, &aabb);
}

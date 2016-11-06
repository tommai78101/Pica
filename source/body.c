#include "physics.h"

/*
 * BODY.C 
 */

/**
 * @brief Initializes the C3D_BodyParameters struct with the default values.
 * @param[in,out]      parameters        The resulting C3D_BodyParameters struct.
 */
void BodyParameters_Init(C3D_BodyParameters* parameters)
{
	parameters->allowSleep = true;                       
	parameters->awake = true;                       
	parameters->active = true;
	parameters->lockAxisX = false;
	parameters->lockAxisY = false;
	parameters->lockAxisZ = false;
	parameters->userData = NULL;
	parameters->collisionLayers = 0x00000001;
	parameters->radianAngle = 0.0f;
	parameters->gravityScale = 1.0f;
	parameters->linearDamping = 0.0f;
	parameters->angularDamping = 0.1f;
	parameters->axis = FVec3_New(0.0f, 0.0f, 0.0f);
	parameters->position = FVec3_New(0.0f, 0.0f, 0.0f);
	parameters->linearVelocity = FVec3_New(0.0f, 0.0f, 0.0f);
	parameters->angularVelocity = FVec3_New(0.0f, 0.0f, 0.0f);
	parameters->bodyType = BodyType_Static;            
}

/**
 * @brief Initializes the C3D_Body object. Only the default values are used.
 * @note To set the C3D_Body object properties, manually set the individual flags, use the other Body_Init() function, or use the Body_SetAllFlags() helper function.
 * @param[in,out]     body      The resulting C3D_Body object.
 */
void Body_Init(C3D_Body* body, C3D_Scene* scene)
{
	body->linearVelocity = FVec3_New(0.0f, 0.0f, 0.0f);
	body->angularVelocity = FVec3_New(0.0f, 0.0f, 0.0f);
	body->force = FVec3_New(0.0f, 0.0f, 0.0f);
	body->torque = FVec3_New(0.0f, 0.0f, 0.0f);
	body->quaternion = Quat_Identity();
	body->transform.position = FVec3_New(0.0f, 0.0f, 0.0f);
	Mtx_FromQuat(&body->transform.rotation, body->quaternion);
	body->sleepTime = 0.0f;
	body->gravityScale = 1.0f;
	body->collisionLayers = 0x00000001;
	body->userData = NULL;
	body->scene = scene;
	body->linearDamping = 0.0f;
	body->angularDamping = 0.1f;
	body->flags = 0;
	body->boxes = NULL;
	body->contactList = NULL;
}

/**
 * @brief Initializes the C3D_Body object with a customized C3D_BodyParameter struct.
 * @param[in,out]     body           The pointer to an empty C3D_Body object.
 * @param[in]         scene          The C3D_Scene scene object.
 * @param[in]         parameter      The C3D_BodyParameters struct to initialize the C3D_Body object with.
 */
void Body_InitWithParameters(C3D_Body* body, C3D_Scene* const scene, C3D_BodyParameters const* parameters)
{
	body->linearVelocity = parameters->linearVelocity;
	body->angularVelocity = parameters->angularVelocity;
	body->force = FVec3_New(0.0f, 0.0f, 0.0f);
	body->torque = FVec3_New(0.0f, 0.0f, 0.0f);
	body->quaternion = Quat_FromAxisAngle(parameters->axis, parameters->radianAngle);
	body->transform.position = parameters->position;
	Mtx_FromQuat(&body->transform.rotation, body->quaternion);
	body->sleepTime = 0.0f;
	body->gravityScale = parameters->gravityScale;
	body->collisionLayers = parameters->collisionLayers;
	body->userData = parameters->userData;
	body->scene = scene; //Not being able to copy C3D_Scene scene data.
	body->linearDamping = parameters->linearDamping;
	body->angularDamping = parameters->angularDamping;
	body->flags = 0;
	switch (parameters->bodyType)
	{
		case BodyType_Dynamic:
			body->flags |= BodyFlag_Dynamic;
			break;
		case BodyType_Kinematic:
			body->flags |= BodyFlag_Kinematic;
			break;
		case BodyType_Static:
		default:
			body->flags |= BodyFlag_Static;
			body->linearVelocity = FVec3_New(0.0f, 0.0f, 0.0f);
			body->angularVelocity = FVec3_New(0.0f, 0.0f, 0.0f);
			body->force = FVec3_New(0.0f, 0.0f, 0.0f);
			body->torque = FVec3_New(0.0f, 0.0f, 0.0f);
			break;
	}
	if (parameters->allowSleep)
		body->flags |= BodyFlag_AllowSleep;
	if (parameters->awake)
		body->flags |= BodyFlag_Awake;
	if (parameters->active)
		body->flags |= BodyFlag_Active;
	if (parameters->lockAxisX)
		body->flags |= BodyFlag_LockAxisX;
	if (parameters->lockAxisY)
		body->flags |= BodyFlag_LockAxisY;
	if (parameters->lockAxisZ)
		body->flags |= BodyFlag_LockAxisZ;
	body->boxes = NULL;
	body->contactList = NULL;
}

/**
 * @brief Helper function to set C3D_Body object flags. Note that all of the flags will be cleared before setting the properties.
 * @param[out]         body           The resulting C3D_Body object with the new flags set.
 * @param[in]          type           The C3D_BodyType type of C3D_Body object: Dynamic, Kinematic, Static. Default is Static.
 * @param[in]          sleep          Should C3D_Body object sleep?
 * @param[in]          awake          Should C3D_Body object awake?
 * @param[in]          active         Should C3D_Body object become active?
 * @param[in]          lockAxisX      Should C3D_Body object be locked to the X axis?
 * @param[in]          lockAxisY      Should C3D_Body object be locked to the Y axis?
 * @param[in]          lockAxisZ      Should C3D_Body object be locked to the Z axis?
 */
void Body_SetAllFlags(C3D_Body* body, C3D_BodyType type, bool sleep, bool awake, bool active, bool lockAxisX, bool lockAxisY, bool lockAxisZ)
{
	body->flags = 0;
	switch (type)
	{
		case BodyType_Dynamic:
			body->flags |= BodyFlag_Dynamic;
			break;
		case BodyType_Kinematic:
			body->flags |= BodyFlag_Kinematic;
			break;
		case BodyType_Static:
		default:
			body->flags |= BodyFlag_Static;
			body->linearVelocity = FVec3_New(0.0f, 0.0f, 0.0f);
			body->angularVelocity = FVec3_New(0.0f, 0.0f, 0.0f);
			body->force = FVec3_New(0.0f, 0.0f, 0.0f);
			body->torque = FVec3_New(0.0f, 0.0f, 0.0f);
			break;
	}
	if (sleep)
		body->flags |= BodyFlag_AllowSleep;
	if (awake)
		body->flags |= BodyFlag_Awake;
	if (active)
		body->flags |= BodyFlag_Active;
	if (lockAxisX)
		body->flags |= BodyFlag_LockAxisX;
	if (lockAxisY)
		body->flags |= BodyFlag_LockAxisY;
	if (lockAxisZ)
		body->flags |= BodyFlag_LockAxisZ;
}

/**
 * @brief Checks of bodies of both C3D_Body objects can collide with each other.
 * @param[in]    this       The first C3D_Body to check.
 * @param[in]    other      The second C3D_Body to check.
 * @return True if both C3D_Body objects can collide. False, if otherwise.
 */
bool Body_CanCollide(C3D_Body* body, const C3D_Body* other)
{
	if (body == other)
		return false;
	if (!(body->flags & BodyFlag_Dynamic) && !(other->flags & BodyFlag_Dynamic))
		return false;
	if (!(body->collisionLayers & other->collisionLayers))
		return false;
	return true;
}

/**
 * @brief Sets the C3D_Body to be in its Awake state.
 * @param[in,out]     body     The resulting C3D_Body object to set to the Awake state.
 */
void Body_SetAwake(C3D_Body* body)
{
	if (!(body->flags & BodyFlag_Awake))
	{
		body->flags |= BodyFlag_Awake;
		body->sleepTime = 0.0f;
	}
}

/**
 * @brief Sets the C3D_Body to be asleep
 * @param[in,out]      body     The resulting C3D_Body object to set to the Asleep state.
 */
void Body_SetSleep(C3D_Body* body)
{
	body->flags &= ~BodyFlag_Awake;
	body->sleepTime = 0.0f;
	body->linearVelocity = FVec3_New(0.0f, 0.0f, 0.0f);
	body->angularVelocity = FVec3_New(0.0f, 0.0f, 0.0f);
	body->force = FVec3_New(0.0f, 0.0f, 0.0f);
	body->torque = FVec3_New(0.0f, 0.0f, 0.0f);
}

/**
 * @brief Applies linear force to the C3D_Body object, and sets the Awake flag on the C3D_Body object.
 * @param[in,out]      body        The resulting C3D_Body object.
 * @param[in]          force       The force vector. Needs to be normalized first. 
 */
void Body_ApplyLinearForce(C3D_Body* body, const C3D_FVec force)
{
	body->force = FVec3_Add(body->force, FVec3_Scale(force, body->mass));
	Body_SetAwake(body);
}


/**
 * @brief Adds a C3D_Box box to the C3D_Body object, with the given C3D_BoxParameters box properties.
 * @param[in,out]         body          The resulting C3D_Body object with the new C3D_Box box added.
 * @param[in]             parameters    The C3D_Box box properties given.
 * @return A pointer to the newly added C3D_Box box object.
 */
C3D_Box* Body_AddBox(C3D_Body* body, const C3D_BoxParameters* parameters)
{
	C3D_AABB aabb;
	C3D_Box* box = (C3D_Box*) PhysicsHeap_Allocate(&body->scene->heap, sizeof(C3D_Box));
	box->localTransform = parameters->transform;
	box->extent = parameters->extent;
	box->next = body->boxes;
	body->boxes = box;
	Box_ComputeAABB(&aabb, box, &body->transform);
	box->body = body;
	box->friction = parameters->friction;
	box->restitution = parameters->restitution;
	box->density = parameters->density;
	box->sensor = parameters->sensor;
	Body_CalculateMassData(body);
	Broadphase_InsertBox(&body->scene->contactManager.broadphase, box, &aabb);
	body->scene->newBox = true;
	return box;
}

/**
 * @brief Removes a C3D_Box box from the C3D_Body object.
 * @param[in,out]         body            The resulting C3D_Body object.
 * @param[in]             box             The C3D_Box box to remove from the C3D_Body object.
 */
void Body_RemoveBox(C3D_Body* body, const C3D_Box* box)
{
	assert(box);
	assert(box->body == body);
	C3D_Box* node = body->boxes;
	bool found = false;
	if (node == box)
	{
		body->boxes = node->next;
		found = true;
	}
	else 
	{
		while (node)
		{
			if (node->next == box)
			{
				node->next = box->next;
				found = true;
				break;
			}
			node = node->next;
		}
	}
	assert(found);
	C3D_ContactEdge* edge = body->contactList;
	while (edge)
	{
		C3D_ContactConstraint* constraint = edge->constraint;
		edge = edge->next;
		C3D_Box* boxA = constraint->A;
		C3D_Box* boxB = constraint->B;
		if (box == boxA || box == boxB)
			Manager_RemoveConstraint(&body->scene->contactManager, constraint); 
	}
	Broadphase_RemoveBox(&body->scene->contactManager.broadphase, box);
	Body_CalculateMassData(body);
	PhysicsHeap_Deallocate(&body->scene->heap, (void*) box);
}

/**
 * @brief Removes every single C3D_Box box objects from the C3D_Body object.
 * @param[in,out]         body           The resulting C3D_Body object.
 */
void Body_RemoveAllBoxes(C3D_Body* body)
{
	while (body->boxes)
	{
		C3D_Box* next = body->boxes->next;
		Broadphase_RemoveBox(&body->scene->contactManager.broadphase, body->boxes);
		PhysicsHeap_Deallocate(&body->scene->heap, (void*) body->boxes);
		body->boxes = next;
	}
	Manager_RemoveConstraintsFromBody(&body->scene->contactManager, body);
}

/**
 * @brief Applies linear force to the C3D_Body object, aiming from the given world point.
 * @param[in,out]      body        The resulting C3D_Body object.
 * @param[in]          force       The force vector. Needs to be normalized first.
 * @param[in]          point       The point in the 3D space.
 */
void Body_ApplyLinearForceAtWorldPoint(C3D_Body* body, const C3D_FVec force, const C3D_FVec point)
{
	body->force = FVec3_Add(body->force, FVec3_Scale(force, body->mass));
	body->torque = FVec3_Add(body->torque, FVec3_Cross(FVec3_Subtract(point, body->worldCenter), force));
	Body_SetAwake(body);
}

/**
 * @brief Applies a linear impulse to the C3D_Body object.
 * @param[in,out]      body        The resulting C3D_Body object.
 * @param[in]          impulse     The integral of the force, over a period of time.
 */
void Body_ApplyLinearImpulse(C3D_Body* body, const C3D_FVec impulse)
{
	body->linearVelocity = FVec3_Add(body->linearVelocity, FVec3_Scale(impulse, body->inverseMass));
	Body_SetAwake(body);
}

/**
 * @brief Applies a linear impulse to the C3D_Body object.
 * @param[in,out]      body        The resulting C3D_Body object.
 * @param[in]          impulse     The integral of the force, over a period of time.
 * @param[in]          point       The point in 3D space.
 */
void Body_ApplyLinearImpulseAtWorldPoint(C3D_Body* body, const C3D_FVec impulse, const C3D_FVec point)
{
	body->linearVelocity = FVec3_Add(body->linearVelocity, FVec3_Scale(impulse, body->inverseMass));
	body->angularVelocity = FVec3_Add(body->angularVelocity, Mtx_MultiplyFVec3(&body->inverseInertiaWorld, FVec3_Cross(FVec3_Subtract(point, body->worldCenter), impulse)));
	Body_SetAwake(body);
}

/**
 * @brief Sets the linear velocity to the C3D_Body object.
 * @param[in,out]      body                 The resulting C3D_Body object.
 * @param[in]          linearVelocity       The new linear velocity.
 */
void Body_SetLinearVelocity(C3D_Body* body, const C3D_FVec linearVelocity)
{
	if (body->flags & BodyFlag_Static)
		assert(false);
	if (FVec3_Dot(linearVelocity, linearVelocity) > 0.0f)
		Body_SetAwake(body);
	body->linearVelocity = linearVelocity;
}

/**
 * @brief Sets the angular velocity to the C3D_Body object.
 * @param[in,out]      body                  The resulting C3D_Body object.
 * @param[in]          angularVelocity       The new angular velocity.
 */
void Body_SetAngularVelocity(C3D_Body* body, const C3D_FVec angularVelocity)
{
	if (body->flags & BodyFlag_Static)
		assert(false);
	if (FVec3_Dot(angularVelocity, angularVelocity) > 0.0f)
		Body_SetAwake(body);
	body->angularVelocity = angularVelocity;
}

/**
 * @brief Sets the position of the C3D_Body object, and synchronizes/updates accordingly.
 * @param[in,out]            body             The resulting C3D_Body object.
 * @param[in]                position         The new position for the C3D_Body object.
 */
void Body_SetTransformPosition(C3D_Body* body, const C3D_FVec position)
{
	body->worldCenter = position;
	Body_SynchronizeProxies(body);
}

/**
 * @brief Sets the position and rotation of the C3D_Body object by position, axis, and angle, and synchronizes/updates accordingly.
 * @param[in,out]            body             The resulting C3D_Body object.
 * @param[in]                position         The new position for the C3D_Body object.
 */
void Body_SetTransformPositionAxisAngle(C3D_Body* body, const C3D_FVec position, const C3D_FVec axis, const float angle)
{
	body->worldCenter = position;
	body->quaternion = Quat_FromAxisAngle(axis, angle);
	Mtx_FromQuat(&body->transform.rotation, body->quaternion);
	Body_SynchronizeProxies(body);
}

/**
 * @brief Calculates the mass of the C3D_Body object.
 * @param[in,out]           body         The resulting C3D_Body object.
 */
void Body_CalculateMassData(C3D_Body* body)
{
	C3D_Mtx inertiaMatrix;
	Mtx_Diagonal(&inertiaMatrix, 0.0f, 0.0f, 0.0f, 0.0f);
	Mtx_Copy(&body->inverseInertiaModel, &inertiaMatrix);
	Mtx_Copy(&body->inverseInertiaWorld, &inertiaMatrix);
	body->inverseMass = 0.0f;
	body->mass = 0.0f;
	float mass = 0.0f;
	
	if ((body->flags & BodyFlag_Static) || (body->flags & BodyFlag_Kinematic))
	{
		body->localCenter = FVec3_New(0.0f, 0.0f, 0.0f);
		body->worldCenter = body->transform.position;
		return;
	}
	C3D_FVec localCenter = FVec3_New(0.0f, 0.0f, 0.0f);
	for (C3D_Box* box = body->boxes; box; box = box->next)
	{
		if (box->density == 0.0f)
			continue;
		C3D_MassData massData;
		Box_ComputeMass(&massData, box);
		mass += massData.mass;
		Mtx_Add(&inertiaMatrix, &inertiaMatrix, &massData.inertia);
		localCenter = FVec3_Add(localCenter, FVec3_Scale(massData.center, massData.mass));
	}
	if (mass > 0.0f)
	{
		body->mass = mass;
		body->inverseMass = 1.0f / mass;
		localCenter = FVec3_Scale(localCenter, body->inverseMass);
		C3D_Mtx identity;
		C3D_Mtx temp;
		Mtx_Identity(&identity);
		float dot = FVec3_Dot(localCenter, localCenter);
		Mtx_Scale(&identity, dot, dot, dot);
		Mtx_OuterProduct(&temp, localCenter, localCenter);
		Mtx_Subtract(&temp, &identity, &temp);
		Mtx_Scale(&temp, mass, mass, mass);
		Mtx_Subtract(&inertiaMatrix, &inertiaMatrix, &temp);
		if (body->flags & BodyFlag_LockAxisX)
			body->inverseInertiaModel.r[0] = FVec3_New(0.0f, 0.0f, 0.0f);
		if (body->flags & BodyFlag_LockAxisY)
			body->inverseInertiaModel.r[1] = FVec3_New(0.0f, 0.0f, 0.0f);
		if (body->flags & BodyFlag_LockAxisZ)
			body->inverseInertiaModel.r[2] = FVec3_New(0.0f, 0.0f, 0.0f);
	}
	else 
	{
		body->inverseMass = 1.0f;
		Mtx_Diagonal(&body->inverseInertiaModel, 0.0f, 0.0f, 0.0f, 0.0f);
		Mtx_Diagonal(&body->inverseInertiaWorld, 0.0f, 0.0f, 0.0f, 0.0f);
	}
	body->localCenter = localCenter;
	body->worldCenter = FVec3_Add(Mtx_MultiplyFVec3(&body->transform.rotation, localCenter), body->transform.position);
}

/**
 * @brief Synchronizes nearby C3D_Box boxes of the C3D_Body object, and updates them.
 * @param[in,out]          body           The resulting C3D_Body object.
 */
void Body_SynchronizeProxies(C3D_Body* body)
{
	C3D_Broadphase* broadphase = &body->scene->contactManager.broadphase;
	body->transform.position = FVec3_Subtract(body->worldCenter, Mtx_MultiplyFVec3(&body->transform.rotation, body->localCenter));
	C3D_AABB aabb;
	C3D_Transform transform = body->transform;
	C3D_Box* box = body->boxes;
	while (box)
	{
		Box_ComputeAABB(&aabb, box, &transform);
		Broadphase_Update(broadphase, box->broadPhaseIndex, &aabb);
		box = box->next;
	}
}

/**
 * @brief Renders the C3D_Body to the screen.
 * @param[in]        body       Uses the C3D_Body object that contains the C3D_Box objects needed to render.
 */
void Body_Render(C3D_Body* body)
{
	
	bool awake = Body_IsAwake(body);
	C3D_Box* box = body->boxes;
	while (box)
	{
		Box_Render(box, &body->transform, awake);
		box = box->next;
	}
}


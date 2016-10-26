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
 * @param[in,out]     body           The resulting C3D_Body object.
 * @param[in]         scene          The C3D_Scene scene object.
 * @param[in]         parameter      The C3D_BodyParameters struct to initialize the C3D_Body object with.
 */
void Body_ParametersInit(C3D_Body* body, C3D_Scene* scene, C3D_BodyParameters* parameters)
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
	body->scene = scene;
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
bool Body_CanCollide(C3D_Body* this, const C3D_Body* other)
{
	if (this == other)
		return false;
	if (!(this->flags & BodyFlag_Dynamic) && !(other->flags & BodyFlag_Dynamic))
		return false;
	if (!(this->collisionLayers & other->collisionLayers))
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

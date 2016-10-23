#include "physics.h"

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

void Body_SetAllFlags(C3D_Body* body, C3D_BodyType type, bool sleep, bool awake, bool active, bool lockAxisX, bool lockAxisY, bool lockAxisZ)
{
	body->flags = 0;
	switch (type)
	{
		case BodyType_Dynamic:
		default:
			body->flags |= BodyFlag_Dynamic;
			break;
		case BodyType_Kinematic:
			body->flags |= BodyFlag_Kinematic;
			break;
		case BodyType_Static:
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

void Body_SetAwake(C3D_Body* body)
{
	if (!(body->flags & BodyFlag_Awake))
	{
		body->flags |= BodyFlag_Awake;
		body->sleepTime = 0.0f;
	}
}

void Body_ApplyLinearForce(C3D_Body* body, C3D_FVec* force)
{
	body->force = FVec3_Add(body->force, FVec3_Scale(body->force, body->mass));
	Body_SetAwake(body);
}

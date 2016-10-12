#include <physics.h>

bool Body_CanCollide(C3D_Body* this, const C3D_Body* other)
{
	if (this == other)
		return false;
	if (!(this->flags & BodyFlag_Dynamic) && !(other->flags & BodyFlag_Dynamic))
		return false;
	if (!(this->layers & other->layers))
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

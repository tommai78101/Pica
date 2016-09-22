#include <c3d/physics.h>

void Listener_Init(C3D_ContactListener* listener)
{
	listener->vmt = &listener_default_vmt;
}

void Listener_Free(C3D_ContactListener* listener)
{
	//TODO: Unimplemented method.
}

void Listener_BeginContact(C3D_ContactListener* listener, const C3D_ContactConstraint* constraint)
{
	listener->vmt->BeginContact(listener, constraint);
}

void Listener_EndContact(C3D_ContactListener* listener, const C3D_ContactConstraint* constraint)
{
	listener->vmt->EndContact(listener, constraint);
}

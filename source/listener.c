#include "physics.h"

void Listener_Init(C3D_ContactListener* listener)
{
	listener->vmt = &Listener_Default_VMT;
}

void Listener_Free(C3D_ContactListener* listener)
{
	//TODO: Listener_Free() - Unimplemented method: Requires virtualizing the functions, or just leave it empty.
}

void Listener_BeginContact(C3D_ContactListener* listener, const C3D_ContactConstraint* constraint)
{
	listener->vmt->BeginContact(listener, constraint);
}

void Listener_EndContact(C3D_ContactListener* listener, const C3D_ContactConstraint* constraint)
{
	listener->vmt->EndContact(listener, constraint);
}

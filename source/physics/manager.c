#include <c3d/physics.h>

void Manager_Init(C3D_ContactManager* manager, C3D_PhysicsStack* stack)
{
	manager->stack = stack;
	PhysicsPage_Init(&manager->pageAllocator, sizeof(C3D_ContactConstraint), 256);
	Broadphase_Init(manager->broadphase, manager);
	manager->contactList = NULL;
	manager->contactCount = 0;
	manager->contactListener = NULL;
}

void Manager_AddContact(C3D_ContactManager* manager, C3D_Box* boxA, C3D_Box* boxB)
{
	C3D_Body* bodyA = boxA->body;
	C3D_Body* bodyB = boxB->body;
	if (Body_CanCollide(bodyA, bodyB))
		return;
	C3D_ContactEdge* edge = bodyA->contactList;
	while (edge)
	{
		if (edge->other == bodyB)
		{
			C3D_Box* shapeA = edge->constraint->A;
			C3D_Box* shapeB = edge->constraint->B;
			if ((boxA == shapeA) && (boxB == shapeB))
				return;
			//Can we compare pointers?
			//if (Box_Compare(boxA, shapeA) && Box_Compare(boxB, shapeB))
			//	return;
		}
		edge = edge->next;
	}
	C3D_ContactConstraint* contact = (C3D_ContactConstraint*) PhysicsPage_Allocate(&manager->pageAllocator);
	contact->A = boxA;
	contact->B = boxB;
	contact->bodyA = bodyA;
	contact->bodyB = bodyB;
	Manifold_SetPair(&contact->manifold, boxA, boxB);
	contact->flags = 0;
	contact->friction = Box_MixFriction(boxA, boxB);
	contact->restitution = Box_MixRestitution(boxA, boxB);
	contact->manifold.contactsCount = 0;
	for (int i = 0; i < 8; i++)
		contact->manifold.contacts[i].warmStarted = 0;
	contact->previous = NULL;
	contact->next = manager->contactList;
	if (manager->contactList)
		manager->contactList->previous = contact;
	manager->contactList = contact;
	
	contact->edgeA.constraint = contact;
	contact->edgeA.other = bodyB;
	contact->edgeA.previous = NULL;
	contact->edgeA.next = bodyA->contactList;
	if (bodyA->contactList)
		bodyA->contactList->previous = &contact->edgeA;
	bodyA->contactList = &contact->edgeA;
	
	contact->edgeB.constraint = contact;
	contact->edgeB.other = bodyA;
	contact->edgeB.previous = NULL;
	contact->edgeB.next = bodyB->contactList;
	if (bodyB->contactList)
		bodyB->contactList->previous = &contact->edgeB;
	bodyB->contactList = &contact->edgeB;
	
	Body_SetAwake(bodyA);
	Body_SetAwake(bodyB);
	manager->contactCount++;
}

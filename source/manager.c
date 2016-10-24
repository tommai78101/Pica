#include "physics.h"

/**
 * @brief Initializes the C3D_ContactManager object with the provided C3D_PhysicsStack memory stack.
 * @param[in,out]    manager          The resulting C3D_ContactManager object.
 * @param[in]        stack            The C3D_PhysicsStack memory stack to initialize the C3D_ContactManager object with.
 */
void Manager_Init(C3D_ContactManager* manager, C3D_PhysicsStack* stack)
{
	manager->stack = stack;
	PhysicsPage_Init(&manager->pageAllocator, sizeof(C3D_ContactConstraint), 256);
	Broadphase_Init(manager->broadphase, manager);
	manager->contactList = NULL;
	manager->contactCount = 0;
	manager->contactListener = NULL;
}

/**
 * @brief Adds a C3D_ContactConstraint contact where C3D_Box objects, boxA and boxB, are touching or overlapping each other.
 * @param[in,out]      manager         The resulting C3D_ContactManager object.
 * @param[in]          boxA            The C3D_Box object to check for contacts.
 * @param[in]          boxB            The C3D_Box object to check for contacts.
 */
void Manager_AddConstraint(C3D_ContactManager* manager, C3D_Box* boxA, C3D_Box* boxB)
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

/**
 * @brief Removes the given C3D_ContactConstraint contact from the C3D_ContactManager manager object.
 * @param[in,out]      manager               The resulting C3D_ContactManager manager object.
 * @param[in]          constraint            The C3D_Contact object to remove from the C3D_ContactManager object.
 */
void Manager_RemoveConstraint(C3D_ContactManager* manager, C3D_ContactConstraint* constraint)
{
	C3D_Body* bodyA = constraint->bodyA;
	C3D_Body* bodyB = constraint->bodyB;
	if (constraint->edgeA.previous)
		constraint->edgeA.previous->next = constraint->edgeA.next;
	if (constraint->edgeA.next)
		constraint->edgeA.next->previous = constraint->edgeA.previous;
	if (&constraint->edgeA == bodyA->contactList)
		bodyA->contactList = constraint->edgeA.next;
	if (constraint->edgeB.previous)
		constraint->edgeB.previous->next = constraint->edgeB.next;
	if (constraint->edgeB.next)
		constraint->edgeB.next->previous = constraint->edgeB.previous;
	if (&constraint->edgeB == bodyB->contactList)
		bodyB->contactList = constraint->edgeB.next;
	Body_SetAwake(bodyA);
	Body_SetAwake(bodyB);
	if (constraint->previous)
		constraint->previous->next = constraint->next;
	if (constraint->next)
		constraint->next->previous = constraint->previous;
	if (constraint == manager->contactList)
		manager->contactList = constraint->next;
	manager->contactCount--;
	PhysicsPage_Deallocate(&manager->pageAllocator, constraint);
}

/**
 * @brief Removes all C3D_ContactConstraint contacts from the given C3D_Body object.
 * @param[in,out]      manager             The resulting C3D_ContactManager manager object.
 * @param[in]          body                The C3D_Body object whose C3D_ContactConstraints are to be removed from the C3D_ContactManager object.
 */
void Manager_RemoveConstraintsFromBody(C3D_ContactManager* manager, C3D_Body* body)
{
	C3D_ContactEdge* edge = body->contactList;
	while (edge)
	{
		C3D_ContactEdge* next = edge->next;
		Manager_RemoveConstraint(manager, edge->constraint);
		edge = next;
	}
}

/**
 * @brief Removes the given C3D_Body object from the given C3D_Broadphase object.
 * @param[in,out]      manager            The resulting C3D_ContactManager manager object.
 * @param[in]          body               The C3D_Body object whose C3D_Box objects are to be removed from the C3D_Broadphase object.
 */
void Manager_RemoveBodyFromBroadphase(C3D_ContactManager* manager, C3D_Body* body)
{
	C3D_Box* box = body->boxes;
	while (box)
	{
		Broadphase_RemoveBox(manager->broadphase, box);
		box = box->next;
	}
}

/**
 * @brief Handles collision checks.
 * @param[in,out]       manager           The resulting C3D_ContactManager manager object.
 */
void Manager_CollisionResponse(C3D_ContactManager* manager)
{
	C3D_ContactConstraint* constraint = manager->contactList;
	while (constraint)
	{
		C3D_Box* boxA = constraint->A;
		C3D_Box* boxB = constraint->B;
		C3D_Body* bodyA = boxA->body;
		C3D_Body* bodyB = boxB->body;
		if (!Body_CanCollide(bodyA, bodyB))
		{
			C3D_ContactConstraint* next = constraint->next;
			Manager_RemoveConstraint(manager, constraint);
			constraint = next;
			continue;
		}
		if (!Body_IsAwake(bodyA) && !Body_IsAwake(bodyB))
		{
			constraint = constraint->next;
			continue;
		}
		if (!Broadphase_CanOverlap(manager->broadphase, boxA->broadPhaseIndex, boxB->broadPhaseIndex))
		{
			C3D_ContactConstraint* next = constraint->next;
			Manager_RemoveConstraint(manager, constraint);
			constraint = next;
			continue;
		}
		C3D_Manifold* manifold = &constraint->manifold;
		C3D_Manifold oldManifold = constraint->manifold;
		C3D_FVec oldTangent0 = oldManifold.tangentVectors[0];
		C3D_FVec oldTangent1 = oldManifold.tangentVectors[1];
		Constraint_CollisionResponse(constraint);
		FVec3_ComputeBasis(&manifold->tangentVectors[0], &manifold->tangentVectors[1], manifold->normal);
		for (unsigned int i = 0; i < manifold->contactsCount; i++)
		{
			C3D_Contact* contact = manifold->contacts + i;
			contact->tangentImpulse[0] = contact->tangentImpulse[1] = contact->normalImpulse = 0.0f;
			u8 oldWarmStarted = contact->warmStarted;
			contact->warmStarted = 0;
			for (unsigned int j = 0; j < oldManifold.contactsCount; j++)
			{
				C3D_Contact* oldContact = oldManifold.contacts + j;
				if (contact->featurePair.key == oldContact->featurePair.key)
				{
					contact->normalImpulse = oldContact->normalImpulse;
					
					C3D_FVec friction = FVec3_Add(FVec3_Scale(oldTangent0, oldContact->tangentImpulse[0]), FVec3_Scale(oldTangent1, oldContact->tangentImpulse[1]));
					contact->tangentImpulse[0] = FVec3_Dot(friction, manifold->tangentVectors[0]);
					contact->tangentImpulse[1] = FVec3_Dot(friction, manifold->tangentVectors[1]);
					contact->warmStarted = oldWarmStarted > oldWarmStarted + 1 ? oldWarmStarted : oldWarmStarted + 1;
					break;
				}
			}
		}
		if (manager->contactListener)
		{
			if ((constraint->flags & ConstraintFlag_Colliding) && !(constraint->flags & ConstraintFlag_WasColliding))
				manager->contactListener->vmt->BeginContact(manager->contactListener, constraint);
			else if (!(constraint->flags & ConstraintFlag_Colliding) && (constraint->flags & ConstraintFlag_WasColliding))
				manager->contactListener->vmt->EndContact(manager->contactListener, constraint);
		}
		constraint = constraint->next;
	}
}

/**
 * @brief Render the C3D_ContactConstraint objects.
 * @param[in,out]      manager        The resulting C3D_ContactManager manager object.
 */
void Manager_RenderConstraints(C3D_ContactManager* manager)
{
	// TODO: Manager_RenderConstraints() - Unimplemented method: Requires Citro3D rendering functions.
	// Reference: https://github.com/RandyGaul/qu3e/blob/master/src/dynamics/q3ContactManager.cpp
}

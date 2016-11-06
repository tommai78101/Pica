#include "physics.h"

/*
 * BROADPHASE.C 
 */

/**
 * @brief Initializes the C3D_Broadphase object.
 * @param[in,out]   out                The resulting C3D_Broadphase object to initialize.
 * @param[in]       contactManager     The C3D_ContactManager object to initialize with.
 */
void Broadphase_Init(C3D_Broadphase* out, C3D_ContactManager* const contactManager)
{
	Tree_Init(&out->tree);
	out->contactManager = contactManager;
	out->pairCount = 0;
	out->pairCapacity = 64;
	out->pairBuffer = (C3D_ContactPair*) linearAlloc(sizeof(C3D_ContactPair) * out->pairCapacity);
	out->moveCount = 0;
	out->moveCapacity = 64;
	out->moveBuffer = (int*) linearAlloc(sizeof(unsigned int) * out->moveCapacity);
}

/**
 * @brief Releases the C3D_Broadphase object.
 * @param[in,out]     out      The resulting C3D_Broadphase object to release.
 */
void Broadphase_Free(C3D_Broadphase* out)
{
	linearFree(out->moveBuffer);
	linearFree(out->pairBuffer);
	Tree_Free(&out->tree);
}

/**
 * @brief Inserts the index of the C3D_DynamicAABBTreeNode node object into the broadphase, and marking the node "moved".
 * @param[in,out]    broadphase        The resulting C3D_Broadphase object for moving the index value in the move buffer.
 * @param[in]        index             The C3D_DynamicAABBTreeNode node index in the C3D_DynamicAABBTree tree of the broadphase.
 */
void Broadphase_BufferMove(C3D_Broadphase* broadphase, int index)
{
	if (broadphase->moveCount == broadphase->moveCapacity)
	{
		int* oldBuffer = broadphase->moveBuffer;
		broadphase->moveCapacity *= 2;
		broadphase->moveBuffer = (int*) linearAlloc(sizeof(int) * broadphase->moveCapacity);
		memcpy(broadphase->moveBuffer, oldBuffer, sizeof(int) * broadphase->moveCapacity);
		linearFree(oldBuffer);
	}
	broadphase->moveBuffer[broadphase->moveCount++] = index;
}

/**
 * @brief Inserts a C3D_Box object containing the following C3D_AABB object.
 * @param[in,out]     broadphase        The resulting C3D_Broadphase object.
 * @param[in]         box               The C3D_Box object to insert into the broadphase's dynamic tree.
 * @param[in]         aabb              The C3D_AABB object to write it in.
 */
void Broadphase_InsertBox(C3D_Broadphase* broadphase, C3D_Box* box, C3D_AABB* const aabb)
{
	int id = Tree_Insert(&broadphase->tree, aabb, box);
	box->broadPhaseIndex = id;
	Broadphase_BufferMove(broadphase, id);
}

/**
 * @brief Removes a C3D_Box object from the tree node of the broadphase.
 * @param[in,out]  broadphase        The resulting C3D_Broadphase object, with the C3D_Box object removed.
 * @param[in]      box               The target C3D_Box object to be removed from the broadphase.
 */
void Broadphase_RemoveBox(C3D_Broadphase* broadphase, const C3D_Box* box)
{
	Tree_Remove(&broadphase->tree, box->broadPhaseIndex);
}

/**
 * @brief A callback function used only for the standard C++ function, std::sort(), in the <algorithm> standard header. Not intended to be used for anything else.
 * @param[in]     lhs       The operand input for qsort().
 * @param[in]     rhs       The operand input for qsort().
 * @return Value for std::sort() to determine priority/order.
 */
bool Broadphase_ContactPairSort(const C3D_ContactPair* lhs, const C3D_ContactPair* rhs)
{
	if (lhs->A < rhs->A)
		return true;
	if (lhs->A == rhs->A)
		return lhs->B < rhs->B;
	return false;
}

/**
 * @brief A callback function used only for the standard C function, qsort(), in the <stdlib.h> standard header. Not intended to be used for anything else.
 * @param[in]     a       The operand input for qsort().
 * @param[in]     b       The operand input for qsort().
 * @return Value for qsort() to determine priority/order.
 */
int Broadphase_ContactPairQSort(const void* a, const void* b)
{
	C3D_ContactPair* lhs = (C3D_ContactPair*) a;
	C3D_ContactPair* rhs = (C3D_ContactPair*) b;
	if (lhs->A < rhs->A)
		return -1;
	if (lhs->A == rhs->A)
	{
		if (lhs->B < rhs->B)
			return -1;
		else
			return 1;
	}
	return 1;
}

/**
 * @brief Updates and validates any modified changes to the C3D_ContactPair objects.
 * @param[in,out]    broadphase       The resulting C3D_Broadphase object to update/validate the C3D_ContactPair objects from.
 */
void Broadphase_UpdatePairs(C3D_Broadphase* broadphase)
{
	broadphase->pairCount = 0;
	for (int i = 0; i < broadphase->moveCount; i++)
	{
		broadphase->currentIndex = broadphase->moveBuffer[i];
		C3D_AABB aabb = Tree_GetFatAABB(&broadphase->tree, broadphase->currentIndex);
		Tree_Query(&broadphase->tree, broadphase, &aabb);
	}
	broadphase->moveCount = 0;
	
	//TODO: Check to see if this qsort() is really working as it should be.
	qsort(broadphase->pairBuffer, broadphase->pairCount, sizeof(C3D_ContactPair), Broadphase_ContactPairQSort);
	
	{
		int i = 0;
		while (i < broadphase->pairCount)
		{
			C3D_ContactPair* pair = broadphase->pairBuffer + i;
			C3D_Box* boxA = (C3D_Box*) Tree_GetUserData(&broadphase->tree, pair->A);
			C3D_Box* boxB = (C3D_Box*) Tree_GetUserData(&broadphase->tree, pair->B);
			Manager_AddConstraint(broadphase->contactManager, boxA, boxB);
			i++;
			while (i < broadphase->pairCount)
			{
				C3D_ContactPair* potentialDuplicate = broadphase->pairBuffer + i;
				if ((pair->A != potentialDuplicate->A) || (pair->B != potentialDuplicate->B))
					break;
				i++;
			}
		}
	}
	Tree_Validate(&broadphase->tree);
}

/**
 * @brief Updates the entire C3D_Broadphase object, by updating the C3D_DynamicAABBTree tree object of the broadphase.
 * @param[in,out]      broadphase           The resulting C3D_Broadphase object to update.
 * @param[in]          id                   The C3D_DynamicAABBTreeNode node object to update with the new C3D_AABB object.
 * @param[in]          aabb                 The C3D_AABB object for updating the C3D_DynamicAABBTreeNode node object with.
 */
void Broadphase_Update(C3D_Broadphase* broadphase, unsigned int id, const C3D_AABB* aabb)
{
	if (Tree_Update(&broadphase->tree, id, aabb))
		Broadphase_BufferMove(broadphase, id);
}

/**
 * @brief Check for any overlapping C3D_DynamicAABBTreeNode node objects based on the nodes' C3D_AABB boundaries.
 * @param[in,out]       broadphase          The resulting C3D_Broadphase object to test for overlaps.
 * @param[in]           A                   The first C3D_DynamicAABBTreeNode node object's ID to test overlaps with.
 * @param[in]           B                   The second C3D_DynamicAABBTreeNode node object's ID to test overlaps with.
 * @return True if there exists an overlap. False, if otherwise. 
 */
bool Broadphase_CanOverlap(C3D_Broadphase* broadphase, int A, int B)
{
	C3D_AABB fatA = Tree_GetFatAABB(&broadphase->tree, A);
	C3D_AABB fatB = Tree_GetFatAABB(&broadphase->tree, A);
	return AABB_CollidesAABB(&fatA, &fatB);
}

/**
 * @brief The default callback function for C3D_Broadphase objects to query. Should not be used outside of C3D_Broadphase objects. Usually, the callback is from 
 *        user-defined callbacks, and not the default callback.
 * @note: 
 * RandyGaul: When a query is made, the query will find all matches, and this exhaustive search takes CPU time. Sometimes all the user cares about is a particular query 
 *            "hit", and then wants to terminate the rest of the search immediately. For example I shoot a ray into the world to check and see if I hit *anything*, so I 
 *            would pass in false to first result.
 *            
 *            The tree callback can be the one given by default, or be a user supplied callback. The internal callback you pointed out in q3BroadPhase.h is interested in 
 *            *all* broadphase reports, and will always return true. This is why you sometimes hear negative comments about using callbacks. Generally they are complicated 
 *            and difficult to follow. They ruin typical code-flow.
 *             
 *            Callbacks are an abstraction, and the abstraction cost is harder to follow code. In the physics engine case since it stores a lot of memory and users want to peek into the memory the 
 *            callbacks seem necessary, but I have never been happy with them. 
 * @param[in,out]     broadphase       The resulting C3D_Broadphase object.
 * @param[in]         index            The C3D_ContactPair object index to check on.
 * @return True, because the default callback is interested in all broadphase reports.
 */
bool Broadphase_TreeCallback(C3D_Broadphase* broadphase, int index)
{
	if (index == broadphase->currentIndex)
		return true;
	if (broadphase->pairCount == broadphase->pairCapacity)
	{
		C3D_ContactPair* oldBuffer = broadphase->pairBuffer;
		broadphase->pairCapacity *= 2;
		broadphase->pairBuffer = (C3D_ContactPair*) linearAlloc(sizeof(C3D_ContactPair) * broadphase->pairCapacity);
		memcpy(broadphase->pairBuffer, oldBuffer, sizeof(C3D_ContactPair) * broadphase->pairCount);
		linearFree(oldBuffer);
	}
	int indexA = index < broadphase->currentIndex ? index : broadphase->currentIndex;
	int indexB = index > broadphase->currentIndex ? index : broadphase->currentIndex;
	broadphase->pairBuffer[broadphase->pairCount].A = indexA;
	broadphase->pairBuffer[broadphase->pairCount].B = indexB;
	broadphase->pairCount++;
	return true;
}

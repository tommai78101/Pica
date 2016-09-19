#include <c3d/physics.h>

void Broadphase_Init(C3D_Broadphase* out, C3D_ContactManager* const contactManager)
{
	out->contactManager = contactManager;
	out->pairCount = 0;
	out->pairCapacity = 64;
	out->pairBuffer = (C3D_ContactPair*) linearAlloc(sizeof(C3D_ContactPair) * out->pairCapacity);
	out->moveCount = 0;
	out->moveCapacity = 64;
	out->moveBuffer = (int*) linearAlloc(sizeof(unsigned int) * out->moveCapacity);
}

void Broadphase_Free(C3D_Broadphase* out)
{
	linearFree(out->moveBuffer);
	linearFree(out->pairBuffer);
}

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

void Broadphase_InsertBox(C3D_Broadphase* broadphase, C3D_Box* box, C3D_AABB* const aabb)
{
	int id = Tree_Insert(broadphase->tree, aabb, box);
	box->broadPhaseIndex = id;
	Broadphase_BufferMove(broadphase, id);
}

void Broadphase_RemoveBox(C3D_Broadphase* broadphase, const C3D_Box* box)
{
	Tree_Remove(broadphase->tree, box->broadPhaseIndex);
}

bool Broadphase_ContactPairSort(const C3D_ContactPair* lhs, const C3D_ContactPair* rhs)
{
	if (lhs->A < rhs->A)
		return true;
	if (lhs->A == rhs->A)
		return lhs->B < rhs->B;
	return false;
}

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

void Broadphase_UpdatePairs(C3D_Broadphase* broadphase)
{
	broadphase->pairCount = 0;
	for (int i = 0; i < broadphase->moveCount; i++)
	{
		broadphase->currentIndex = broadphase->moveBuffer[i];
		C3D_AABB aabb = Tree_GetFatAABB(broadphase->tree, broadphase->currentIndex);
		Tree_Query(broadphase->tree, broadphase, &aabb);
	}
	broadphase->moveCount = 0;
	
	//TODO: Check to see if this qsort() is really working as it should be.
	qsort(broadphase->pairBuffer, broadphase->pairCount, sizeof(C3D_ContactPair), Broadphase_ContactPairQSort);
	
	{
		int i = 0;
		while (i < broadphase->pairCount)
		{
			C3D_ContactPair* pair = broadphase->pairBuffer + i;
			C3D_Box* boxA = (C3D_Box*) Tree_GetUserData(broadphase->tree, pair->A);
			C3D_Box* boxB = (C3D_Box*) Tree_GetUserData(broadphase->tree, pair->B);
			Manager_AddContact(broadphase->contactManager, boxA, boxB);
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
	Tree_Validate(broadphase->tree);
}

void Broadphase_Update(C3D_Broadphase* broadphase, int id, const C3D_AABB* aabb)
{
	if (Tree_Update(broadphase->tree, id, aabb))
		Broadphase_BufferMove(broadphase, id);
}

bool Broadphase_CanOverlap(C3D_Broadphase* broadphase, int A, int B)
{
	C3D_AABB fatA = Tree_GetFatAABB(broadphase->tree, A);
	C3D_AABB fatB = Tree_GetFatAABB(broadphase->tree, A);
	return AABB_CollidesAABB(&fatA, &fatB);
}

bool Broadphase_TreeCallback(C3D_Broadphase* broadphase, int index)
{
	if (index == broadphase->currentIndex)
		return false;  //FIXME: Base code says this is "true", but I feel this should be "false".
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

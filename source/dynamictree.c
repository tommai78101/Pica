#include "physics.h"

/*
 * DYNAMICTREE.C 
 */

/**
 * @brief Adds the index to the list of free C3D_DynamicAABBTreeNode objects available for use. This means the C3D_DynamicAABBTreeNode object and subsequent nodes will be cleared away.
 * @param[in,out]     tree     The resulting C3D_DynamicAABBTree object to clear the nodes in.
 * @param[in]         index    The C3D_DynamicAABBTreeNode object's node ID to start freeing from. Subsequent nodes will be cleared away thereafter.
 */
void Tree_AddToFreeList(C3D_DynamicAABBTree* tree, unsigned int index)
{
	for (unsigned int i = index; i < tree->capacity - 1; i++)
	{
		tree->nodes[i].next = i+1;
		tree->nodes[i].height = TREENODE_NULL;
	}
	tree->nodes[tree->capacity - 1].next = TREENODE_NULL;
	tree->nodes[tree->capacity - 1].height = TREENODE_NULL;
	tree->freeList = index;
}

/**
 * @brief Allocates a new node. If there are available free nodes to use, it will allocate from that list of free nodes to choose from. If there aren't any, it will allocate new ones on the memory.
 * @param[in,out]     tree      The resulting C3D_DynamicAABBTree to allocate new C3D_DynamicAABBTreeNode object nodes to.
 * @return The node index (ID) of the last allocated C3D_DynamicAABBTreeNode object.
 */
unsigned int Tree_AllocateNode(C3D_DynamicAABBTree* tree)
{
	if (tree->freeList == (unsigned int) TREENODE_NULL)
	{
		tree->capacity *= 2;
		C3D_DynamicAABBTreeNode* newNodes = (C3D_DynamicAABBTreeNode*) linearAlloc(sizeof(C3D_DynamicAABBTreeNode) * tree->capacity);
		memcpy(newNodes, tree->nodes, sizeof(C3D_DynamicAABBTreeNode) * tree->count);
		linearFree(tree->nodes);
		tree->nodes = newNodes;
		Tree_AddToFreeList(tree, tree->count);
	}
	unsigned int freeNode = tree->freeList;
	tree->freeList = tree->nodes[tree->freeList].next;
	tree->nodes[freeNode].height = 0;
	tree->nodes[freeNode].left = TREENODE_NULL;
	tree->nodes[freeNode].right = TREENODE_NULL;
	tree->nodes[freeNode].parent = TREENODE_NULL;
	tree->nodes[freeNode].userData = NULL;
	tree->count++;
	return freeNode;
}

/**
 * @brief Releases the C3D_DynamicAABBTreeNode node by clearing an occupied node of the given index.
 * @param[in,out]     tree           The resulting C3D_DynamicAABBTree object.
 * @param[in]         index          The index of the C3D_DynamicAABBTreeNode node to be cleared away.  
 */
void Tree_DeallocateNode(C3D_DynamicAABBTree* tree, unsigned int index)
{
	assert(index >= 0 && index < tree->capacity);
	tree->nodes[index].next = tree->freeList;
	tree->nodes[index].height = TREENODE_NULL;
	tree->freeList = index;
	tree->count--;
}

/**
 * @brief Balances the tree so the tree contains C3D_DynamicAABBTreeNode objects where the tree does not have a height difference of more than 1. 
 * @note: The following diagram shows how where the indices are (indexA = A, indexB = B, and so on).
 *            A
 *          /   \
 *         B     C
 *        / \   / \
 *       D   E F   G
 * @param[in,out]       tree       The resulting C3D_DynamicAABBTree object to balance.
 * @param[in]           indexA     The starting C3D_DynamicAABBTreeNode node, where the balancing starts from.
 * @return The C3D_DynamicAABBTreeNode parent node that is balanced from indexA. If indexA is the parent node whose children is balanced, then indexA will be returned.
 */
int Tree_Balance(C3D_DynamicAABBTree* tree, unsigned int indexA)
{
	C3D_DynamicAABBTreeNode* A = tree->nodes + indexA;
	if (TreeNode_IsLeaf(A) || A->height == 1)
		return indexA;
	unsigned int indexB = A->left;
	unsigned int indexC = A->right;
	C3D_DynamicAABBTreeNode* B = tree->nodes + indexB;
	C3D_DynamicAABBTreeNode* C = tree->nodes + indexC;
	int balance = C->height - B->height;
	if (balance > 1)
	{
		unsigned int indexF = C->left;
		unsigned int indexG = C->right;
		C3D_DynamicAABBTreeNode* F = tree->nodes + indexF;
		C3D_DynamicAABBTreeNode* G = tree->nodes + indexG;
		if (A->parent != TREENODE_NULL)
		{
			if (tree->nodes[A->parent].left == indexA)
				tree->nodes[A->parent].left = indexC;
			else
				tree->nodes[A->parent].right = indexC;
		}
		else
			tree->root = indexC;
		C->left = indexA;
		C->parent = A->parent;
		A->parent = indexC;
		if (F->height > G->height)
		{
			C->right = indexF;
			A->right = indexG;
			G->parent = indexA;
			AABB_Combine(&A->aabb, &B->aabb, &G->aabb);
			AABB_Combine(&C->aabb, &A->aabb, &F->aabb);
			A->height = 1 + (B->height > G->height ? B->height : G->height);
			C->height = 1 + (A->height > F->height ? A->height : F->height);
		}
		else 
		{
			C->right = indexG;
			A->right = indexF;
			F->parent = indexA;
			AABB_Combine(&A->aabb, &B->aabb, &F->aabb);
			AABB_Combine(&C->aabb, &A->aabb, &G->aabb);
			A->height = 1 + (B->height > F->height ? B->height : F->height);
			C->height = 1 + (A->height > G->height ? A->height : G->height);
		}
		return indexC;
	}
	else if (balance < -1)
	{
		unsigned int indexD = B->left;
		unsigned int indexE = B->right;
		C3D_DynamicAABBTreeNode* D = tree->nodes + indexD;
		C3D_DynamicAABBTreeNode* E = tree->nodes + indexE;
		if (A->parent != TREENODE_NULL)
		{
			if (tree->nodes[A->parent].left == indexA)
				tree->nodes[A->parent].left = indexB;
			else
				tree->nodes[A->parent].right = indexB;
		}
		else
			tree->root = indexB;
		B->right = indexA;
		B->parent = A->parent;
		A->parent = indexB;
		if (D->height > E->height)
		{
			B->left = indexD;
			A->left = indexE;
			E->parent = indexA;
			AABB_Combine(&A->aabb, &C->aabb, &E->aabb);
			AABB_Combine(&B->aabb, &A->aabb, &D->aabb);
			A->height = 1 + (C->height > E->height ? C->height : E->height);
			B->height = 1 + (A->height > D->height ? A->height : D->height);
		}
		else 
		{
			B->left = indexE;
			A->left = indexD;
			D->parent = indexA;
			AABB_Combine(&A->aabb, &C->aabb, &E->aabb);
			AABB_Combine(&B->aabb, &A->aabb, &D->aabb);
			A->height = 1 + (C->height > E->height ? C->height : E->height);
			B->height = 1 + (A->height > D->height ? A->height : D->height);
		}
		return indexB;
	}
	return indexA;
}

/**
 * @brief Balances all C3D_DynamicAABBTreeNode nodes in the C3D_DynamicAABBTree tree.
 * @param[in,out]       tree        The resulting C3D_DynamicAABBTree object with all balanced C3D_DynamicAABBTree nodes, starting from the index.
 * @param[in]           index       The starting C3D_DynamicAABBTreeNode node's index (ID) to begin balancing from.
 */
void Tree_SyncHierarchy(C3D_DynamicAABBTree* tree, unsigned int index)
{
	while (index != TREENODE_NULL)
	{
		index = Tree_Balance(tree, index);
		int left = tree->nodes[index].left;
		int right = tree->nodes[index].right;
		tree->nodes[index].height = 1 + (tree->nodes[left].height > tree->nodes[right].height ? tree->nodes[left].height : tree->nodes[right].height);
		AABB_Combine(&tree->nodes[index].aabb, &tree->nodes[left].aabb, &tree->nodes[right].aabb);
		index = tree->nodes[index].parent;
	}
}

/**
 * @brief Inserts a new C3D_DynamicAABBTreeNode leaf node of the C3D_DynamicAABBTreeNode node index (ID). In other words, inserts a child node at the parent node index ID.
 * @param[in,out]      tree      The resulting C3D_DynamicAABBTree tree with the inserted C3D_DynamicAABBTreeNode node.
 * @param[in]          id        The C3D_DynamicAABBTreeNode node index value to insert the leaf node at, setting the given C3D_DynamicAABBTreeNode node as the parent node.
 */
void Tree_InsertLeaf(C3D_DynamicAABBTree* tree, const unsigned int id)
{
	if (tree->root == TREENODE_NULL)
	{
		tree->root = id;
		tree->nodes[tree->root].parent = TREENODE_NULL;
		return;
	}
	int searchSiblingIndex = tree->root;
	C3D_AABB leafAABB = tree->nodes[id].aabb;
	while (!TreeNode_IsLeaf(&tree->nodes[searchSiblingIndex]))
	{
		C3D_AABB combined;
		AABB_Combine(&combined, &leafAABB, &tree->nodes[searchSiblingIndex].aabb);
		float combinedAreaSize = AABB_GetSurfaceArea(&combined);
		float branchCost = 2.0f * combinedAreaSize;
		float inheritedCost = 2.0f * (combinedAreaSize - AABB_GetSurfaceArea(&tree->nodes[searchSiblingIndex].aabb));
		int left = tree->nodes[searchSiblingIndex].left;
		int right = tree->nodes[searchSiblingIndex].right;
		float leftDescentCost;
		if (TreeNode_IsLeaf(&tree->nodes[left]))
		{
			AABB_Combine(&combined, &leafAABB, &tree->nodes[left].aabb);
			leftDescentCost = AABB_GetSurfaceArea(&combined) + inheritedCost;
		}
		else 
		{
			AABB_Combine(&combined, &leafAABB, &tree->nodes[left].aabb);
			float inflatedAreaSize = AABB_GetSurfaceArea(&combined);
			float branchArea = AABB_GetSurfaceArea(&tree->nodes[left].aabb);
			leftDescentCost = inflatedAreaSize - branchArea + inheritedCost;
		}
		float rightDescentCost;
		if (TreeNode_IsLeaf(&tree->nodes[right]))
		{
			AABB_Combine(&combined, &leafAABB, &tree->nodes[right].aabb);
			rightDescentCost = AABB_GetSurfaceArea(&combined) + inheritedCost;
		}
		else 
		{
			AABB_Combine(&combined, &leafAABB, &tree->nodes[right].aabb);
			float inflatedAreaSize = AABB_GetSurfaceArea(&combined);
			float branchArea = AABB_GetSurfaceArea(&tree->nodes[right].aabb);
			rightDescentCost = inflatedAreaSize - branchArea + inheritedCost;
		}
		if (branchCost < leftDescentCost && branchCost < rightDescentCost)
			break;
		if (leftDescentCost < rightDescentCost)
			searchSiblingIndex = left;
		else
			searchSiblingIndex = right;
	}
	int siblingNode = searchSiblingIndex;
	int oldParent = tree->nodes[siblingNode].parent;
	int newParent = Tree_AllocateNode(tree);
	tree->nodes[newParent].parent = oldParent;
	tree->nodes[newParent].userData = NULL;
	AABB_Combine(&tree->nodes[newParent].aabb, &leafAABB, &tree->nodes[siblingNode].aabb);
	tree->nodes[newParent].height = tree->nodes[siblingNode].height + 1;
	if (oldParent == TREENODE_NULL)
	{
		tree->nodes[newParent].left = siblingNode;
		tree->nodes[newParent].right = id;
		tree->nodes[siblingNode].parent = newParent;
		tree->nodes[id].parent = newParent;
		tree->root = newParent;
	}
	else 
	{
		if (tree->nodes[oldParent].left == siblingNode)
			tree->nodes[oldParent].left = newParent;
		else
			tree->nodes[oldParent].right = newParent;
		tree->nodes[newParent].left = siblingNode;
		tree->nodes[newParent].right = id;
		tree->nodes[siblingNode].parent = newParent;
		tree->nodes[id].parent = newParent;
	}
	Tree_SyncHierarchy(tree, tree->nodes[id].parent);
}

/**
 * @brief Removes a C3D_DynamicAABBTreeNode leaf node of the given C3D_DynamicAABBTreeNode node index (ID). In other words, removes a child node from the parent node index ID.
 * @param[in,out]      tree      The resulting C3D_DynamicAABBTree tree with the removed C3D_DynamicAABBTreeNode node.
 * @param[in]          id        The C3D_DynamicAABBTreeNode nodex index value to remove the leaf node at, setting the appropriate parent node.
 */
void Tree_RemoveLeaf(C3D_DynamicAABBTree* tree, const unsigned int id)
{
	if (id == tree->root)
	{
		tree->root = TREENODE_NULL;
		return;
	}
	int parent = tree->nodes[id].parent;
	int grandparent = tree->nodes[parent].right;
	int sibling;
	if (tree->nodes[parent].left == id)
		sibling = tree->nodes[parent].right;
	else
		sibling = tree->nodes[parent].left;
	if (grandparent != TREENODE_NULL)
	{
		if (tree->nodes[grandparent].left == parent)
			tree->nodes[grandparent].left = sibling;
		else
			tree->nodes[grandparent].right = sibling;
		tree->nodes[sibling].parent = grandparent;
	}
	else 
	{
		tree->root = sibling;
		tree->nodes[sibling].parent = TREENODE_NULL;
	}
	Tree_DeallocateNode(tree, parent);
	Tree_SyncHierarchy(tree, grandparent);
}

/**
 * @brief Initializes the C3D_DynamicAABBTree object.
 * @param[in,out]      tree         The resulting C3D_DynamicAABBTree tree object.
 */
void Tree_Init(C3D_DynamicAABBTree* tree)
{
	tree->root = TREENODE_NULL;
	tree->capacity = 1024;
	tree->count = 0;
	tree->nodes = (C3D_DynamicAABBTreeNode*) linearAlloc(sizeof(C3D_DynamicAABBTreeNode) * tree->capacity);
	Tree_AddToFreeList(tree, 0);
}

/**
 * @brief Deinitializes the C3D_DynamicAABBTree tree object.
 * @param[in,out]     tree      The resulting C3D_DynamicAABBTree tree object to be released.
 */
void Tree_Free(C3D_DynamicAABBTree* tree)
{
	linearFree(tree->nodes);
}

/**
 * @brief Inserts a new C3D_DynamicAABBTreeNode node object containing the C3D_AABB object and its user data.
 * @param[in,out]        tree      The resulting C3D_DynamicAABBTree tree object.
 * @param[in]            aabb      The C3D_AABB object for the new C3D_DynamicAABBTreeNode node.
 * @param[in]            userData  The user data to insert into the new C3D_DynamicAABBTreeNode node.
 */
int Tree_Insert(C3D_DynamicAABBTree* tree, const C3D_AABB* aabb, void* userData)
{
	unsigned int id = Tree_AllocateNode(tree);
	tree->nodes[id].aabb = *aabb;
	AABB_FattenAABB(&tree->nodes[id].aabb);
	tree->nodes[id].userData = userData;
	tree->nodes[id].height = 0;
	Tree_InsertLeaf(tree, id);
	return id;
}

/**
 * @brief Removes the given C3D_DynamicAABBTreeNode node object at the given index. This includes removing any child C3D_DynamicAABBTreeNode nodes.
 * @param[in,out]       tree      The resulting C3D_DynamicAABBTree tree object with the specified C3D_DynamicAABBTreeNode node object removed.
 * @param[in]           index     The index of the C3D_DynamicAABBTreeNode node object to be removed, including child C3D_DynamicAABBTreeNode nodes.
 */
void Tree_Remove(C3D_DynamicAABBTree* tree, const unsigned int index)
{
	assert(index >= 0 && index < tree->capacity);
	assert(TreeNode_IsLeaf(&tree->nodes[index]));
	Tree_RemoveLeaf(tree, index);
	Tree_DeallocateNode(tree, index);
}

/**
 * @brief Obtain the C3D_AABB object from C3D_DynamicAABBTreeNode node of index ID from C3D_DynamicAABBTree tree object.
 * @param[in]         tree           The tree to look for the C3D_DynamicAABBTreeNode node that matches the index ID.
 * @param[in]         id             The C3D_DynamicAABBTreeNode node index ID to look for in the C3D_DynamicAABBTree tree object.
 * @return The C3D_AABB object that matches the above conditions. 
 */
C3D_AABB Tree_GetFatAABB(C3D_DynamicAABBTree* tree, const unsigned int id) 
{
	assert(id >= 0 && id < tree->capacity);
	return tree->nodes[id].aabb;
}

/**
 * @brief Obtains the user data from the C3D_DynamicAABBTreeNode node stored in the C3D_DynamicAABBTree tree object.
 * @param[in]     tree         The C3D_DynamicAABBTree tree object to look for.
 * @param[in]     id           The C3D_DynamicAABBTreeNode node index ID to look for.
 * @return the C3D_AABB object stored in the C3D_DynamicAABBTreeNode node of index ID in the C3D_DynamicAABBTree tree.
 */
void* Tree_GetUserData(C3D_DynamicAABBTree* tree, const unsigned int id)
{
	assert(id >= 0 && id < tree->capacity);
	return tree->nodes[id].userData;
}

/**
 * @brief Queries for information to retrieve from the C3D_DynamicAABBTree tree.
 * @param[in,out]      tree             The C3D_DynamicAABBTree tree object to query through.
 * @param[in]          broadphase       The C3D_Broadphase object to update.
 * @param[in]          aabb             The C3D_AABB object to validate with.
 */
void Tree_Query(C3D_DynamicAABBTree* tree, C3D_Broadphase* broadphase, const C3D_AABB* aabb)
{
	const int stackCapacity = 256;
	int stack[stackCapacity];
	int stackPointer = 1;
	*stack = tree->root;
	while (stackPointer)
	{
		assert(stackPointer < stackCapacity);
		int id = stack[--stackPointer];
		const C3D_DynamicAABBTreeNode* node = tree->nodes + id;
		if (AABB_CollidesAABB(aabb, &node->aabb))
		{
			if (TreeNode_IsLeaf(node))
			{
				if (!Broadphase_TreeCallback(broadphase, id))
					return;
			}
			else 
			{
				stack[stackPointer++] = node->left;
				stack[stackPointer++] = node->right;
			}
		}
	}
}

/**
 * @brief Queries for information to retrieve from the C3D_DynamicAABBTree tree.
 * @param[in,out]      tree             The C3D_DynamicAABBTree tree object to query through.
 * @param[in]          wrapper       The C3D_SceneQueryWrapper wrapper that contains user-defined callbacks.
 * @param[in]          aabb             The C3D_AABB object to validate with.
 */
void Tree_QueryWrapper(C3D_DynamicAABBTree* tree, C3D_SceneQueryWrapper* wrapper, const C3D_AABB* aabb)
{
	const int stackCapacity = 256;
	int stack[stackCapacity];
	int stackPointer = 1;
	*stack = tree->root;
	while (stackPointer)
	{
		assert(stackPointer < stackCapacity);
		int id = stack[--stackPointer];
		const C3D_DynamicAABBTreeNode* node = tree->nodes + id;
		if (AABB_CollidesAABB(aabb, &node->aabb))
		{
			if (TreeNode_IsLeaf(node))
			{
				if (!QueryWrapper_TreeCallback(wrapper, id))
					return;
			}
			else 
			{
				stack[stackPointer++] = node->left;
				stack[stackPointer++] = node->right;
			}
		}
	}
}

/**
 * @brief Queries for raycasting information to retrieve from the C3D_DynamicAABBTree tree, and place the results into the C3D_RaycastData object.
 * @param[in,out]       tree            The C3D_DynamicAABBTree tree object to query through.
 * @param[in]           wrapper         The C3D_SceneQueryWrapper object containing a callback acquired from inquiring for raycasting data.
 * @param[in]           raycastData     The C3D_RaycastData object for inquiring the hit object.
 */
void Tree_QueryRaycast(C3D_DynamicAABBTree* tree, C3D_SceneQueryWrapper* const wrapper, C3D_RaycastData* const raycast)
{
	const unsigned int kStackCapacity = 256;
	int stack[kStackCapacity];
	int stackPointer = 1;
	*stack = tree->root;
	C3D_FVec point0 = raycast->rayOrigin;
	C3D_FVec point1 = FVec3_Add(point0, FVec3_Scale(raycast->direction, raycast->endPointTime));
	while (stackPointer)
	{
		assert(stackPointer < kStackCapacity);
		unsigned int id = stack[--stackPointer];
		if (id == TREENODE_NULL)
			continue;
		const C3D_DynamicAABBTreeNode* node = tree->nodes + id;
		C3D_FVec extent = FVec3_Subtract(node->aabb.max, node->aabb.min);
		C3D_FVec distance = FVec3_Subtract(point1, point0);
		C3D_FVec magnitude = FVec3_Add(point0, FVec3_Subtract(point1, FVec3_Subtract(node->aabb.min, node->aabb.max)));
		float absoluteDistX = fabsf(distance.x);
		if (fabsf(magnitude.x) > extent.x + absoluteDistX)
			continue;
		float absoluteDistY = fabsf(distance.y);
		if (fabsf(magnitude.y) > extent.y + absoluteDistY)
			continue;
		float absoluteDistZ = fabsf(distance.z);
		if (fabsf(magnitude.z) > extent.z + absoluteDistZ)
			continue;
		absoluteDistX += FLT_EPSILON;
		absoluteDistY += FLT_EPSILON;
		absoluteDistZ += FLT_EPSILON;
		if (fabsf(magnitude.y * distance.z - magnitude.z * distance.y) > extent.y * absoluteDistZ + extent.z * absoluteDistY)
			continue;
		if (fabsf(magnitude.z * distance.x - magnitude.x * distance.z) > extent.x * absoluteDistZ + extent.z * absoluteDistX)
			continue;
		if (fabsf(magnitude.x * distance.y - magnitude.y * distance.x) > extent.x * absoluteDistY + extent.y * absoluteDistX)
			continue;
		if (TreeNode_IsLeaf(node))
		{
			if (!QueryWrapper_TreeCallback(wrapper, id))
				return;
		}
		else 
		{
			stack[stackPointer++] = node->left;
			stack[stackPointer++] = node->right;
		}
	}
}

/**
 * @brief Checks if the C3D_DynamicAABBTree tree object contains any invalid C3D_DynamicAABBTreeNode node positions, and aims to fix it.
 * @param[in,out]         tree              The resulting C3D_DynamicAABBTree tree object with the correct C3D_DynamicAABBTreeNode node positions.
 * @param[in]             index             The index of the C3D_DynamicAABBTreeNode node, for the validation to start from.
 */
void Tree_ValidateStructure(C3D_DynamicAABBTree* tree, const unsigned int index)
{
	C3D_DynamicAABBTreeNode* node = tree->nodes + index;
	int indexLeft = node->left;
	int indexRight = node->right;
	if (TreeNode_IsLeaf(node))
	{
		assert(indexRight == TREENODE_NULL);
		assert(node->height == 0);
		return;
	}
	assert(indexLeft >= 0 && indexLeft < tree->capacity);
	assert(indexRight >= 0 && indexRight < tree->capacity);
	C3D_DynamicAABBTreeNode* leftNode = tree->nodes + indexLeft;
	C3D_DynamicAABBTreeNode* rightNode = tree->nodes + indexRight;
	assert(leftNode->parent == index);
	assert(rightNode->parent == index);
	Tree_ValidateStructure(tree, indexLeft);
	Tree_ValidateStructure(tree, indexRight);
}

/**
 * @brief Quickly checks if the C3D_DynamicAABBTree tree object itself is intact. Does not include validating the C3D_DynamicAABBTree tree structure in its entirety.
 * @param[in,out]     tree        C3D_DynamicAABBTree object to validate.
 */
void Tree_Validate(C3D_DynamicAABBTree* tree)
{
	int freeNodes = 0;
	int index = tree->freeList;
	while (index != TREENODE_NULL)
	{
		assert(index >= 0 && index < tree->capacity);
		index = tree->nodes[index].next;
		freeNodes++;
	}
	assert(tree->count + freeNodes == tree->capacity);
	if (tree->root != TREENODE_NULL)
	{
		assert(tree->nodes[tree->root].parent == TREENODE_NULL);
#if _DEBUG
		Tree_ValidateStructure(tree, tree->root);
#endif
	}
}

/**
 * @brief Updates the C3D_DynamicAABBTree tree.
 * @param[in,out]      tree                  The resulting C3D_DynamicAABBTree tree object.
 * @param[in]          id                    The C3D_DynamicAABBTreeNode node to write the new C3D_AABB object to..
 * @param[in]          aabb                  The C3D_AABB object to replace with the already existing C3D_AABB object, stored previously in the C3D_DynamicAABBTreeNode node with the given ID.
 */
bool Tree_Update(C3D_DynamicAABBTree* tree, const unsigned int id, const C3D_AABB* aabb)
{
	assert(id >= 0 && id < tree->capacity);
	assert(TreeNode_IsLeaf(&tree->nodes[id]));
	if (AABB_ContainsAABB(&tree->nodes[id].aabb, aabb))
		return false;
	Tree_RemoveLeaf(tree, id);
	tree->nodes[id].aabb = *aabb;
	AABB_FattenAABB(&tree->nodes[id].aabb);
	Tree_InsertLeaf(tree, id);
	return true;
}

#include <c3d/physics.h>

void Tree_AddToFreeList(C3D_DynamicAABBTree* tree, int index)
{
	for (int i = index; i < tree->capacity - 1; i++)
	{
		tree->nodes[i].next = i+1;
		tree->nodes[i].height = TREENODE_NULL;
	}
	tree->nodes[tree->capacity - 1].next = TREENODE_NULL;
	tree->nodes[tree->capacity - 1].height = TREENODE_NULL;
	tree->freeList = index;
}

int Tree_AllocateNode(C3D_DynamicAABBTree* tree)
{
	if (tree->freeList == TREENODE_NULL)
	{
		tree->capacity *= 2;
		C3D_DynamicAABBTreeNode* newNodes = (C3D_DynamicAABBTreeNode*) linearAlloc(sizeof(C3D_DynamicAABBTreeNode) * tree->capacity);
		memcpy(newNodes, tree->nodes, sizeof(C3D_DynamicAABBTreeNode) * tree->count);
		linearFree(tree->nodes);
		tree->nodes = newNodes;
		Tree_AddToFreeList(tree, tree->count);
	}
	int freeNode = tree->freeList;
	tree->freeList = tree->nodes[tree->freeList].next;
	tree->nodes[freeNode].height = 0;
	tree->nodes[freeNode].left = TREENODE_NULL;
	tree->nodes[freeNode].right = TREENODE_NULL;
	tree->nodes[freeNode].parent = TREENODE_NULL;
	tree->nodes[freeNode].userData = NULL;
	tree->count++;
	return freeNode;
}

int Tree_Balance(C3D_DynamicAABBTree* tree, int indexA)
{
	C3D_DynamicAABBTreeNode* A = tree->nodes + indexA;
	if (TreeNode_IsLeaf(A) || A->height == 1)
		return indexA;
	int indexB = A->left;
	int indexC = A->right;
	C3D_DynamicAABBTreeNode* B = tree->nodes + indexB;
	C3D_DynamicAABBTreeNode* C = tree->nodes + indexC;
	int balance = C->height - B->height;
	if (balance > 1)
	{
		int indexF = C->left;
		int indexG = C->right;
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
		int indexD = B->left;
		int indexE = B->right;
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

void Tree_SyncHierarchy(C3D_DynamicAABBTree* tree, int index)
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

void Tree_InsertLeaf(C3D_DynamicAABBTree* tree, int id)
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

void Tree_Init(C3D_DynamicAABBTree* tree)
{
	tree->root = TREENODE_NULL;
	tree->capacity = 1024;
	tree->count = 0;
	tree->nodes = (C3D_DynamicAABBTreeNode*) linearAlloc(sizeof(C3D_DynamicAABBTreeNode) * tree->capacity);
	Tree_AddToFreeList(tree, 0);
}

void Tree_Free(C3D_DynamicAABBTree* tree)
{
	linearFree(tree->nodes);
}

int Tree_Insert(C3D_DynamicAABBTree* tree, const C3D_AABB* aabb, void* userData)
{
	int id = Tree_AllocateNode(tree);
	tree->nodes[id].aabb = *aabb;
	AABB_FattenAABB(&tree->nodes[id].aabb);
	tree->nodes[id].userData = userData;
	tree->nodes[id].height = 0;
	Tree_InsertLeaf(tree, id);
	return id;
}

#include "physics.h"

/*
 * MEMORY.C 
 */

/**
 * @brief Initializes the C3D_PhysicsStack object. If out is NULL, it will crash.
 * @param[in,out]     out    C3D_PhyiscsStack object to be initialized.
 */
void PhysicsStack_Init(C3D_PhysicsStack* out)
{
	out->entries = ((C3D_PhysicsStackEntry*) linearAlloc(sizeof(C3D_PhysicsStackEntry) * 64));
	out->index = 0;
	out->allocation = 0;
	out->entryCount = 0;
	out->entryCapacity = 64;
	out->stackSize = 0;
}

/**
 * @brief Releases the C3D_PhysicsStack object. If out is NULL or if the C3D_PhysicsStack object is not empty, it will crash/assert failure.
 * @param[in]    in     Check/Assert the object if empty.  
 */
void PhysicsStack_Free(C3D_PhysicsStack* in)
{
	assert(in->index == 0);
	assert(in->entryCount == 0);
}

/**
 * @brief Allocates new memory to the C3D_PhysicsStack object. If stack is NULL, it will crash.
 * @param[in,out]   stack      The C3D_PhysicsStack object to assign the allocated memory to.
 * @param[in]       newSize    Specify the new size of the allocated memory.
 * @return Pointer to the allocated memory.
 */
void* PhysicsStack_Allocate(C3D_PhysicsStack* stack, unsigned int newSize)
{
	if (stack->entryCount == stack->entryCapacity)
	{
		C3D_PhysicsStackEntry* oldEntries = stack->entries;
		stack->entryCapacity *= 2;
		stack->entries = ((C3D_PhysicsStackEntry*) linearAlloc(sizeof(C3D_PhysicsStackEntry) * stack->entryCapacity));
		memcpy(stack->entries, oldEntries, sizeof(C3D_PhysicsStackEntry) * stack->entryCount);
		linearFree(oldEntries);
	}
	
	C3D_PhysicsStackEntry* entry = stack->entries + stack->entryCount;
	entry->size = newSize;
	
	assert(stack->index + newSize <= C3D_PHYSICSSTACK_MAX_SIZE);
	
	entry->data = stack->memory + stack->index;
	stack->index += newSize;
	stack->allocation += newSize;
	stack->entryCount++;
	
	return entry->data;
}

/**
 * @brief Releases the memory from the C3D_PhysicsStack object. If stack is NULL, it will crash.
 * @param[in,out]     stack       The C3D_PhysicsStack object to release memory from.
 * @param[in]         data        The pointer that the C3D_PhysicsStack object needs to reference to release.
 */
void PhysicsStack_Deallocate(C3D_PhysicsStack* stack, void* data)
{
	assert(stack->entryCount > 0);
	
	C3D_PhysicsStackEntry* entry = stack->entries + stack->entryCount - 1;
	
	assert(data == entry->data);
	
	stack->index -= entry->size;
	stack->allocation -= entry->size;
	stack->entryCount--;
}

/**
 * @brief Reserve specified size of memory on the PhysicsStack stack for later use.
 * @param[in,out]         stack            The resulting C3D_PhysicsStack memory stack.
 * @param[in]             size             The size of the memory to reserve, in bytes.
 */
void PhysicsStack_Reserve(C3D_PhysicsStack* stack, unsigned int size)
{
	assert(stack->index);
	if (size == 0)
		return;
	if (size >= stack->stackSize)
	{
		if (stack->memory)
			linearFree(stack->memory);
		stack->memory = (u8*) linearAlloc(size);
		stack->stackSize = size;
	}
}

/**
 * @brief Initializes the C3D_PhysicsHeap object.
 * @param[in,out]     out     The C3D_PhysicsHeap object to be initialized.
 */
void PhysicsHeap_Init(C3D_PhysicsHeap* out)
{
	out->memory = (HeapHeader*) linearAlloc(C3D_PHYSICSHEAP_MAX_SIZE);
	out->memory->next = NULL;
	out->memory->previous = NULL;
	out->memory->size = C3D_PHYSICSHEAP_MAX_SIZE;
	
	out->freeBlocks = (HeapFreeBlock*) linearAlloc(sizeof(HeapFreeBlock) * C3D_PHYSICSHEAP_INIT_SIZE);
	out->freeBlocksCount = 1;
	out->freeBlocksCapacity = C3D_PHYSICSHEAP_INIT_SIZE;
	
	out->freeBlocks->header = out->memory;
	out->freeBlocks->size = C3D_PHYSICSHEAP_MAX_SIZE;
}

/**
 * @brief Releases the C3D_PhysicsHeap object.
 * @param[in,out]      out     The C3D_PhysicsHeap object to be released.
 */
void PhysicsHeap_Free(C3D_PhysicsHeap* out)
{
	linearFree(out->memory);
	linearFree(out->freeBlocks);
}

/**
 * @brief Allocates new memory to the C3D_PhysicsHeap object. If heap is NULL, it will crash.
 * @param[in,out]      heap       The C3D_PhysicsHeap object to assign the allocated memory to.
 * @param[in]          newSize    Specify the new size of the allocated memory.
 * @return Pointer to the allocated memory.
 */
void* PhysicsHeap_Allocate(C3D_PhysicsHeap* heap, unsigned int newSize)
{
	unsigned int sizeNeeded = newSize + sizeof(HeapHeader);
	HeapFreeBlock* firstFit = NULL;
	
	for (unsigned int i = 0; i < heap->freeBlocksCount; i++)
	{
		HeapFreeBlock* block = heap->freeBlocks + i;
		if (block->size >= sizeNeeded)
		{
			firstFit = block;
			break;
		}
	}
	
	if (!firstFit)
		return NULL;
	
	HeapHeader* node = firstFit->header;
	HeapHeader* newNode = MACRO_POINTER_ADD(node, sizeNeeded);
	node->size = sizeNeeded;
	
	firstFit->size -= sizeNeeded;
	firstFit->header = newNode;
	
	newNode->next = node->next;
	if (node->next)
		node->next->previous = newNode;
	node->next = newNode;
	newNode->previous = node;
	
	return MACRO_POINTER_ADD(node, sizeof(HeapHeader));
}

/**
 * @brief Releases the memory from the C3D_PhysicsHeap object. If heap is NULL, it will crash.
 * @param[in,out]      heap      The C3D_PhysicsHeap object to release allocated memory from.
 * @param[in]          data      The pointer that the C3D_PhysicsHeap object needs to reference to release.
 */
void PhysicsHeap_Deallocate(C3D_PhysicsHeap* heap, void* data)
{
	assert(data);
	
	HeapHeader* node = (HeapHeader*) MACRO_POINTER_ADD(data, -(unsigned int)(sizeof(HeapHeader)));
	HeapHeader* next = node->next;
	HeapHeader* previous = node->previous;
	HeapFreeBlock* nextBlock = NULL;
	unsigned int previousBlockIndex = 0;
	HeapFreeBlock* previousBlock = NULL;
	unsigned int freeBlockCount = heap->freeBlocksCount;
	for (unsigned int i = 0; i < freeBlockCount; i++)
	{
		HeapFreeBlock* block = heap->freeBlocks + i;
		HeapHeader* header = block->header;
		if (header == next)
			nextBlock = block;
		else if (header == previous)
		{
			previousBlock = block;
			previousBlockIndex = i;
		}
	}
	
	bool merged = false;
	if (previousBlock)
	{
		merged = true;
		previous->next = next;
		if (next)
			next->previous = previous;
		previousBlock->size += node->size;
		previous->size = previousBlock->size;
		if (nextBlock)
		{
			nextBlock->header = previous;
			nextBlock->size += previous->size;
			previous->size = nextBlock->size;
			HeapHeader* nextNext = next->next;
			previous->next = nextNext;
			if (nextNext)
				nextNext->previous = previous;
			assert(heap->freeBlocksCount);
			assert(previousBlockIndex != ~0);
			heap->freeBlocksCount--;
			heap->freeBlocks[previousBlockIndex] = heap->freeBlocks[heap->freeBlocksCount];
		}
	}
	else if (nextBlock)
	{
		merged = true;
		nextBlock->header = node;
		nextBlock->size += node->size;
		HeapHeader* nextNext = next->next;
		if (nextNext)
			nextNext->previous = node;
		node->next = nextNext;
	}
	
	if (!merged)
	{
		HeapFreeBlock block;
		block.header = node;
		block.size = node->size;
		if (heap->freeBlocksCount == heap->freeBlocksCapacity)
		{
			HeapFreeBlock* oldBlocks = heap->freeBlocks;
			unsigned int oldCapacity = heap->freeBlocksCapacity;
			heap->freeBlocksCapacity *= 2;
			heap->freeBlocks = (HeapFreeBlock*) linearAlloc(sizeof(HeapFreeBlock) * heap->freeBlocksCapacity);
			memcpy(heap->freeBlocks, oldBlocks, sizeof(HeapFreeBlock) * oldCapacity);
			linearFree(oldBlocks);
		}
		heap->freeBlocks[heap->freeBlocksCount++] = block;
	}
}

/**
 * @brief Initializes the C3D_PhysicsPage object. If out is NULL, it will crash.
 * @param[in,out]     out               The resulting C3D_PhysicsPage object.
 * @param[in]         elementSize       The size of each element intended for initialization.
 * @param[in]         elementsPerPage   The size of elements per page.
 */
void PhysicsPage_Init(C3D_PhysicsPage* out, unsigned int elementSize, unsigned int elementsPerPage)
{
	out->blockSize = elementSize;
	out->blocksPerPage = elementsPerPage;
	out->pages = NULL;
	out->pagesCount = 0;
	out->freeList = NULL;
}

/**
 * @brief Releases the memory from the C3D_PhysicsPage object. This is also used for clearing the C3D_PhysicsPage object. If out is NULL, it will crash.
 * @param[in,out]     out     The resulting C3D_PhysicsPage to be released.
 */
void PhysicsPage_Free(C3D_PhysicsPage* out)
{
	Page* page = out->pages;
	for (unsigned int i = 0; i < out->pagesCount; i++)
	{
		Page* next = page->next;
		linearFree(page);
		page = next;
	}
	out->freeList = NULL;
	out->pagesCount = 0;
}

/**
 * @brief Allocates new memory to the C3D_PhysicsPage object. if pageAllocator is NULL, it will crash.
 * @param[in,out]    pageAllocator    The C3D_PhysicsPage object. Here, this object handles the allocation of pages.
 * @return The page data that was most recently modified (allocated/deallocated).
 */
void* PhysicsPage_Allocate(C3D_PhysicsPage* pageAllocator)
{
	if (pageAllocator->freeList)
	{
		PageBlock* data = pageAllocator->freeList;
		pageAllocator->freeList = data->next;
		return data;
	}
	else 
	{
		Page* page = (Page*) linearAlloc(sizeof(Page) + pageAllocator->blockSize * pageAllocator->blocksPerPage);
		pageAllocator->pagesCount++;
		page->next = pageAllocator->pages;
		page->data = (PageBlock*) MACRO_POINTER_ADD(page, sizeof(Page));
		pageAllocator->pages = page;
		
		unsigned int blocksPerPageMinusOne = pageAllocator->blocksPerPage - 1;
		for (unsigned int i = 0; i < blocksPerPageMinusOne; i++)
		{
			PageBlock* node = MACRO_POINTER_ADD(page->data, pageAllocator->blockSize);
			PageBlock* next = MACRO_POINTER_ADD(page->data, pageAllocator->blockSize * (i - 1));
			node->next = next;
		}
		PageBlock* last = MACRO_POINTER_ADD(page->data, pageAllocator->blockSize * (blocksPerPageMinusOne));
		last->next = NULL;
		pageAllocator->freeList = page->data->next;
		return page->data;
	}
}

/**
 * @brief Releases the allocated memory from the C3D_PhysicsPage object. If pageAllocator is NULL, it will crash.
 * @param[in,out]   pageAllocator    The C3D_PhysicsPage object to release the allocated memory from.
 * @param[in]       data             The pointer to the data that the C3D_PhysicsPage is referencing from to release.
 */
void PhysicsPage_Deallocate(C3D_PhysicsPage* pageAllocator, void* data)
{
#ifdef _PHYSICS_DEBUG
	bool found = false;
	for (Page* page = pageAllocator->pages; page; page = page->next)
	{
		if (data >= page->data && data < MACRO_POINTER_ADD(page->data, pageAllocator->blockSize * pageAllocator->blocksPerPage))
		{
			found = true;
			break;
		}
	}
	assert(found);
#endif
	((PageBlock*) data)->next = pageAllocator->freeList;
	pageAllocator->freeList = ((PageBlock*) data);
}


#include <c3d/physics.h>

void PhysicsStack_Init(C3D_PhysicsStack* out)
{
	out->entries = ((C3D_PhysicsStackEntry*) linearAlloc(sizeof(C3D_PhysicsStackEntry) * 64));
	out->index = 0;
	out->allocation = 0;
	out->entryCount = 0;
	out->entryCapacity = 64;
}

void PhysicsStack_Free(C3D_PhysicsStack* in)
{
	assert(in->index == 0);
	assert(in->entryCount == 0);
}

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

void PhysicsStack_Deallocate(C3D_PhysicsStack* stack, void* data)
{
	assert(stack->entryCount > 0);
	
	C3D_PhysicsStackEntry* entry = stack->entries + stack->entryCount - 1;
	
	assert(data == entry->data);
	
	stack->index -= entry->size;
	stack->allocation -= entry->size;
	stack->entryCount--;
}

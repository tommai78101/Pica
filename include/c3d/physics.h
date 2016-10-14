#pragma once
#include "3ds.h"
#include "citro3d.h"
#include "types.h"
#include "maths.h"
#include <float.h>
#include <stdbool.h>
#include <assert.h>
#include <stdlib.h>

/*
 * PHYSICS.H 
 */

/**************************************************
 * Constants / Defines
 **************************************************/

//May need to change the stack and heap size to different values.
#define C3D_PHYSICSSTACK_MAX_SIZE 1024*20   //20KB
#define C3D_PHYSICSHEAP_MAX_SIZE 1024*20    //20KB
#define C3D_PHYSICSHEAP_INIT_SIZE 1024      //1KB
#define MACRO_POINTER_ADD(POINTER,BYTES) ((__typeof__(POINTER))(((u8 *)POINTER)+(BYTES)))
#define COLLISION_IN_FRONT(a) ((a) < 0.0f)
#define COLLISION_BEHIND(a) ((a) >= 0.0f)
#define COLLISION_ON(a) ((a) < 0.005f && (a) > -0.005f)
#define C3D_BAUMGARTE 0.2f
#define C3D_SLEEP_ANGULAR ((1.0f/120.0f) * M_TAU)
#define C3D_SLEEP_TIME 0.5f
#define C3D_PENETRATION_SLOP 0.05f

/**
 * @brief Only used for Dynamic AABB Tree objects and related nodes.
 */
static const int TREENODE_NULL = -1;

/**************************************************
 * Enumerations
 **************************************************/

typedef enum C3D_ConstraintFlag 
{
	ConstraintFlag_Colliding        = 0x00000001,
	ConstraintFlag_WasColliding     = 0x00000002,
	ConstraintFlag_Island           = 0x00000004,
} C3D_ConstraintFlag;

typedef enum C3D_BodyType 
{
	BodyType_Static,
	BodyType_Dynamic,
	BodyType_Kinematic
} C3D_BodyType;

typedef enum C3D_BodyFlag 
{
	BodyFlag_Awake         = 0x001,
	BodyFlag_Active        = 0x002,
	BodyFlag_AllowSleep    = 0x004,
	BodyFlag_BodyIsland    = 0x010,
	BodyFlag_Static        = 0x020,
	BodyFlag_Dynamic       = 0x040,
	BodyFlag_Kinematic     = 0x080,
	BodyFlag_LockAxisX     = 0x100,
	BodyFlag_LockAxisY     = 0x200,
	BodyFlag_LockAxisZ     = 0x400,
} C3D_BodyFlag;

/**************************************************
 * Basic Structures
 **************************************************/

typedef struct C3D_PhysicsStackEntry 
{
	u8* data;
	unsigned int size;
} C3D_PhysicsStackEntry;

typedef struct C3D_PhysicsStack 
{
	u8 memory[C3D_PHYSICSSTACK_MAX_SIZE];
	unsigned int index;
	unsigned int allocation;
	unsigned int entryCount;
	unsigned int entryCapacity;
	struct C3D_PhysicsStackEntry* entries;
} C3D_PhysicsStack;

typedef struct HeapHeader 
{
	unsigned int size;
	struct HeapHeader* next;
	struct HeapHeader* previous;
} HeapHeader;

typedef struct HeapFreeBlock 
{
	unsigned int size;
	struct HeapHeader* header;
} HeapFreeBlock;

typedef struct C3D_PhysicsHeap 
{
	unsigned int freeBlocksCount;
	unsigned int freeBlocksCapacity;
	struct HeapHeader* memory;
	struct HeapFreeBlock* freeBlocks;
} C3D_PhysicsHeap;

typedef struct PageBlock 
{
	struct PageBlock* next;
} PageBlock;

typedef struct Page 
{
	struct Page* next;
	struct PageBlock* data;
} Page;

typedef struct C3D_PhysicsPage 
{
	unsigned int blockSize;
	unsigned int blocksPerPage;
	unsigned int pagesCount;
	struct Page* pages;
	struct PageBlock* freeList;
} C3D_PhysicsPage;

typedef struct C3D_Transform 
{
	C3D_FVec position;
	C3D_Mtx rotation;
} C3D_Transform;

typedef struct C3D_AABB 
{
	C3D_FVec min;
	C3D_FVec max;
} C3D_AABB;

typedef struct C3D_DynamicAABBTreeNode 
{
	union 
	{
		int parent;
		int next;
	};
	struct 
	{
		int left;
		int right;
	};
	int height;
	void* userData;
	C3D_AABB aabb;
} C3D_DynamicAABBTreeNode;

/**
 * @note Extent: Half-extents, or the half size of a full axis-aligned bounding box volume. Center of the box, plus half width/height/depth.
 *       Extents as in, you have vec3 and the real position of the box is -vec3 (AABB.min) and +vec3 (AABB.max).
 *       RandyGaul: "Extent, as in the extent of each OBB axis."
 */
typedef struct C3D_Box 
{
	bool sensor;
	unsigned int broadPhaseIndex;
	float friction;
	float restitution;
	float density;
	void* userData;
	C3D_FVec extent; 
	struct C3D_Transform localTransform;
	struct C3D_Box* next;
	struct C3D_Body* body;
} C3D_Box;

typedef struct C3D_HalfSpace 
{
	float distance;
	C3D_FVec normal;
} C3D_HalfSpace;

typedef struct C3D_RaycastData 
{
	float endPointTime;
	float timeOfImpact;  //Solved time of impact.
	C3D_FVec rayOrigin;
	C3D_FVec direction;
	C3D_FVec normal;     //Surface normal at impact.
} C3D_RaycastData;

/**
 *  @note 
 *  RandyGaul: The closest pair of features between two objects (a feature is either a vertex or an edge).
 *   
 *  in stands for "incoming"
 *  out stands for "outgoing"
 *  I stands for "incident"
 *  R stands for "reference"
 *  See Dirk Gregorius GDC 2015 on creating contacts for more details. (Physics for Game Programmers: Robust Contact Creation for Physics Simulations)
 *  
 *  Each feature pair is used to cache solutions from one physics tick to another. This is
 *  called warmstarting, and lets boxes stack and stay stable. Feature pairs identify points
 *  of contact over multiple physics ticks. Each feature pair is the junction of an incoming
 *  feature and an outgoing feature, usually a result of clipping routines. The exact info
 *  stored in the feature pair can be arbitrary as long as the result is a unique ID for a
 *  given intersecting configuration.
 */
typedef union C3D_FeaturePair 
{
	struct 
	{
		u8 incomingReference;
		u8 outgoingReference;
		u8 incomingIncident;
		u8 outgoingIncident;
	};
	unsigned int key;
} C3D_FeaturePair;

typedef struct C3D_Contact 
{
	u8 warmStarted;                        //Used for debug rendering.
	float penetration;                     //Depth of penetration from collision
	float normalImpulse;                   //Accumulated normal impulse.
	float tangentImpulse[2];               //Accumulated friction impulse. Tangent, because it's the opposite direction.
	float bias;                            //Restitution + Baumgarte Stabilization.
	float normalMass;                      //Normal constraint mass.
	float tangentMass[2];                  //Tangent constraint mass.
	C3D_FVec position;                     //World coordinate contact position
	union C3D_FeaturePair featurePair;     //Features on A and B for this contact position.
} C3D_Contact;

typedef struct C3D_ContactPair 
{
	int A;
	int B;
} C3D_ContactPair;

typedef struct C3D_Manifold
{
	bool sensor;
	int contactsCount;
	C3D_FVec normal;
	C3D_FVec tangentVectors[2];
	struct C3D_Box* A;
	struct C3D_Box* B;
	struct C3D_Contact contacts[8];
	struct C3D_Manifold* next;
	struct C3D_Manifold* previous;
} C3D_Manifold;

typedef struct C3D_MassData 
{
	float mass;
	C3D_FVec center;
	C3D_Mtx inertia;
} C3D_MassData;

typedef struct C3D_ContactEdge 
{
	struct C3D_Body* other;
	struct C3D_ContactConstraint* constraint;
	struct C3D_ContactEdge* next;
	struct C3D_ContactEdge* previous;
} C3D_ContactEdge;

typedef struct C3D_ContactConstraint 
{
	unsigned int flags;
	float friction;
	float restitution;
	struct C3D_Box* A;
	struct C3D_Box* B;
	struct C3D_Body* bodyA;
	struct C3D_Body* bodyB;
	struct C3D_ContactEdge edgeA;
	struct C3D_ContactEdge edgeB;
	struct C3D_ContactConstraint* next;
	struct C3D_ContactConstraint* previous;
	struct C3D_Manifold manifold;
} C3D_ContactConstraint;

typedef struct C3D_ContactState 
{
	float penetration;            //Depth of penetration from collision.
	float normalImpulse;          //Accumulated normal impulse.
	float tangentImpulse[2];      //Accumulated friction impulse.
	float bias;                   //Restitution + Baumgarte Stabilization.
	float normalMass;             //Normal constraint mass.
	float tangentMass[2];         //Tangent constraint mass.
	C3D_FVec radiusContactA;      //Vector position point to the contact point relative to the shape A's center of mass. (C.O.M. is center of mass)
	C3D_FVec radiusContactB;      //Vector position point to the contact point relative to the shape B's center of mass.
	
} C3D_ContactState;

typedef struct C3D_ContactConstraintState 
{
	int contactCount;
	int indexBodyA;           //Index of body A in the cache of C3D_Body object pointers.
	int indexBodyB;           //Index of body B in the cache of C3D_Body object pointers.
	float indexMassA;         //Index of mass A.
	float indexMassB;         //Index of mass B.
	float restitution;
	float friction;
	C3D_FVec tangentVectors[2];
	C3D_FVec normal;          //From shape A to shape B.
	C3D_FVec centerA;
	C3D_FVec centerB;
	C3D_Mtx inverseMassA;     //Inverse mass of body A.
	C3D_Mtx inverseMassB;     //Inverse mass of body B.
	struct C3D_ContactState contactStates[8];
} C3D_ContactConstraintState;

typedef struct C3D_ContactManager 
{
	int contactCount;
	struct C3D_ContactConstraint* contactList;
	struct C3D_PhysicsStack* stack;
	struct C3D_PhysicsPage pageAllocator;
	struct C3D_Broadphase* broadphase;
	struct C3D_ContactListener* contactListener;
} C3D_ContactManager;

typedef struct C3D_Body 
{
	void* userData;
	unsigned int layers;
	unsigned int flags;
	unsigned int islandIndex;
	float linearDamping;
	float angularDamping;
	float sleepTime;
	float gravityScale;
	float mass;
	float inverseMass;
	C3D_FVec linearVelocity;
	C3D_FVec angularVelocity;
	C3D_FVec force;
	C3D_FVec torque;
	C3D_FVec localCenter;
	C3D_FVec worldCenter;
	C3D_FQuat quaternion;
	C3D_Mtx inverseInertiaModel;
	C3D_Mtx inverseInertiaWorld;
	struct C3D_Transform transform;
	struct C3D_Box* boxes;
	struct C3D_Scene* scene;
	struct C3D_Body* next;
	struct C3D_Body* previous;
	struct C3D_ContactEdge* contactList;
} C3D_Body;

typedef struct C3D_DynamicAABBTree 
{
	int root;
	int freeList;
	unsigned int count;
	unsigned int capacity;
	struct C3D_DynamicAABBTreeNode* nodes;
} C3D_DynamicAABBTree;

typedef struct C3D_Broadphase 
{
	int* moveBuffer;
	unsigned int pairCount;
	unsigned int pairCapacity;
	unsigned int moveCount;
	unsigned int moveCapacity;
	unsigned int currentIndex;
	struct C3D_DynamicAABBTree* tree;
	struct C3D_ContactManager* contactManager;
	struct C3D_ContactPair* pairBuffer;
} C3D_Broadphase;

typedef struct C3D_Scene 
{
	struct C3D_ContactManager contactManager;
} C3D_Scene;

/**
 * @note Taken from: http://stackoverflow.com/questions/3113583/how-could-one-implement-c-virtual-functions-in-c
 */ 
typedef struct C3D_ContactListener_FuncTable 
{
	void (*Init)(struct C3D_ContactListener*);
	void (*Free)(struct C3D_ContactListener*);
	void (*BeginContact)(struct C3D_ContactListener*, const C3D_ContactConstraint* constraint);
	void (*EndContact)(struct C3D_ContactListener*, const C3D_ContactConstraint* constraint);
} C3D_ContactListener_FuncTable;

typedef struct C3D_ContactListener 
{
	struct C3D_ContactListener_FuncTable* vmt; // vmt:  Virtual Method Table
} C3D_ContactListener;

typedef struct C3D_ClipVertex
{
	C3D_FVec vertex;
	union C3D_FeaturePair featurePair;
} C3D_ClipVertex;

typedef struct C3D_VelocityState 
{
	C3D_FVec w;
	C3D_FVec v;
} C3D_VelocityState;

typedef struct C3D_Island 
{
	bool allowSleep;
	bool enableFriction;
	int iterations;
	unsigned int bodyCapacity;
	unsigned int bodyCount;
	unsigned int contactConstraintStateCount;
	unsigned int contactConstraintStateCapacity;
	float deltaTime;
	C3D_FVec gravity;
	struct C3D_Body** bodies;
	struct C3D_VelocityState* velocityStates;
	struct C3D_ContactConstraint** contactConstraints;
	struct C3D_ContactConstraintState* contactConstraintStates;
} C3D_Island;

typedef struct C3D_ContactSolver 
{
	bool enableFriction;
	unsigned int contactConstraintStateCount;
	struct C3D_Island* island;
	struct C3D_ContactConstraintState* contactConstraintStates;
	struct C3D_VelocityState* velocityStates;
} C3D_ContactSolver;

/**************************************************
 * Common Non-Standard Citro3D Functions 
 **************************************************/

/**
 * @brief Obtain the smaller C3D_FVec vector of the given 2 C3D_FVec vectors.
 * @param[in]    lhs      C3D_FVec vector to compare.
 * @param[in]    rhs      C3D_FVec vector to compare.
 * @return The smallest C3D_FVec vector.
 */
C3D_FVec FVec3_Min(C3D_FVec lhs, C3D_FVec rhs);

/**
 * @brief Obtain the larger C3D_FVec vector of the given 2 C3D_FVec vectors.
 * @param[in]    lhs      C3D_FVec vector to compare.
 * @param[in]    rhs      C3D_FVec vector to compare.
 * @return The largest C3D_FVec vector.
 */
C3D_FVec FVec3_Max(C3D_FVec lhs, C3D_FVec rhs);

/**
 * @brief Obtain the C3D_FVec vector containing the absolute values of each element.
 * @param[in]    vector        The original vector.
 * @return The C3D_FVec vector with absolute values of each element. 
 */
C3D_FVec FVec3_Abs(C3D_FVec vector);

/**
 * @brief See: http://box2d.org/2014/02/computing-a-basis/
 * @param[in]   a   A unit vector.
 * @param[out]  b   A unit vector perpendicular to unit vector, "a".
 * @param[out]  c   A unit vector perpendicular to unit vectors, "a" and "b".
 */ 
void FVec3_ComputeBasis(const C3D_FVec* a, C3D_FVec* b, C3D_FVec* c);

/**
 * @brief Creates a C3D_Mtx containing the outer product of C3D_FVec vectors, lhs and rhs.
 * @param[out]   out     The resulting C3D_Mtx matrix.
 * @param[in]    lhs     The first C3D_FVec vector.
 * @param[in]    rhs     The second C3D_FVec vector.
 */
void Mtx_OuterProduct(C3D_Mtx* out, C3D_FVec* lhs, C3D_FVec* rhs);

/**
 * @brief Obtain the column vector from a C3D_Mtx matrix.
 * @param[in]       in        The C3D_Mtx matrix to retrieve the column vector from.
 * @param[in]       column    The column index of the C3D_Mtx matrix, in the range from 0 to 2 inclusive. (0 ~ 2)
 * @return The column vector from the C3D_Mtx matrix. Size of vector is 3, which are X, Y and Z components.
 */
static inline C3D_FVec Mtx_Column3(C3D_Mtx* in, int column)
{
	return FVec4_New(in->r[0].c[column], in->r[1].c[column], in->r[2].c[column], 0.0f);
}

/**
 * @brief Obtain the column vector from a C3D_Mtx matrix.
 * @param[in]       in        The C3D_Mtx matrix to retrieve the column vector from.
 * @param[in]       column    The column index of the C3D_Mtx matrix, in the range from 0 to 3 inclusive. (0 ~ 3)
 * @return The column vector from the C3D_Mtx matrix. Size of vector is 4, which are X, Y, Z, and W components.
 */
static inline C3D_FVec Mtx_Column4(C3D_Mtx* in, int column)
{
	return FVec4_New(in->r[0].c[column], in->r[1].c[column], in->r[2].c[column], in->r[3].c[column]);
}

/**************************************************
 * Axis-aligned Bounding Box Functions (AABB)
 **************************************************/

/**
 * @brief Checks if inner AABB box is within the outer AABB box.
 * @param[in]   outer   Outer AABB box.
 * @param[in]   inner    Inner AABB box to compare with.
 * @return True if outer AABB box contains an inner AABB box. False, if otherwise. 
 */
bool AABB_ContainsAABB(const C3D_AABB* outer, const C3D_AABB* inner);

/**
 * @brief Checks if C3D_FVec point is within the AABB box.
 * @param[in]   outer   Outer AABB box.
 * @param[in]   inner   C3D_FVec vector position.
 * @return True if vector position is within the AABB box. False, if otherwise.
 */
bool AABB_ContainsFVec3(const C3D_AABB* outer, const C3D_FVec* inner);

/**
 * @brief Obtain the surface area of the AABB box specified.
 * @param[in]  myself   The AABB box for finding the surface area.
 * @return The surface area.
 */
float AABB_GetSurfaceArea(const C3D_AABB* myself);

/**
 * @brief Unionize two AABB boxes to create 1 bigger AABB box that contains the two AABB boxes.
 * @param[out]   out   The resulting larger AABB box.
 * @param[in]    a     The first AABB box.
 * @param[in]    b     The second AABB box.
 */
void AABB_Combine(C3D_AABB* out, const C3D_AABB* a, const C3D_AABB* b);

/**
 * @brief Checks if the two AABB boxes intersects each other.
 * @param[in]    a     The first AABB box.
 * @param[in]    b     The second AABB box.
 * @return True if both boxes intersect each other. False, if otherwise.
 */
bool AABB_CollidesAABB(const C3D_AABB* a, const C3D_AABB* b);

/**
 * @brief Increase the AABB boundaries by 0.5F.
 * @param[in,out]    aabb    C3D_AABB object to fatten.
 */
static inline void AABB_FattenAABB(C3D_AABB* aabb)
{
	const float fattener = 0.5f;
	C3D_FVec fatVector = FVec3_New(fattener, fattener, fattener);
	aabb->min = FVec3_Subtract(aabb->min, fatVector);
	aabb->max = FVec3_Add(aabb->max, fatVector);
}

/**************************************************
 * Raycasting Helper Functions (Raycast)
 **************************************************/

/**
 * @brief Create a new C3D_RaycastData object.
 * @param[out]    out            The resulting C3D_RaycastData object. If out is NULL, it will crash.
 * @param[in]     origin         The beginning point of the ray.
 * @param[in]     direction      The direction of the ray.
 * @param[in]     endPointTime   The time specifying the ray end point.
 */
static inline void Raycast_New(C3D_RaycastData* out, const C3D_FVec* origin, const C3D_FVec* direction, const float endPointTime) 
{
	out->rayOrigin = *origin;
	out->direction = FVec3_Normalize(*direction);
	out->endPointTime = endPointTime;
}

/**************************************************
 * Half-Space Helper Functions (HS)
 **************************************************/

/**
 * @brief Create a new C3D_HalfSpace object.
 * @param[out]     out         The resulting C3D_HalfSpace object. If out is NULL, it will crash.
 * @param[in]      normal      The normal vector.
 * @param[in]      distance    The distance.
 */
static inline void HS_Init(C3D_HalfSpace* out, const C3D_FVec normal, float distance)
{
	out->normal = normal;
	out->distance = distance;
}

/**
 * @brief Create a new C3D_HalfSpace object, using 3 points in the 3D space.
 * @param[out]     out         The resulting C3D_HalfSpace object. If out is NULL, it will crash.
 * @param[in]      a           The first point.
 * @param[in]      b           The second point.
 * @param[in]      c           The third point.
 */
static inline void HS_NewFVec(C3D_HalfSpace* out, const C3D_FVec* a, const C3D_FVec* b, const C3D_FVec* c)
{
	out->normal = FVec3_Normalize(FVec3_Cross(FVec3_Subtract(*b, *a), FVec3_Subtract(*c, *a)));
	out->distance = FVec3_Dot(out->normal, *a);
}

/**
 * @brief Create a new C3D_HalfSpace object, using a normal vector and a vector position.
 * @param[out]     out      The resulting C3D_HalfSpace object. If out is NULL, it will crash.
 * @param[in]      normal   The normal vector.
 * @param[in]      point    The vector position.
 */
static inline void HS_New(C3D_HalfSpace* out, const C3D_FVec* normal, const C3D_FVec* point)
{
	out->normal = FVec3_Normalize(*normal);
	out->distance = FVec3_Dot(out->normal, *point);
}

/**
 * @brief Obtain the origin from the C3D_HalfSpace object.
 * @param[in]     in    C3D_HalfSpace object to retrieve the origin vector position from.
 * @return The C3D_FVec origin.
 */
C3D_FVec HS_GetOrigin(C3D_HalfSpace* in);

/**
 * @brief Get the distance from the half space to the point.
 * @param[in]    in     C3D_HalfSpace object to measure from.
 * @param[in]    point  C3D_FVec vector position to measure to.
 * @return The distance between the half space to the point.
 */
float HS_GetDistance(C3D_HalfSpace* in, const C3D_FVec* point);

/**
 * @brief Projects the half space to the vector position.
 * @param[in]    in     C3D_HalfSpace object.
 * @param[in]    point  Projection destination position.
 * @return The projection vector.
 */
C3D_FVec HS_Project(C3D_HalfSpace* in, const C3D_FVec* point);

/**************************************************
 * Physics Memory Functions (Physics)
 **************************************************/

/**
 * @brief Initializes the C3D_PhysicsStack object. If out is NULL, it will crash.
 * @param[in,out]     out    C3D_PhyiscsStack object to be initialized.
 */
void PhysicsStack_Init(C3D_PhysicsStack* out);

/**
 * @brief Releases the C3D_PhysicsStack object. If out is NULL or if the C3D_PhysicsStack object is not empty, it will crash/assert failure.
 * @param[in]    in     Check/Assert the object if empty.  
 */
void PhysicsStack_Free(C3D_PhysicsStack* in);

/**
 * @brief Allocates new memory to the C3D_PhysicsStack object. If stack is NULL, it will crash.
 * @param[in,out]   stack      The C3D_PhysicsStack object to assign the allocated memory to.
 * @param[in]       newSize    Specify the new size of the allocated memory.
 * @return Pointer to the allocated memory.
 */
void* PhysicsStack_Allocate(C3D_PhysicsStack* stack, unsigned int newSize);

/**
 * @brief Releases the memory from the C3D_PhysicsStack object. If stack is NULL, it will crash.
 * @param[in,out]     stack       The C3D_PhysicsStack object to release memory from.
 * @param[in]         data        The pointer that the C3D_PhysicsStack object needs to reference to release.
 */
void PhysicsStack_Deallocate(C3D_PhysicsStack* stack, void* data);

/**
 * @brief Initializes the C3D_PhysicsHeap object.
 * @param[in,out]     out     The C3D_PhysicsHeap object to be initialized.
 */
void PhysicsHeap_Init(C3D_PhysicsHeap* out);

/**
 * @brief Releases the C3D_PhysicsHeap object.
 * @param[in,out]      out     The C3D_PhysicsHeap object to be released.
 */
void PhysicsHeap_Free(C3D_PhysicsHeap* out);

/**
 * @brief Allocates new memory to the C3D_PhysicsHeap object. If heap is NULL, it will crash.
 * @param[in,out]      heap       The C3D_PhysicsHeap object to assign the allocated memory to.
 * @param[in]          newSize    Specify the new size of the allocated memory.
 * @return Pointer to the allocated memory.
 */
void* PhysicsHeap_Allocate(C3D_PhysicsHeap* heap, unsigned int newSize);

/**
 * @brief Releases the memory from the C3D_PhysicsHeap object. If heap is NULL, it will crash.
 * @param[in,out]      heap      The C3D_PhysicsHeap object to release allocated memory from.
 * @param[in]          data      The pointer that the C3D_PhysicsHeap object needs to reference to release.
 */
void PhysicsHeap_Deallocate(C3D_PhysicsHeap* heap, void* data);

/**
 * @brief Initializes the C3D_PhysicsPage object. If out is NULL, it will crash.
 * @param[in,out]     out               The resulting C3D_PhysicsPage object.
 * @param[in]         elementSize       The size of each element intended for initialization.
 * @param[in]         elementsPerPage   The size of elements per page.
 */
void PhysicsPage_Init(C3D_PhysicsPage* out, unsigned int elementSize, unsigned int elementsPerPage);

/**
 * @brief Releases the memory from the C3D_PhysicsPage object. This is also used for clearing the C3D_PhysicsPage object. If out is NULL, it will crash.
 * @param[in,out]     out     The resulting C3D_PhysicsPage to be released.
 */
void PhysicsPage_Free(C3D_PhysicsPage* out);

/**
 * @brief Allocates new memory to the C3D_PhysicsPage object. if pageAllocator is NULL, it will crash.
 * @param[in,out]    pageAllocator    The C3D_PhysicsPage object. Here, this object handles the allocation of pages.
 * @return The page data that was most recently modified (allocated/deallocated).
 */
void* PhysicsPage_Allocate(C3D_PhysicsPage* pageAllocator);

/**
 * @brief Releases the allocated memory from the C3D_PhysicsPage object. If pageAllocator is NULL, it will crash.
 * @param[in,out]   pageAllocator    The C3D_PhysicsPage object to release the allocated memory from.
 * @param[in]       data             The pointer to the data that the C3D_PhysicsPage is referencing from to release.
 */
void PhysicsPage_Deallocate(C3D_PhysicsPage* pageAllocator, void* data);

/**************************************************
 * Transform Helper Functions. (Transform)
 **************************************************/

/**
 * @brief Copies data from one to the other.
 * @param[out]   out   The destination to copy to.
 * @param[in]    in    The source to copy from.
 */
static inline void Transform_Copy(C3D_Transform* out, C3D_Transform* in)
{
	*out = *in;
}

/**
 * @brief Multiplies the C3D_Transform objects together. Breaks them into rotation matrix multiplication and matrix translation.
 * @param[out]      out      The resulting C3D_Transform object.
 * @param[in]       lhs      The C3D_Transform object operand to be multiplied on the left-hand side of the operator.
 * @param[in]       rhs      The C3D_Transform object operand to be multiplied on the right-hand side of the operator.
 */
static inline void Transform_Multiply(C3D_Transform* out, const C3D_Transform* lhs, const C3D_Transform* rhs)
{
	C3D_Transform temp;
	if (out == lhs || out == rhs)
	{
		Transform_Multiply(&temp, lhs, rhs);
		Transform_Copy(out, &temp);
		return;
	}
	Mtx_Multiply(&temp.rotation, &lhs->rotation, &rhs->rotation);
	temp.position = FVec3_Add(Mtx_MultiplyFVec3(&temp.rotation, rhs->position), lhs->position);
	temp.position.w = 1.0f;
	Transform_Copy(out, &temp);
}

/**
 * @brief First transposes the rotation matrix, then multiplies the rotation matrix with the vector.
 * @param[out]       out                The resulting C3D_FVec vector.
 * @param[in]        rotationMatrix     The rotation matrix, untransposed.
 * @param[in]        vector             The C3D_FVec vector to be multiplied.
 */
static inline void Transform_MultiplyTransposeFVec(C3D_FVec* out, const C3D_Mtx* rotationMatrix, const C3D_FVec* vector)
{
	C3D_Mtx transpose;
	Mtx_Copy(&transpose, rotationMatrix);
	Mtx_Transpose(&transpose);
	*out = Mtx_MultiplyFVec3(&transpose, *vector);
}

/**
 * @brief First obtain the difference from the vector to the transform's position, then multiply the transpose of the transform's rotation matrix with the difference. 
 *        Shorthand version of multiplying the rotation matrix from the C3D_Transform with the vector, relative to the C3D_Transform position.
 * @param[out]    out           The resulting C3D_FVec vector.
 * @param[in]     transform     The C3D_Transform object to work with.
 * @param[in]     vector        The C3D_FVec vector relative to the C3D_Transform object's position.
 */
static inline void Transform_MultiplyTransformFVec(C3D_FVec* out, const C3D_Transform* transform, const C3D_FVec* vector)
{
	C3D_FVec difference = FVec3_Subtract(*vector, transform->position);
	Transform_MultiplyTransposeFVec(out, &transform->rotation, &difference);
}

/**************************************************
 * Box Helper Functions. (Box)
 **************************************************/

/**
 * @brief Sets user data. Because this is C, you can directly manipulate user data from the C3D_Box object, if you choose so.
 * @note Possibly not needed at all.
 * @param[in,out]     box          The resulting C3D_Box object to store the user data.
 * @param[in]         ptrData      Pointer to the user data store in C3D_Box object.
 */
static inline void Box_SetUserData(C3D_Box* const box, void* ptrData)
{
	box->userData = ptrData;
}

/**
 * @brief Gets user data. Because this is C, you can directly access user data from the C3D_Box object, if you choose so.
 * @note Possibly not needed at all.
 * @param[in]     box       The resulting C3D_Box object to access the user data.
 * @return Pointer to the user data from the C3D_Box object.
 */
static inline void* Box_GetUserData(C3D_Box* const box)
{
	return box->userData;
}

/**
 * @brief Sets the C3D_Box object sensor flag.
 * @note Possibly not needed at all.
 * @param[in,out]     box      The resulting C3D_Box object.
 * @param[in]         flag     The new sensor flag value.
 */
static inline void Box_SetSensorFlag(C3D_Box* const box, bool flag)
{
	box->sensor = flag;
}

/**
 * @brief Cast a ray
 * @note RandyGaul: The entire function performs box to ray and finds the hit point. Using the transpose lets one solve ray to AABB and still get the 
 *       correct results. Ray to AABB is easier than ray to OBB.
 */
bool Box_Raycast(C3D_Box* box, const C3D_Transform* transform, C3D_RaycastData* const raycastData);

/**
 * @brief Using the given C3D_Box object data, create a C3D_AABB object that encapsulate the C3D_Box data.
 * @param[out]     aabb           The resulting C3D_AABB object from the C3D_Box data.
 * @param[in]      box            The C3D_Box object to derive C3D_AABB object from.
 * @param[in]      transform      The C3D_Transform object's transform properties to convert from local transform space to world transform space.
 */
void Box_ComputeAABB(C3D_AABB* const aabb, C3D_Box* const box, const C3D_Transform* transform);

/**
 * @brief Using the given C3D_Box data, compute and obtain the mass.
 * @param[out]   out     The resulting C3D_MassData object.
 * @param[in]    box     The C3D_Box data to compute.
 */
void Box_ComputeMass(C3D_MassData* const out, C3D_Box* const box);

//bool Box_Compare(C3D_Box* this, C3D_Box* other)
//{
//	if (this->broadPhaseIndex != other->broadPhaseIndex)
//		return false;
//	if (this->body->flags != this->body->flags)
//		return false;
//	if (this != other)
//		return false;
//	return true;
//}

/**
 * @brief Obtain the total net force of the combined friction exerted on both C3D_Box objects.
 * @note Friction mixing. The idea is to allow a very low friction value to drive down the mixing result. Example: anything slides on ice.
 * @param[in]        A           One of the C3D_Box objects to measure.
 * @param[in]        B           One of the other C3D_Box objects to measure.
 * @return The combined friction applied to both C3D_Box objects. 
 */
static inline float Box_MixFriction(const C3D_Box* A, const C3D_Box* B)
{
	return sqrtf(A->friction * B->friction);
}

/**
 * @brief Obtain the total net coefficient of restitution from both C3D_Box objects. Restitution is how much of the kinetic energy remains for the objects to 
 *        rebound from one another vs. how much is lost as heat.
 * @note Restitution mixing. The idea is to use the maximum bounciness, so bouncy objects will never not bounce during collisions.
 * @param[in]         A           One of the C3D_Box objects to measure.
 * @param[in]         B           One of the other C3D_Box objects to measure.
 * @return The combined coefficient of restitution from both C3D_Box objects.
 */
static inline float Box_MixRestitution(const C3D_Box* A, const C3D_Box* B)
{
	return (A->restitution > B->restitution ? A->restitution : B->restitution);
}

/**************************************************
 * Physics Body Functions. (Body)
 **************************************************/

/**
 * @brief Checks of bodies of both C3D_Body objects can collide with each other.
 * @param[in]    this       The first C3D_Body to check.
 * @param[in]    other      The second C3D_Body to check.
 * @return True if both C3D_Body objects can collide. False, if otherwise.
 */
bool Body_CanCollide(C3D_Body* this, const C3D_Body* other);

/**
 * @brief Sets the C3D_Body to be in its Awake state.
 * @param[in,out]     body     The resulting C3D_Body object to set to the Awake state.
 */
void Body_SetAwake(C3D_Body* body);

/**
 * @brief Checks if C3D_Body is in Awake state.
 * @param[in,out]     body     The resulting C3D_Body object to check.
 */
static inline bool Body_IsAwake(C3D_Body* body) 
{
	return body->flags & BodyFlag_Awake ? true : false;
}

/**************************************************
 * Broadphase Functions (Broadphase)
 **************************************************/

/**
 * @brief Initializes the C3D_Broadphase object.
 * @param[in,out]   out                The resulting C3D_Broadphase object to initialize.
 * @param[in]       contactManager     The C3D_ContactManager object to initialize with.
 */
void Broadphase_Init(C3D_Broadphase* out, C3D_ContactManager* const contactManager);

/**
 * @brief Releases the C3D_Broadphase object.
 * @param[in,out]     out      The resulting C3D_Broadphase object to release.
 */
void Broadphase_Free(C3D_Broadphase* out);

/**
 * @brief Inserts the index of the C3D_DynamicAABBTreeNode node object into the broadphase, and marking the node "moved".
 * @param[in,out]    broadphase        The resulting C3D_Broadphase object for moving the index value in the move buffer.
 * @param[in]        index             The C3D_DynamicAABBTreeNode node index in the C3D_DynamicAABBTree tree of the broadphase.
 */
void Broadphase_BufferMove(C3D_Broadphase* broadphase, int index);

/**
 * @brief Inserts a C3D_Box object containing the following C3D_AABB object.
 * @param[in,out]     broadphase        The resulting C3D_Broadphase object.
 * @param[in]         box               The C3D_Box object to insert into the broadphase's dynamic tree.
 * @param[in]         aabb              The C3D_AABB object to write it in.
 */
void Broadphase_InsertBox(C3D_Broadphase* broadphase, C3D_Box* box, C3D_AABB* const aabb);

/**
 * @brief Removes a C3D_Box object from the tree node of the broadphase.
 * @param[in,out]  broadphase        The resulting C3D_Broadphase object, with the C3D_Box object removed.
 * @param[in]      box               The target C3D_Box object to be removed from the broadphase.
 */
void Broadphase_RemoveBox(C3D_Broadphase* broadphase, const C3D_Box* box);

/**
 * @brief A callback function used only for the standard C++ function, std::sort(), in the <algorithm> standard header. Not intended to be used for anything else.
 * @param[in]     lhs       The operand input for qsort().
 * @param[in]     rhs       The operand input for qsort().
 * @return Value for std::sort() to determine priority/order.
 */
bool Broadphase_ContactPairSort(const C3D_ContactPair* lhs, const C3D_ContactPair* rhs);

/**
 * @brief A callback function used only for the standard C function, qsort(), in the <stdlib.h> standard header. Not intended to be used for anything else.
 * @param[in]     a       The operand input for qsort().
 * @param[in]     b       The operand input for qsort().
 * @return Value for qsort() to determine priority/order.
 */
int Broadphase_ContactPairQSort(const void* a, const void* b);

/**
 * @brief Updates and validates any modified changes to the C3D_ContactPair objects.
 * @param[in,out]    broadphase       The resulting C3D_Broadphase object to update/validate the C3D_ContactPair objects from.
 */
void Broadphase_UpdatePairs(C3D_Broadphase* broadphase);

/**
 * @brief Updates the entire C3D_Broadphase object, by updating the C3D_DynamicAABBTree tree object of the broadphase.
 * @param[in,out]      broadphase           The resulting C3D_Broadphase object to update.
 * @param[in]          id                   The C3D_DynamicAABBTreeNode node object to update with the new C3D_AABB object.
 * @param[in]          aabb                 The C3D_AABB object for updating the C3D_DynamicAABBTreeNode node object with.
 */
void Broadphase_Update(C3D_Broadphase* broadphase, int id, const C3D_AABB* aabb);

/**
 * @brief Check for any overlapping C3D_DynamicAABBTreeNode node objects based on the nodes' C3D_AABB boundaries.
 * @param[in,out]       broadphase          The resulting C3D_Broadphase object to test for overlaps.
 * @param[in]           A                   The first C3D_DynamicAABBTreeNode node object's ID to test overlaps with.
 * @param[in]           B                   The second C3D_DynamicAABBTreeNode node object's ID to test overlaps with.
 * @return True if there exists an overlap. False, if otherwise. 
 */
bool Broadphase_CanOverlap(C3D_Broadphase* broadphase, int A, int B);

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
bool Broadphase_TreeCallback(C3D_Broadphase* broadphase, int index);

/**************************************************
 * Dynamic AABB Tree Node Functions (TreeNode)
 **************************************************/

/**
 * @brief Checks to see if the node is a leaf in the C3D_DynamicAABBTree data structure.
 * @param[in]    node     C3D_DynamicAABBTreeNode object to check.
 * @return True if it is a leaf. False, if it is a parent.
 */
static inline bool TreeNode_IsLeaf(const C3D_DynamicAABBTreeNode* const node)
{
	return (node->right == TREENODE_NULL);
}

/**************************************************
 * Dynamic AABB Tree Functions (Tree)
 **************************************************/

/**
 * @brief Adds the index to the list of free C3D_DynamicAABBTreeNode objects available for use. This means the C3D_DynamicAABBTreeNode object and subsequent nodes will be cleared away.
 * @param[in,out]     tree     The resulting C3D_DynamicAABBTree object to clear the nodes in.
 * @param[in]         index    The C3D_DynamicAABBTreeNode object's node ID to start freeing from. Subsequent nodes will be cleared away thereafter.
 */
void Tree_AddToFreeList(C3D_DynamicAABBTree* tree, int index);

/**
 * @brief Allocates a new node. If there are available free nodes to use, it will allocate from that list of free nodes to choose from. If there aren't any, it will allocate new ones on the memory.
 * @param[in,out]     tree      The resulting C3D_DynamicAABBTree to allocate new C3D_DynamicAABBTreeNode object nodes to.
 * @return The node index (ID) of the last allocated C3D_DynamicAABBTreeNode object.
 */
int Tree_AllocateNode(C3D_DynamicAABBTree* tree);

/**
 * @brief Releases the C3D_DynamicAABBTreeNode node by clearing an occupied node of the given index.
 * @param[in,out]     tree           The resulting C3D_DynamicAABBTree object.
 * @param[in]         index          The index of the C3D_DynamicAABBTreeNode node to be cleared away.  
 */
void Tree_DeallocateNode(C3D_DynamicAABBTree* tree, int index);

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
int Tree_Balance(C3D_DynamicAABBTree* tree, int indexA);

/**
 * @brief Balances all C3D_DynamicAABBTreeNode nodes in the C3D_DynamicAABBTree tree.
 * @param[in,out]       tree        The resulting C3D_DynamicAABBTree object with all balanced C3D_DynamicAABBTree nodes, starting from the index.
 * @param[in]           index       The starting C3D_DynamicAABBTreeNode node's index (ID) to begin balancing from.
 */
void Tree_SyncHierarchy(C3D_DynamicAABBTree* tree, int index);

/**
 * @brief Inserts a new C3D_DynamicAABBTreeNode leaf node of the C3D_DynamicAABBTreeNode node index (ID). In other words, inserts a child node at the parent node index ID.
 * @param[in,out]      tree      The resulting C3D_DynamicAABBTree tree with the inserted C3D_DynamicAABBTreeNode node.
 * @param[in]          id        The C3D_DynamicAABBTreeNode node index value to insert the leaf node at, setting the given C3D_DynamicAABBTreeNode node as the parent node.
 */
void Tree_InsertLeaf(C3D_DynamicAABBTree* tree, int id);

/**
 * @brief Removes a C3D_DynamicAABBTreeNode leaf node of the given C3D_DynamicAABBTreeNode node index (ID). In other words, removes a child node from the parent node index ID.
 * @param[in,out]      tree      The resulting C3D_DynamicAABBTree tree with the removed C3D_DynamicAABBTreeNode node.
 * @param[in]          id        The C3D_DynamicAABBTreeNode nodex index value to remove the leaf node at, setting the appropriate parent node.
 */
void Tree_RemoveLeaf(C3D_DynamicAABBTree* tree, int id);

/**
 * @brief Initializes the C3D_DynamicAABBTree object.
 * @param[in,out]      tree         The resulting C3D_DynamicAABBTree tree object.
 */
void Tree_Init(C3D_DynamicAABBTree* tree);

/**
 * @brief Deinitializes the C3D_DynamicAABBTree tree object.
 * @param[in,out]     tree      The resulting C3D_DynamicAABBTree tree object to be released.
 */
void Tree_Free(C3D_DynamicAABBTree* tree);

/**
 * @brief Inserts a new C3D_DynamicAABBTreeNode node object containing the C3D_AABB object and its user data.
 * @param[in,out]        tree      The resulting C3D_DynamicAABBTree tree object.
 * @param[in]            aabb      The C3D_AABB object for the new C3D_DynamicAABBTreeNode node.
 * @param[in]            userData  The user data to insert into the new C3D_DynamicAABBTreeNode node.
 */
int Tree_Insert(C3D_DynamicAABBTree* tree, const C3D_AABB* aabb, void* userData);

/**
 * @brief Removes the given C3D_DynamicAABBTreeNode node object at the given index. This includes removing any child C3D_DynamicAABBTreeNode nodes.
 * @param[in,out]       tree      The resulting C3D_DynamicAABBTree tree object with the specified C3D_DynamicAABBTreeNode node object removed.
 * @param[in]           index     The index of the C3D_DynamicAABBTreeNode node object to be removed, including child C3D_DynamicAABBTreeNode nodes.
 */
void Tree_Remove(C3D_DynamicAABBTree* tree, int index);

/**
 * @brief Obtain the C3D_AABB object from C3D_DynamicAABBTreeNode node of index ID from C3D_DynamicAABBTree tree object.
 * @param[in]         tree           The tree to look for the C3D_DynamicAABBTreeNode node that matches the index ID.
 * @param[in]         id             The C3D_DynamicAABBTreeNode node index ID to look for in the C3D_DynamicAABBTree tree object.
 * @return The C3D_AABB object that matches the above conditions. 
 */
C3D_AABB Tree_GetFatAABB(C3D_DynamicAABBTree* tree, int id);

/**
 * @brief Obtains the user data from the C3D_DynamicAABBTreeNode node stored in the C3D_DynamicAABBTree tree object.
 * @param[in]     tree         The C3D_DynamicAABBTree tree object to look for.
 * @param[in]     id           The C3D_DynamicAABBTreeNode node index ID to look for.
 * @return the C3D_AABB object stored in the C3D_DynamicAABBTreeNode node of index ID in the C3D_DynamicAABBTree tree.
 */
void* Tree_GetUserData(C3D_DynamicAABBTree* tree, int id);

/**
 * @brief Queries for information to retrieve from the C3D_DynamicAABBTree tree.
 * @param[in,out]      tree             The C3D_DynamicAABBTree tree object to query through.
 * @param[in]          broadphase       The C3D_Broadphase object to update.
 * @param[in]          aabb             The C3D_AABB object to validate with.
 */
void Tree_Query(C3D_DynamicAABBTree* tree, C3D_Broadphase* broadphase, const C3D_AABB* aabb);

/**
 * @brief Checks if the C3D_DynamicAABBTree tree object contains any invalid C3D_DynamicAABBTreeNode node positions, and aims to fix it.
 * @param[in,out]         tree              The resulting C3D_DynamicAABBTree tree object with the correct C3D_DynamicAABBTreeNode node positions.
 * @param[in]             index             The index of the C3D_DynamicAABBTreeNode node, for the validation to start from.
 */
void Tree_ValidateStructure(C3D_DynamicAABBTree* tree, int index);

/**
 * @brief Quickly checks if the C3D_DynamicAABBTree tree object itself is intact. Does not include validating the C3D_DynamicAABBTree tree structure in its entirety.
 * @param[in,out]     tree        C3D_DynamicAABBTree object to validate.
 */
void Tree_Validate(C3D_DynamicAABBTree* tree);

/**
 * @brief Updates the C3D_DynamicAABBTree tree.
 * @param[in,out]      tree                  The resulting C3D_DynamicAABBTree tree object.
 * @param[in]          id                    The C3D_DynamicAABBTreeNode node to write the new C3D_AABB object to..
 * @param[in]          aabb                  The C3D_AABB object to replace with the already existing C3D_AABB object, stored previously in the C3D_DynamicAABBTreeNode node with the given ID.
 */
bool Tree_Update(C3D_DynamicAABBTree* tree, int id, const C3D_AABB* aabb);


/**************************************************
 * Contact Manager Functions (Manager)
 **************************************************/

/**
 * @brief Initializes the C3D_ContactManager object with the provided C3D_PhysicsStack memory stack.
 * @param[in,out]    manager          The resulting C3D_ContactManager object.
 * @param[in]        stack            The C3D_PhysicsStack memory stack to initialize the C3D_ContactManager object with.
 */
void Manager_Init(C3D_ContactManager* manager, C3D_PhysicsStack* stack);

/**
 * @brief Adds a C3D_ContactConstraint contact where C3D_Box objects, boxA and boxB, are touching or overlapping each other.
 * @param[in,out]      manager         The resulting C3D_ContactManager object.
 * @param[in]          boxA            The C3D_Box object to check for contacts.
 * @param[in]          boxB            The C3D_Box object to check for contacts.
 */
void Manager_AddConstraint(C3D_ContactManager* manager, C3D_Box* boxA, C3D_Box* boxB);

/**
 * @brief Finds new C3D_ContactConstraints contact objects to work with.
 * @param[in,out]        manager             The resulting C3D_ContactManager manager object.
 */
static inline void Manager_FindNewConstraints(C3D_ContactManager* manager)
{
	Broadphase_UpdatePairs(manager->broadphase);
}

/**
 * @brief Removes the given C3D_ContactConstraint contact from the C3D_ContactManager manager object.
 * @param[in,out]      manager               The resulting C3D_ContactManager manager object.
 * @param[in]          constraint            The C3D_Contact object to remove from the C3D_ContactManager object.
 */
void Manager_RemoveConstraint(C3D_ContactManager* manager, C3D_ContactConstraint* constraint);

/**
 * @brief Removes all C3D_ContactConstraint contacts from the given C3D_Body object.
 * @param[in,out]      manager             The resulting C3D_ContactManager manager object.
 * @param[in]          body                The C3D_Body object whose C3D_ContactConstraints are to be removed from the C3D_ContactManager object.
 */
void Manager_RemoveConstraintsFromBody(C3D_ContactManager* manager, C3D_Body* body);

/**
 * @brief Removes the given C3D_Body object from the given C3D_Broadphase object.
 * @param[in,out]      manager            The resulting C3D_ContactManager manager object.
 * @param[in]          body               The C3D_Body object whose C3D_Box objects are to be removed from the C3D_Broadphase object.
 */
void Manager_RemoveBodyFromBroadphase(C3D_ContactManager* manager, C3D_Body* body);

/**
 * @brief Handles collision checks.
 * @param[in,out]       manager           The resulting C3D_ContactManager manager object.
 */
void Manager_CollisionResponse(C3D_ContactManager* manager);

/**
 * @brief Render the C3D_ContactConstraint objects.
 * @param[in,out]      manager        The resulting C3D_ContactManager manager object.
 */
void Manager_RenderConstraints(C3D_ContactManager* manager);

/**************************************************
 * Contact Manifold Functions (Manifold)
 **************************************************/

/**
 * @brief Sets the C3D_Manifold object to store the C3D_Box pair, boxA and boxB. The C3D_Manifold object is used for solving collisions.
 * @param[in,out]    manifold       The resulting C3D_Manifold to store the C3D_Box pair, A and B.
 * @param[in]        boxA           The first C3D_Box.
 * @param[in]        boxB           The second C3D_Box.
 */
void Manifold_SetPair(C3D_Manifold* manifold, C3D_Box* boxA, C3D_Box* boxB);

/**************************************************
 * Contact Constraints Functions (Constraint)
 **************************************************/

/**
 * @brief To generate contact information, and to solve collisions from a given C3D_Constraint object.
 * @param[in,out]      constraint       A C3D_Constraint object to generate contact information with.
 */
void Constraint_CollisionResponse(C3D_ContactConstraint* constraint);

/**************************************************
 * Contact Solver Functions (Solver)
 **************************************************/

/**
 * @brief Initializes the C3D_ContactSolver object using the given C3D_Island object.
 * @param[out]      solver           The resulting C3D_ContactSolver object.
 * @param[in]       island           The given C3D_Island object to initialize.
 */
void Solver_Init(C3D_ContactSolver* solver, C3D_Island* island);

/**
 * @brief Releases the C3D_ContactSolver object to its original state, by shutting it down.
 * @param[in,out]    solver          The resulting C3D_ContactSolver object.
 */
void Solver_Free(C3D_ContactSolver* solver);

/**
 * @brief Precalculates the C3D_ContactSolver object, so it will be ready when it is being solved. Also, precalculates (Jm^-1)*(Jt) for contact and friction constraints
 * @param[in,out]       solver         The resulting C3D_ContactSolver object.
 * @param[in]           deltaTime      Used for precalculating the bias factor of the contact states in the C3D_ContactSolver object.
 */
void Solver_PreSolve(C3D_ContactSolver* solver, float deltaTime);

/**
 * @brief Calculates the C3D_ContactSolver object.
 * @param[in,out]       solver         The resulting C3D_ContactSolver object.
 */
void Solver_Solve(C3D_ContactSolver* solver);

/**************************************************
 * Clip Vertex Functions (ClipVertex)
 **************************************************/

/**
 * @brief Initializes the C3D_ClipVertex clip vertex.
 * @param[in,out]     clipVertex     The resulting C3D_ClipVertex object.
 */
static inline void ClipVertex_Init(C3D_ClipVertex* clipVertex)
{
	clipVertex->featurePair.key = ~0;
}

/**************************************************
 * Collision Functions (Collision)
 **************************************************/

/**
 * @note 
 * RandyGual: Each collision has two shapes, named by convention, called the incident and the reference shape. The reference shape can be thought 
 * of as the "reference frame" for the collision scenario, and in my code, is also treated as the origin of the "reference space". So a lot of the 
 * code operates from the point of view of the reference shape or the reference shape's face that collided with something on the incident shape.
 */

/**
 * @brief Tracks an axis from a face and checks if the current separation value is positive, else re-adjusts the separation value, current axis, and current axis normal. 
 * @note The Separating Axis Theorem is often used to check for collisions between two simple polygons, thus guessing this function is part of the Separating Axis Theorem. 
 *       If the separation value is larger than the maximum separation value, the separation will become the maximum separation value, the axis will be replaced with the
 *       current axis, and the normal will be replaced with the current axis' normal.
 *       
 * RandyGual: The idea of SAT is to test all axes of potential separation by computing a signed overlap value that represents distance along the axis's normal vector. In 3D 
 *            we deal with polyhedra, spheres and capsules. This means we can have faces that describe planes (with the normal as the axis), or pairs of edges that define a 
 *            cross product, which defines the axis direction. In any case, we find signed overlap values, positive for separating and negative for overlapping. We want to 
 *            find the smallest signed value. If this minimum is positive, no overlap occurred. If negative, this is the axis we want to resolve the collision upon.
 *       
 *       See the following link to understand where the 15 axes were obtained from:
 *       http://gamedev.stackexchange.com/questions/44500/how-many-and-which-axes-to-use-for-3d-obb-collision-with-sat/
 *       
 *       The magnitude of the separation does not determine how strong the collision response will be in a physically based simulation.
 * @param[out]      axis             Pointer to a number representing an axis, from the 1st axis to the 15th axis (probably indexed from 0 to 14). 
 * @param[out]      axisNormal       The memory storing the normal associated with maxSeparation value.
 * @param[out]      maxSeparation    The current maximum separation. 
 * @param[in]       currentAxis      Current axis to evaluate.
 * @param[in]       normal           The current normal corresponding to the axis, currentAxis.
 * @param[in]       separation       Defines the current signed distance of overlap of an axis. It's not a vector nor a minimum.
 * @return True if the separation value is positive. False, if otherwise.
 */
bool Collision_TrackFaceAxis(int* axis, C3D_FVec* axisNormal, float* maxSeparation, int currentAxis, const C3D_FVec* normal, float separation);

/**
 * @brief Sets the reference edge indices and calculates the basis matrix.
 * @param[out]        referenceEdgeIndices            The output indices for the reference edges.
 * @param[out]        basisMatrix                     The rotation matrix to represent the orientation of the reference face.
 * @param[out]        alignedBasisExtent              The extent that's vector-aligned to the Cartesian axes within the space of the basis matrix.     
 * @param[in]         normal                          The normal of the reference face.
 * @param[in]         referenceShapeExtent            The extent of the reference shape.
 * @param[in]         referenceTransform              The reference transformation.
 * @param[in]         separationAxis                  The number representing the axis of separation.
 */
void Collision_ComputeReferenceEdgeAndBasis(u8* referenceEdgeIndices, C3D_Mtx* basisMatrix, C3D_FVec* alignedBasisExtent, C3D_FVec* normal, 
	                                        C3D_FVec* referenceShapeExtent, C3D_Transform* referenceTransform, int separationAxis);

/**
 * @brief Computes the clipping information of the incident shape's face.
 * @param[out]        outClipVertexArray     The array of C3D_ClipVertex objects, to store the clipping information into. Array size must be at least 4 elements.
 * @param[in]         incidentTransform      The local space to world space transformation of the incident shape.
 * @param[in]         extent                 The extent of the incident shape. (I'm assuming this.)
 * @param[in]         normal                 The normal of the incident shape's face.
 */
void Collision_ComputeIncidentFace(C3D_ClipVertex* outClipVertexArray, C3D_Transform* incidentTransform, C3D_FVec* extent, C3D_FVec* normal);

/**
 * @brief The reference face is a rectangle centered at the origin. To clip the incident face (of arbitrary orientation) we can look down the x and z axes of the reference face and 
 *        do one-dimensional clipping routines (i.e. lerp) and perform an orthographic clip. It's just a orthographic clipping routine. So if we look down the y axis (straight into 
 *        the reference face) with an orthographic perspective, we shave away all of the incident face that lay beyond the boundary of the reference face, which is a rectangle. To 
 *        me, this is the most novel and interesting part.
 * @note
 * RandyGual: The purpose of this function is to clip the incident face against the reference face side planes. I know this is jargon to you, but that's OK. 
 *            If someone really wants to know what the jargon is they can look it up without too much trouble, especially in the links I provide around the source code. 
 * @param[out]       outClipVertex        The resulting C3D_ClipVertex to return;
 * @param[in]        sign                 Computes one dimensional dot product. It's the sign of a plane.
 * @param[in]        extentComponent      The extent component of the object's bounding box's extent vector.
 * @param[in]        axis                 The number representing the axis of separation, from Axis 1 to Axis 15 (0 ~ 14).
 * @param[in]        inClipVertex         The C3D_ClipVertex to pass into.
 * @param[in]        clipEdge             Determines which edge of this rectangle are we clipping against.
 * @return The size of the C3D_ClipVertex object.
 */
int Collision_Orthographic(C3D_ClipVertex* outClipVertex, float sign, float extentComponent, int axis, int clipEdge, C3D_ClipVertex* inClipVertex, int inCount);

/**
 * @brief Compute using Sutherland-Hodgman Clipping. See Collision_Orthographic() explanation for more info.
 * @note Resources provided by Randy Gual:
 *       http://www.randygaul.net/2013/10/27/sutherland-hodgman-clipping/
 *       https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm
 * @param[out]      outClipVertices           An array of C3D_ClipVertex vertices, storing results from doing one-dimensional clipping routines. Array size should be 8.
 * @param[out]      outDepths                 An array of floats, storing the vector Z component depth differences of the incident face against the reference face.
 * @param[in]       referenceFacePosition     Reference face's position vertex. (Assuming)
 * @param[in]       extent                    The reference shape's extent.
 * @param[in]       basis                     The reference shape's basis matrix. 
 * @param[in]       clipEdges                 An array of the reference shape's clip edges. Array size should be 4.
 * @param[in]       incident                  An array of the incident shape's C3D_ClipVertex vertices. Array size should be 4.
 * @return The number of incident vertices that are behind the reference shape's face. Value is always >= 0.
 */
int Collision_Clip(C3D_ClipVertex* outClipVertices, float* outDepths, C3D_FVec* referenceFacePosition, C3D_FVec* extent, C3D_Mtx* basis, u8* clipEdges, C3D_ClipVertex* incident);

/**
 * @brief Computes closest points between two lines in 3D space.
 * @note RandyGual: Sometimes the shapes are called reference and incident shapes (by old notation!), and sometimes I just refer to them as A and B. P and Q define a 
 *       line segment by two points. PA-QA, and PB-QB, are two different line segments, one from each shape, A and B.
 *       
 *       See reference (Closest Point of Approach, CPA): http://geomalgorithms.com/a07-_distance.html
 * @param[out]          closestA           The closest point on reference shape, A.
 * @param[out]          closestB           The closest point on incident shape, B.
 * @param[in]           PA                 The P point on line segment, PQ, located on the reference shape, A.
 * @param[in]           QA                 The Q point on line segment, PQ, located on the reference shape, A.
 * @param[in]           PB                 The P point on line segment, PQ, located on the incident shape, B.
 * @param[in]           QB                 The Q point on line segment, PQ, located on the incident shape, B.
 */
void Collision_EdgesContact(C3D_FVec* closestA, C3D_FVec* closestB, C3D_FVec* PA, C3D_FVec* QA, C3D_FVec* PB, C3D_FVec* QB);

/**
 * @brief Computes the support edge from local to world transform of a shape. The support edge goes from point A to point B.
 * @param[out]         pointA              Point A on the line segment that goes from A to B.
 * @param[out]         pointB              Point B on the line segment that goes from A to B.
 * @param[in]          shapeTransform      The local to world transform of a shape we are computing the support of.
 * @param[in]          extent              The shape's extent.
 * @param[in]          normal              The support edge's normal.
 */
void Collision_SupportEdge(C3D_FVec* pointA, C3D_FVec* pointB, C3D_Transform* shapeTransform, C3D_FVec* extent, C3D_FVec* normal);

/**
 * @brief C3D_Box to C3D_Box collision detection and response
 * @note Available Resources:
 *       1. Deriving OBB to OBB Intersection and Manifold Generation, Randy Gual -    http://www.randygaul.net/2014/05/22/deriving-obb-to-obb-intersection-sat/
 *       2. Modeling and Solving Constraints, GDC 2007 Lecture by Erin Catto -        http://box2d.org/files/GDC2007/GDC2007_Catto_Erin_Physics1.ppt
 *       3. Contact Manifolds, GDC 2007 Lecture by Erin Catto -                       http://box2d.org/files/GDC2007/GDC2007_Catto_Erin_Physics2.ppt
 *       4. Box2D Lite version download -                                             http://box2d.org/files/GDC2006/Box2D_Lite.zip
 * @param[out]        manifold         The C3D_Manifold object to generate and store the collision properties for the C3D_Box objects, Box A and Box B.
 * @param[in]         boxA             The first C3D_Box object to collide with.
 * @param[in]         boxB             The second C3D_Box object to collide with.
 */
void Collision_BoxToBox(C3D_Manifold* manifold, C3D_Box* boxA, C3D_Box* boxB);

/**************************************************
 * Island Functions (Island)
 **************************************************/

// TODO: https://github.com/RandyGaul/qu3e/blob/master/src/dynamics/q3Island.cpp

/**************************************************
 * Contact Listener Functions (C++ virtual function, Listener)
 **************************************************/

void Listener_Init(C3D_ContactListener* this);

void Listener_Free(C3D_ContactListener* this);

void Listener_BeginContact(C3D_ContactListener* this, const C3D_ContactConstraint* constraint);

void Listener_EndContact(C3D_ContactListener* this, const C3D_ContactConstraint* constraint);

/**
 * @note For all derived contact listener structs, it is up to the developer(s) to provide their own virtual method tables (VMTs).
 *       They must use the following initialization format given below. After that, it is assigned to the derived contact listener struct's "vmt" variable. 
 */
C3D_ContactListener_FuncTable Listener_Default_VMT = {Listener_Init, Listener_Free, Listener_BeginContact, Listener_EndContact};

/**************************************************
 * Scene Functions (Scene)
 **************************************************/

// TODO: https://github.com/RandyGaul/qu3e/blob/master/src/scene/q3Scene.cpp

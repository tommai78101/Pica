#include "physics.h"

/*
 * BOX.C 
 */

/**
 * @brief Initializes the C3D_BoxParameters box properties. You must manually set the values for restitution, friction, density, and sensor flag.
 * @param[in,out]        parameters          The resulting C3D_BoxParameters box property.
 * @param[in]            transform           The local C3D_Transform transform.
 * @param[in]            extents             The full extent of a C3D_Box box. Each element of the extents will be halved.
 */
void BoxParameters_Init(C3D_BoxParameters* parameters, const C3D_Transform transform, const C3D_FVec extents)
{
	parameters->transform = transform;
	parameters->extent = FVec3_Scale(extents, 0.5f);
	parameters->restitution = 0.0f;
	parameters->friction = 0.0f;
	parameters->density = 0.0f;
	parameters->sensor = false;
}

/**
 * @brief Cast a ray
 * @note RandyGaul: The entire function performs box to ray and finds the hit point. Using the transpose lets one solve ray to AABB and still get the 
 *       correct results. Ray to AABB is easier than ray to OBB.
 * @param[in,out]             box             The resulting C3D_Box object.
 * @param[in,out]             raycastData     The resulting C3D_RaycastData object.
 * @param[in]                 transform       A C3D_Transform used to convert local transform to world transform.
 * @return True if the raycast successfully hits the C3D_Box object. False, if otherwise.
 */
bool Box_Raycast(C3D_Box* box, C3D_RaycastData* const raycastData, const C3D_Transform* transform)
{
	C3D_Transform worldTransform;
	Transform_Multiply(&worldTransform, transform, &box->localTransform);
	C3D_FVec direction = Transform_MultiplyTransposeFVec(&worldTransform.rotation, raycastData->direction);
	C3D_FVec position =	Transform_MultiplyTransformFVec(&worldTransform, raycastData->rayOrigin);
	float minimumTime = 0.0f;
	float maximumTime = raycastData->endPointTime;
	float time0Value;  //Point at t = 0 (unit)
	float time1Value;  //Point at t = 1 (unit)
	C3D_FVec normal0;
	for (int i = 3; i > 0; i--)
	{
		//C3D_FVec is structured as WZYX, so the index goes from 3 -> 2 -> 1 -> break.
		if (fabsf(direction.c[i]) < FLT_EPSILON)
		{
			if (position.c[i] < -box->extent.c[i] || position.c[i] > box->extent.c[i])
				return false;
		}
		else 
		{
			float inverseDirection = 1.0f / direction.c[i];
			float sign = (direction.c[i] >= 0.0f ? 1.0f : -1.0f);
			float extentValue = box->extent.c[i] * sign;
			C3D_FVec normal = {};
			normal.c[i] = -sign;
			
			time0Value = -(extentValue + position.c[i]) * inverseDirection;
			time1Value = (extentValue - position.c[i]) * inverseDirection;
			if (time0Value > minimumTime)
			{
				normal0 = normal;
				minimumTime = time0Value;
			}
			maximumTime = (maximumTime < time1Value ? maximumTime : time1Value);
			if (minimumTime > maximumTime)
				return false;
		}
	}
	raycastData->normal = Mtx_MultiplyFVec3(&worldTransform.rotation, normal0);
	raycastData->timeOfImpact = minimumTime;
	return true;
}

/**
 * @brief Using the given C3D_Box object data, create a C3D_AABB object that encapsulate the C3D_Box data.
 * @param[out]     aabb           The resulting C3D_AABB object from the C3D_Box data.
 * @param[in]      box            The C3D_Box object to derive C3D_AABB object from.
 * @param[in]      transform      The C3D_Transform object's transform properties to convert from local transform space to world transform space.
 */
void Box_ComputeAABB(C3D_AABB* const aabb, C3D_Box* const box, const C3D_Transform* transform)
{
	C3D_Transform worldTransform;
	Transform_Multiply(&worldTransform, transform, &box->localTransform);
	C3D_FVec vectors[8] = 
	{
		FVec3_New(-box->extent.x, -box->extent.y, -box->extent.z),
		FVec3_New(-box->extent.x, -box->extent.y,  box->extent.z),
		FVec3_New(-box->extent.x,  box->extent.y, -box->extent.z),
		FVec3_New(-box->extent.x,  box->extent.y,  box->extent.z),
		FVec3_New( box->extent.x, -box->extent.y, -box->extent.z),
		FVec3_New( box->extent.x, -box->extent.y,  box->extent.z),
		FVec3_New( box->extent.x,  box->extent.y, -box->extent.z),
		FVec3_New( box->extent.x,  box->extent.y,  box->extent.z)
	};
	for (int i = 0; i < 8; i++)
		vectors[i] = Transform_MultiplyTransformFVec(&worldTransform, vectors[i]);
	C3D_FVec minimum = FVec3_New(FLT_MAX, FLT_MAX, FLT_MAX);
	C3D_FVec maximum = FVec3_New(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	for (int i = 0; i < 8; i++)
	{
		minimum = FVec3_Min(minimum, vectors[i]);
		maximum = FVec3_Max(maximum, vectors[i]);
	}
	aabb->min = minimum;
	aabb->max = maximum;
}

/**
 * @brief Using the given C3D_Box data, compute and obtain the mass.
 * @param[out]   out     The resulting C3D_MassData object.
 * @param[in]    box     The C3D_Box data to compute.
 */
void Box_ComputeMass(C3D_MassData* const out, C3D_Box* const box)
{
	float squaredExtentX = 4.0f * box->extent.x * box->extent.x;
	float squaredExtentY = 4.0f * box->extent.y * box->extent.y;
	float squaredExtentZ = 4.0f * box->extent.z * box->extent.z;
	float mass = 8.0f * box->extent.x * box->extent.y * box->extent.z * box->density;
	float newX = (1.0f / 12.0f) * mass * (squaredExtentY + squaredExtentZ);
	float newY = (1.0f / 12.0f) * mass * (squaredExtentX + squaredExtentZ);
	float newZ = (1.0f / 12.0f) * mass * (squaredExtentX + squaredExtentY);
	
	C3D_Mtx inertiaMatrix;
	Mtx_Diagonal(&inertiaMatrix, newX, newY, newZ, 1.0f);
	
	C3D_Mtx transposedRotationMatrix;
	Mtx_Copy(&transposedRotationMatrix, &box->localTransform.rotation);
	Mtx_Transpose(&transposedRotationMatrix);
	
	C3D_Mtx temp;
	Mtx_Multiply(&temp, &inertiaMatrix, &transposedRotationMatrix);
	Mtx_Multiply(&inertiaMatrix, &box->localTransform.rotation, &temp);
	
	C3D_Mtx identity;
	Mtx_Identity(&identity);
	
	float dotPosition = FVec3_Dot(box->localTransform.position, box->localTransform.position);
	Mtx_Scale(&identity, dotPosition, dotPosition, dotPosition);
	
	C3D_Mtx outerProduct;
	Mtx_OuterProduct(&outerProduct, box->localTransform.position, box->localTransform.position);
	
	Mtx_Subtract(&temp, &identity, &outerProduct);
	Mtx_Scale(&temp, mass, mass, mass);
	Mtx_Add(&inertiaMatrix, &inertiaMatrix, &temp);
	
	out->center = box->localTransform.position;
	Mtx_Copy(&out->inertia, &inertiaMatrix);
	out->mass = mass;
}

/**
 * @brief Tests if the given point is within the C3D_Box object's world boundaries.
 * @param[in,out]           box            The resulting C3D_Box object.
 * @param[in]               transform      The C3D_Transform transform from the C3D_Body object's local transform.
 * @param[in]               point          The vertex point to test.
 * @return True, if the given point is within the C3D_Box object's world boundaries. False, if otherwise.
 */
bool Box_TestPoint(C3D_Box* box, C3D_Transform* const transform, const C3D_FVec point)
{
	C3D_Transform world;
	Transform_Multiply(&world, transform, &box->localTransform);
	C3D_FVec point0 = Transform_MultiplyTransformFVec(&world, point);
	for (int i = 0; i < 3; i++)
	{
		float d = point0.c[3-i];
		float extent = box->extent.c[3-i];
		if (d > extent || d < extent)
			return false;
	}
	return true;
}

/**
 * @brief Renders the C3D_Box box.
 * TODO: Box_Render() is unimplemented.
 */
void Box_Render(C3D_Box* box, C3D_Transform* const transform, bool awake)
{
	struct Vertex 
	{
		C3D_FVec position;
		C3D_FVec normal;
	};
	C3D_Transform world;
	Transform_Multiply(&world, transform, &box->localTransform);
	C3D_FVec vertices[8] = {
		FVec3_New( -box->extent.x, -box->extent.y, -box->extent.z ),
		FVec3_New( -box->extent.x, -box->extent.y,  box->extent.z ),
		FVec3_New( -box->extent.x,  box->extent.y, -box->extent.z ),
		FVec3_New( -box->extent.x,  box->extent.y,  box->extent.z ),
		FVec3_New(  box->extent.x, -box->extent.y, -box->extent.z ),
		FVec3_New(  box->extent.x, -box->extent.y,  box->extent.z ),
		FVec3_New(  box->extent.x,  box->extent.y, -box->extent.z ),
		FVec3_New(  box->extent.x,  box->extent.y,  box->extent.z )
	};
	struct Vertex vertexBuffer[36];
	for (int i = 0; i < 36; i += 3)
	{
		C3D_FVec a = Transform_MultiplyTransformFVec(&world, vertices[kBoxIndices[i]]);
		C3D_FVec b = Transform_MultiplyTransformFVec(&world, vertices[kBoxIndices[i+1]]);
		C3D_FVec c = Transform_MultiplyTransformFVec(&world, vertices[kBoxIndices[i+2]]);
		C3D_FVec n = FVec3_Normalize(FVec3_Cross(FVec3_Subtract(b, a), FVec3_Subtract(c, a)));
		
		vertexBuffer[i].position = a;
		vertexBuffer[i].normal = n;
		vertexBuffer[i+1].position = b;
		vertexBuffer[i+1].normal = n;
		vertexBuffer[i+2].position = c;
		vertexBuffer[i+2].normal = n;
	}
	
	C3D_TexEnv* env = C3D_GetTexEnv(0);
	C3D_TexEnvSrc(env, C3D_Both, GPU_PRIMARY_COLOR, 0, 0);
	C3D_TexEnvOp(env, C3D_Both, 0, 0, 0);
	C3D_TexEnvFunc(env, C3D_Both, GPU_REPLACE);

	C3D_AttrInfo* attrInfo = C3D_GetAttrInfo();
	AttrInfo_Init(attrInfo);
	AttrInfo_AddLoader(attrInfo, 0, GPU_FLOAT, 3); //vertices
	AttrInfo_AddLoader(attrInfo, 1, GPU_FLOAT, 3); //normals
	
	C3D_BufInfo* bufferInfo = C3D_GetBufInfo();
	BufInfo_Init(bufferInfo);
	BufInfo_Add(bufferInfo, &vertexBuffer, sizeof(struct Vertex), 2, 0x10);
	
	C3D_DrawArrays(GPU_TRIANGLES, 0, 36);
}


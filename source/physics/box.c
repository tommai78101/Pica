#include <c3d/physics.h>

bool Box_Raycast(C3D_Box* box, const C3D_Transform* transform, C3D_RaycastData* const raycastData)
{
	C3D_Transform worldTransform;
	Transform_Multiply(&worldTransform, transform, &box->localTransform);
	C3D_FVec direction;
	Transform_MultiplyTransposeFVec(&direction, &worldTransform.rotation, &raycastData->direction);
	C3D_FVec position;
	Transform_MultiplyTransformFVec(&position, &worldTransform, &raycastData->rayOrigin);
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
		Transform_MultiplyTransformFVec((vectors + i), &worldTransform, (vectors + i));
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
	Mtx_OuterProduct(&outerProduct, &box->localTransform.position, &box->localTransform.position);
	
	Mtx_Subtract(&temp, &identity, &outerProduct);
	Mtx_Scale(&temp, mass, mass, mass);
	Mtx_Add(&inertiaMatrix, &inertiaMatrix, &temp);
	
	out->center = box->localTransform.position;
	Mtx_Copy(&out->inertia, &inertiaMatrix);
	out->mass = mass;
}

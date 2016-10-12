#include <c3d/physics.h>

bool Collision_TrackFaceAxis(int* axis, C3D_FVec* axisNormal, float* maxSeparation, int currentAxis, const C3D_FVec* normal, float separation)
{
	if (separation > 0.0f)
		return true;
	if (separation > *maxSeparation)
	{
		*maxSeparation = separation;
		*axis = currentAxis;
		*axisNormal = *normal;
	}
	return false;
}

void Collision_ComputeIncidentFace(C3D_ClipVertex* outClipVertexArray, C3D_Transform* incidentTransform, C3D_FVec* extent, C3D_FVec* normal)
{
	C3D_FVec newNormal;
	Transform_MultiplyTransposeFVec(&newNormal, &incidentTransform->rotation, normal);
	newNormal = FVec3_Scale(newNormal, -1.0f);
	C3D_FVec absoluteNormal = FVec3_Abs(newNormal);
	if (absoluteNormal.x > absoluteNormal.y && absoluteNormal.x > absoluteNormal.z)
	{
		if (newNormal.x > 0.0f)
		{
			outClipVertexArray[0].vertex = FVec3_New(extent->x,  extent->y, -extent->z);
			outClipVertexArray[1].vertex = FVec3_New(extent->x,  extent->y,  extent->z);
			outClipVertexArray[2].vertex = FVec3_New(extent->x, -extent->y,  extent->z);
			outClipVertexArray[3].vertex = FVec3_New(extent->x, -extent->y, -extent->z);
			outClipVertexArray[0].featurePair.incomingIncident = 9;
			outClipVertexArray[0].featurePair.outgoingIncident = 1;
			outClipVertexArray[1].featurePair.incomingIncident = 1;
			outClipVertexArray[1].featurePair.outgoingIncident = 8;
			outClipVertexArray[2].featurePair.incomingIncident = 8;
			outClipVertexArray[2].featurePair.outgoingIncident = 7;
			outClipVertexArray[3].featurePair.incomingIncident = 7;
			outClipVertexArray[3].featurePair.outgoingIncident = 9;
		}
		else 
		{
			outClipVertexArray[0].vertex = FVec3_New(-extent->x, -extent->y,  extent->z);
			outClipVertexArray[1].vertex = FVec3_New(-extent->x,  extent->y,  extent->z);
			outClipVertexArray[2].vertex = FVec3_New(-extent->x,  extent->y, -extent->z);
			outClipVertexArray[3].vertex = FVec3_New(-extent->x, -extent->y, -extent->z);
			outClipVertexArray[0].featurePair.incomingIncident = 5;
			outClipVertexArray[0].featurePair.outgoingIncident = 11;
			outClipVertexArray[1].featurePair.incomingIncident = 11;
			outClipVertexArray[1].featurePair.outgoingIncident = 3;
			outClipVertexArray[2].featurePair.incomingIncident = 3;
			outClipVertexArray[2].featurePair.outgoingIncident = 10;
			outClipVertexArray[3].featurePair.incomingIncident = 10;
			outClipVertexArray[3].featurePair.outgoingIncident = 5;
		}
	}
	else if (absoluteNormal.y > absoluteNormal.x && absoluteNormal.y > absoluteNormal.z)
	{
		if (newNormal.y > 0.0f)
		{
			outClipVertexArray[0].vertex = FVec3_New(-extent->x, extent->y,  extent->z);
			outClipVertexArray[1].vertex = FVec3_New( extent->x, extent->y,  extent->z);
			outClipVertexArray[2].vertex = FVec3_New( extent->x, extent->y, -extent->z);
			outClipVertexArray[3].vertex = FVec3_New(-extent->x, extent->y, -extent->z);
			outClipVertexArray[0].featurePair.incomingIncident = 3;
			outClipVertexArray[0].featurePair.outgoingIncident = 0;
			outClipVertexArray[1].featurePair.incomingIncident = 0;
			outClipVertexArray[1].featurePair.outgoingIncident = 1;
			outClipVertexArray[2].featurePair.incomingIncident = 1;
			outClipVertexArray[2].featurePair.outgoingIncident = 2;
			outClipVertexArray[3].featurePair.incomingIncident = 2;
			outClipVertexArray[3].featurePair.outgoingIncident = 3;
		}
		else 
		{
			outClipVertexArray[0].vertex = FVec3_New( extent->x, -extent->y,  extent->z);
			outClipVertexArray[1].vertex = FVec3_New(-extent->x, -extent->y,  extent->z);
			outClipVertexArray[2].vertex = FVec3_New(-extent->x, -extent->y, -extent->z);
			outClipVertexArray[3].vertex = FVec3_New( extent->x, -extent->y, -extent->z);
			outClipVertexArray[0].featurePair.incomingIncident = 7;
			outClipVertexArray[0].featurePair.outgoingIncident = 4;
			outClipVertexArray[1].featurePair.incomingIncident = 4;
			outClipVertexArray[1].featurePair.outgoingIncident = 5;
			outClipVertexArray[2].featurePair.incomingIncident = 5;
			outClipVertexArray[2].featurePair.outgoingIncident = 6;
			
			//RandyGual: These numbers came from a single master drawing, and are fairly arbitrary. They might be wrong, but I took pains to make sure they were correct.
			//Original:
			//outClipVertexArray[3].featurePair.incomingIncident = 5;
			//outClipVertexArray[3].featurePair.outgoingIncident = 6;
			//Modified:
			outClipVertexArray[3].featurePair.incomingIncident = 6;
			outClipVertexArray[3].featurePair.outgoingIncident = 7;
		}   
	}
	else 
	{
		if (newNormal.z > 0.0f)
		{
			outClipVertexArray[0].vertex = FVec3_New(-extent->x,  extent->y, extent->z);
			outClipVertexArray[1].vertex = FVec3_New(-extent->x, -extent->y, extent->z);
			outClipVertexArray[2].vertex = FVec3_New( extent->x, -extent->y, extent->z);
			outClipVertexArray[3].vertex = FVec3_New( extent->x,  extent->y, extent->z);
			outClipVertexArray[0].featurePair.incomingIncident = 0;
			outClipVertexArray[0].featurePair.outgoingIncident = 11;
			outClipVertexArray[1].featurePair.incomingIncident = 11;
			outClipVertexArray[1].featurePair.outgoingIncident = 4;
			outClipVertexArray[2].featurePair.incomingIncident = 4;
			outClipVertexArray[2].featurePair.outgoingIncident = 8;
			outClipVertexArray[3].featurePair.incomingIncident = 8;
			outClipVertexArray[3].featurePair.outgoingIncident = 0;
		}
		else 
		{
			outClipVertexArray[0].vertex = FVec3_New( extent->x, -extent->y, -extent->z);
			outClipVertexArray[1].vertex = FVec3_New(-extent->x, -extent->y, -extent->z);
			outClipVertexArray[2].vertex = FVec3_New(-extent->x,  extent->y, -extent->z);
			outClipVertexArray[3].vertex = FVec3_New( extent->x,  extent->y, -extent->z);
			outClipVertexArray[0].featurePair.incomingIncident = 9;
			outClipVertexArray[0].featurePair.outgoingIncident = 6;
			outClipVertexArray[1].featurePair.incomingIncident = 6;
			outClipVertexArray[1].featurePair.outgoingIncident = 10;
			outClipVertexArray[2].featurePair.incomingIncident = 10;
			outClipVertexArray[2].featurePair.outgoingIncident = 2;
			outClipVertexArray[3].featurePair.incomingIncident = 2;
			outClipVertexArray[3].featurePair.outgoingIncident = 9;
		}
	}
	for (int i = 0; i < 4; i++)
	{
		C3D_FVec tempVertex = Mtx_MultiplyFVec3(&incidentTransform->rotation, outClipVertexArray[i].vertex);
		outClipVertexArray[i].vertex = FVec3_Add(tempVertex, incidentTransform->position);
	}
}

void Collision_ComputeReferenceEdgeAndBasis(u8* referenceEdgeIndices, C3D_Mtx* basisMatrix, C3D_FVec* alignedBasisExtent, C3D_FVec* normal, 
	                                        C3D_FVec* referenceShapeExtent, C3D_Transform* referenceTransform, int separationAxis)
{
	C3D_FVec newNormal;
	Transform_MultiplyTransposeFVec(&newNormal, &referenceTransform->rotation, normal);
	if (separationAxis >= 3)
		separationAxis -= 3;
	switch (separationAxis)
	{
		case 0:
			if (newNormal.x > 0.0f)
			{
				referenceEdgeIndices[0] = 1;
				referenceEdgeIndices[1] = 8;
				referenceEdgeIndices[2] = 7;
				referenceEdgeIndices[3] = 9;
				alignedBasisExtent->x = referenceShapeExtent->y;
				alignedBasisExtent->y = referenceShapeExtent->z;
				alignedBasisExtent->z = referenceShapeExtent->x;
				basisMatrix->r[0] = referenceTransform->rotation.r[1];
				basisMatrix->r[1] = referenceTransform->rotation.r[2];
				basisMatrix->r[2] = referenceTransform->rotation.r[0];
			}
			else 
			{
				referenceEdgeIndices[0] = 11;
				referenceEdgeIndices[1] = 3;
				referenceEdgeIndices[2] = 10;
				referenceEdgeIndices[3] = 5;
				alignedBasisExtent->x = referenceShapeExtent->z;
				alignedBasisExtent->y = referenceShapeExtent->y;
				alignedBasisExtent->z = referenceShapeExtent->x;
				basisMatrix->r[0] = referenceTransform->rotation.r[2];
				basisMatrix->r[1] = referenceTransform->rotation.r[1];
				basisMatrix->r[2] = FVec3_Scale(referenceTransform->rotation.r[0], -1.0f);
			}
			break;
		case 1:
			if (newNormal.y > 0.0f)
			{
				referenceEdgeIndices[0] = 0;
				referenceEdgeIndices[1] = 1;
				referenceEdgeIndices[2] = 2;
				referenceEdgeIndices[3] = 3;
				alignedBasisExtent->x = referenceShapeExtent->z;
				alignedBasisExtent->y = referenceShapeExtent->x;
				alignedBasisExtent->z = referenceShapeExtent->y;
				basisMatrix->r[0] = referenceTransform->rotation.r[2];
				basisMatrix->r[1] = referenceTransform->rotation.r[0];
				basisMatrix->r[2] = referenceTransform->rotation.r[1];
			}
			else 
			{
				referenceEdgeIndices[0] = 4;
				referenceEdgeIndices[1] = 5;
				referenceEdgeIndices[2] = 6;
				referenceEdgeIndices[3] = 7;
				alignedBasisExtent->x = referenceShapeExtent->z;
				alignedBasisExtent->y = referenceShapeExtent->x;
				alignedBasisExtent->z = referenceShapeExtent->y;
				basisMatrix->r[0] = referenceTransform->rotation.r[2];
				basisMatrix->r[1] = FVec3_Scale(referenceTransform->rotation.r[0], -1.0f);
				basisMatrix->r[2] = FVec3_Scale(referenceTransform->rotation.r[1], -1.0f);
			}
			break;
		case 2:
			if (newNormal.z > 0.0f)
			{
				referenceEdgeIndices[0] = 11;
				referenceEdgeIndices[1] = 4;
				referenceEdgeIndices[2] = 8;
				referenceEdgeIndices[3] = 0;
				alignedBasisExtent->x = referenceShapeExtent->y;
				alignedBasisExtent->y = referenceShapeExtent->x;
				alignedBasisExtent->z = referenceShapeExtent->z;
				basisMatrix->r[0] = FVec3_Scale(referenceTransform->rotation.r[1], -1.0f);
				basisMatrix->r[1] = referenceTransform->rotation.r[0];
				basisMatrix->r[2] = referenceTransform->rotation.r[2];
			}
			else 
			{
				referenceEdgeIndices[0] = 6;
				referenceEdgeIndices[1] = 10;
				referenceEdgeIndices[2] = 2;
				referenceEdgeIndices[3] = 9;
				alignedBasisExtent->x = referenceShapeExtent->y;
				alignedBasisExtent->y = referenceShapeExtent->x;
				alignedBasisExtent->z = referenceShapeExtent->z;
				basisMatrix->r[0] = FVec3_Scale(referenceTransform->rotation.r[1], -1.0f);
				basisMatrix->r[1] = FVec3_Scale(referenceTransform->rotation.r[0], -1.0f);
				basisMatrix->r[2] = FVec3_Scale(referenceTransform->rotation.r[2], -1.0f);
			}
			break;
	}
	basisMatrix->r[3] = FVec4_New(0.0f, 0.0f, 0.0f, 1.0f);
}

int Collision_Orthographic(C3D_ClipVertex* outClipVertex, float sign, float extent, int axis, int clipEdge, C3D_ClipVertex* inClipVertex, int inCount)
{
	int outCount = 0;
	C3D_ClipVertex clipVertexA = inClipVertex[inCount - 1];
	for (int i = 0; i < inCount; i++)
	{
		C3D_ClipVertex tempClipVertex;
		C3D_ClipVertex clipVertexB = inClipVertex[i];
		float deltaClipVertexA = sign * clipVertexA.vertex.c[3 - axis] - extent;
		float deltaClipVertexB = sign * clipVertexB.vertex.c[3 - axis] - extent;
		if ((COLLISION_IN_FRONT(deltaClipVertexA) && COLLISION_IN_FRONT(deltaClipVertexB)) || COLLISION_ON(deltaClipVertexA) || COLLISION_ON(deltaClipVertexB))
		{
			assert(outCount < 8);
			outClipVertex[outCount++] = clipVertexB;
		}
		else if (COLLISION_IN_FRONT(deltaClipVertexA) && COLLISION_BEHIND(deltaClipVertexB))
		{
			tempClipVertex.featurePair = clipVertexB.featurePair;
			tempClipVertex.vertex = FVec3_Add(clipVertexA.vertex, FVec3_Scale(FVec3_Subtract(clipVertexB.vertex, clipVertexA.vertex), (deltaClipVertexA / (deltaClipVertexA - deltaClipVertexB))));
			tempClipVertex.featurePair.outgoingReference = clipEdge;
			tempClipVertex.featurePair.outgoingIncident = 0;
			assert(outCount < 8);
			outClipVertex[outCount++] = tempClipVertex;
		}
		else if (COLLISION_IN_FRONT(deltaClipVertexB) && COLLISION_BEHIND(deltaClipVertexA))
		{
			tempClipVertex.featurePair = clipVertexA.featurePair;
			tempClipVertex.vertex = FVec3_Add(clipVertexA.vertex, FVec3_Scale(FVec3_Subtract(clipVertexB.vertex, clipVertexA.vertex), (deltaClipVertexA / (deltaClipVertexA - deltaClipVertexB))));
			tempClipVertex.featurePair.incomingReference = clipEdge;
			tempClipVertex.featurePair.incomingIncident = 0;
			assert(outCount < 8);
			outClipVertex[outCount++] = tempClipVertex;
			assert(outCount < 8);
			outClipVertex[outCount++] = clipVertexB;
		}
		clipVertexA = clipVertexB;
	}
	return outCount;
}

int Collision_Clip(C3D_ClipVertex* outClipVertices, float* outDepths, C3D_FVec* referenceFacePosition, C3D_FVec* extent, C3D_Mtx* basis, u8* clipEdges, C3D_ClipVertex* incident)
{
	int inCount = 4;
	int outCount;
	C3D_ClipVertex in[8];
	C3D_ClipVertex out[8];
	C3D_FVec temp;
	for (int i = 0; i < 4; i++)
	{
		temp = FVec3_Subtract(incident[i].vertex, *referenceFacePosition);
		Transform_MultiplyTransposeFVec(&(in[i].vertex), basis, &temp);
	}
	outCount = Collision_Orthographic(out, 1.0f, extent->x, 0, clipEdges[0], in, inCount);
	if (!outCount)
		return 0;
	inCount = Collision_Orthographic(in, 1.0f, extent->y, 1, clipEdges[1], out, outCount);
	if (!inCount)
		return 0;
	outCount = Collision_Orthographic(out, -1.0f, extent->x, 0, clipEdges[2], in, inCount);
	if (!outCount)
		return 0;
	inCount = Collision_Orthographic(in, -1.0f, extent->y, 1, clipEdges[3], out, outCount);
	outCount = 0;
	for (int i = 0; i < inCount; i++)
	{
		float difference = in[i].vertex.z - extent->z;
		if (difference <= 0.0f)
		{
			outClipVertices[outCount].vertex = FVec3_Add(Mtx_MultiplyFVec3(basis, in[i].vertex), *referenceFacePosition);
			outClipVertices[outCount].featurePair = in[i].featurePair;
			outDepths[outCount++] = difference;
		}
	}
	assert(outCount <= 8);
	return outCount;
}

void Collision_EdgesContact(C3D_FVec* closestA, C3D_FVec* closestB, C3D_FVec* PA, C3D_FVec* QA, C3D_FVec* PB, C3D_FVec* QB)
{
	C3D_FVec lineRayA = FVec3_Subtract(*QA, *PA);
	C3D_FVec lineRayB = FVec3_Subtract(*QB, *PB);
	C3D_FVec rayAB = FVec3_Subtract(*PA, *PB);
	float a = FVec3_Dot(lineRayA, lineRayA);          //a >= 0, always.
	float b = FVec3_Dot(lineRayA, lineRayB);
	float c = FVec3_Dot(lineRayB, lineRayB);          //c >= 0, always.
	float d = FVec3_Dot(lineRayA, rayAB);
	float e = FVec3_Dot(lineRayB, rayAB);
	float denominator = a * c - b * b;                //denominator >= 0, always. Got to make sure denominator isn't 0.0f.
	float TA = (b * e - c * d) / denominator;
	float TB = (b * TA + e) / c;
	*closestA = FVec3_Add(*PA, FVec3_Scale(lineRayA, TA));
	*closestB = FVec3_Add(*PB, FVec3_Scale(lineRayB, TB));
}

void Collision_SupportEdge(C3D_FVec* pointA, C3D_FVec* pointB, C3D_Transform* shapeTransform, C3D_FVec* extent, C3D_FVec* normal)
{
	C3D_FVec newNormal; 
	Transform_MultiplyTransposeFVec(&newNormal, &shapeTransform->rotation, normal);
	C3D_FVec absoluteNormal = FVec3_Abs(newNormal);
	C3D_FVec a;
	C3D_FVec b;
	
	if (absoluteNormal.x > absoluteNormal.y)
	{
		if (absoluteNormal.y > absoluteNormal.z)
		{
			a = FVec3_New(extent->x, extent->y,  extent->z);
			b = FVec3_New(extent->x, extent->y, -extent->z);
		}
		else 
		{
			a = FVec3_New(extent->x,  extent->y, extent->z);
			b = FVec3_New(extent->x, -extent->y, extent->z);
		}
	}
	else 
	{
		if (absoluteNormal.x > absoluteNormal.z)
		{
			a = FVec3_New(extent->x, extent->y,  extent->z);
			b = FVec3_New(extent->x, extent->y, -extent->z);
		}
		else 
		{
			a = FVec3_New( extent->x, extent->y, extent->z);
			b = FVec3_New(-extent->x, extent->y, extent->z);
		}
	}
	float signX = newNormal.x >= 0.0f ? 1.0f : -1.0f;
	float signY = newNormal.y >= 0.0f ? 1.0f : -1.0f;
	float signZ = newNormal.z >= 0.0f ? 1.0f : -1.0f;
	a.x *= signX;
	a.y *= signY;
	a.z *= signZ;
	b.x *= signX;
	b.y *= signY;
	b.z *= signZ;
	Transform_MultiplyTransformFVec(pointA, shapeTransform, &a);
	Transform_MultiplyTransformFVec(pointB, shapeTransform, &b);
}

void Collision_BoxToBox(C3D_Manifold* manifold, C3D_Box* boxA, C3D_Box* boxB)
{
	C3D_Mtx tempMatrix;
	C3D_FVec tempVector;
	
	C3D_Transform transformA = boxA->body->transform;
	C3D_Transform transformB = boxB->body->transform;
	C3D_Transform localA = boxA->localTransform;
	C3D_Transform localB = boxB->localTransform;
	Transform_Multiply(&transformA, &transformA, &localA);
	Transform_Multiply(&transformB, &transformB, &localB);
	C3D_FVec extentA = boxA->extent;
	C3D_FVec extentB = boxB->extent;
	Mtx_Copy(&tempMatrix, &transformA.rotation);
	Mtx_Transpose(&tempMatrix);
	C3D_Mtx collisionMatrix;
	Mtx_Multiply(&collisionMatrix, &tempMatrix, &transformB.rotation);
	C3D_Mtx absoluteCollisionMatrix;
	bool isParallel = false;
	const float kCosTolerance = 1.0e-6f;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			float value = fabsf(collisionMatrix.r[i].c[3-j]);
			absoluteCollisionMatrix.r[i].c[3-j] = value;
			if (value + kCosTolerance >= 1.0f)
				isParallel = true;
		}
	}
	C3D_FVec distanceCentersBA;
	tempVector = FVec3_Subtract(transformB.position, transformA.position);
	Transform_MultiplyTransposeFVec(&distanceCentersBA, &transformA.rotation, &tempVector);
	float separation;
	float maxA = -FLT_MAX;
	float maxB = -FLT_MAX;
	float maxEdge = -FLT_MAX;
	int axisA = ~0;
	int axisB = ~0;
	int axisEdge = ~0;
	C3D_FVec normalA;
	C3D_FVec normalB;
	C3D_FVec normalEdge;
	separation = fabsf(distanceCentersBA.x) - (extentA.x + FVec3_Dot(Mtx_Column3(&absoluteCollisionMatrix, 0), extentB));
	if (Collision_TrackFaceAxis(&axisA, &normalA, &maxA, 0, &transformA.rotation.r[0], separation))
		return;
	separation = fabsf(distanceCentersBA.y) - (extentA.y + FVec3_Dot(Mtx_Column3(&absoluteCollisionMatrix, 1), extentB));
	if (Collision_TrackFaceAxis(&axisA, &normalA, &maxA, 1, &transformA.rotation.r[1], separation))
		return;
	separation = fabsf(distanceCentersBA.z) - (extentA.z + FVec3_Dot(Mtx_Column3(&absoluteCollisionMatrix, 2), extentB));
	if (Collision_TrackFaceAxis(&axisA, &normalA, &maxA, 2, &transformA.rotation.r[2], separation))
		return;
	separation = fabsf(FVec3_Dot(distanceCentersBA, collisionMatrix.r[0])) - (extentB.x + FVec3_Dot(absoluteCollisionMatrix.r[0], extentA));
	if (Collision_TrackFaceAxis(&axisB, &normalB, &maxB, 3, &transformB.rotation.r[0], separation))
		return;
	separation = fabsf(FVec3_Dot(distanceCentersBA, collisionMatrix.r[1])) - (extentB.y + FVec3_Dot(absoluteCollisionMatrix.r[1], extentA));
	if (Collision_TrackFaceAxis(&axisB, &normalB, &maxB, 4, &transformB.rotation.r[1], separation))
		return;
	separation = fabsf(FVec3_Dot(distanceCentersBA, collisionMatrix.r[2])) - (extentB.z + FVec3_Dot(absoluteCollisionMatrix.r[2], extentA));
	if (Collision_TrackFaceAxis(&axisB, &normalB, &maxB, 5, &transformB.rotation.r[2], separation))
		return;
	if (!isParallel)
	{
		float resultA;
		float resultB;
		resultA = extentA.y * absoluteCollisionMatrix.r[0].c[3-2] + extentA.z * absoluteCollisionMatrix.r[0].c[3-1];
		resultB = extentB.y * absoluteCollisionMatrix.r[2].c[3-0] + extentB.z * absoluteCollisionMatrix.r[1].c[3-0];
		separation = fabsf(distanceCentersBA.z * collisionMatrix.r[0].c[3-1] - distanceCentersBA.y * collisionMatrix.r[0].c[3-2]) - (resultA + resultB);
		tempVector = FVec3_New(0.0f, -collisionMatrix.r[0].c[3-2], collisionMatrix.r[0].c[3-1]);
		if (Collision_TrackFaceAxis(&axisEdge, &normalEdge, &maxEdge, 6, &tempVector, separation))
			return;
		resultA = extentA.y * absoluteCollisionMatrix.r[1].c[3-2] + extentA.z * absoluteCollisionMatrix.r[1].c[3-1];
		resultB = extentB.x * absoluteCollisionMatrix.r[2].c[3-0] + extentB.z * absoluteCollisionMatrix.r[0].c[3-0];
		separation = fabsf(distanceCentersBA.z * collisionMatrix.r[1].c[3-1] - distanceCentersBA.y * collisionMatrix.r[1].c[3-2]) - (resultA + resultB);
		tempVector = FVec3_New(0.0f, -collisionMatrix.r[1].c[3-2], collisionMatrix.r[1].c[3-1]);
		if (Collision_TrackFaceAxis(&axisEdge, &normalEdge, &maxEdge, 7, &tempVector, separation))
			return;
		resultA = extentA.y * absoluteCollisionMatrix.r[2].c[3-2] + extentA.z * absoluteCollisionMatrix.r[2].c[3-1];
		resultB = extentB.x * absoluteCollisionMatrix.r[1].c[3-0] + extentB.y * absoluteCollisionMatrix.r[0].c[3-0];
		separation = fabsf(distanceCentersBA.z * collisionMatrix.r[2].c[3-1] - distanceCentersBA.y * collisionMatrix.r[2].c[3-2]) - (resultA + resultB);
		tempVector = FVec3_New(0.0f, -collisionMatrix.r[2].c[3-2], collisionMatrix.r[2].c[3-1]);
		if (Collision_TrackFaceAxis(&axisEdge, &normalEdge, &maxEdge, 8, &tempVector, separation))
			return;
		resultA = extentA.x * absoluteCollisionMatrix.r[0].c[3-2] + extentA.z * absoluteCollisionMatrix.r[0].c[3-0];
		resultB = extentB.y * absoluteCollisionMatrix.r[2].c[3-1] + extentB.z * absoluteCollisionMatrix.r[1].c[3-1];
		separation = fabsf(distanceCentersBA.x * collisionMatrix.r[0].c[3-2] - distanceCentersBA.z * collisionMatrix.r[0].c[3-0]) - (resultA + resultB);
		tempVector = FVec3_New(collisionMatrix.r[0].c[3-2], 0.0f, -collisionMatrix.r[0].c[3-0]);
		if (Collision_TrackFaceAxis(&axisEdge, &normalEdge, &maxEdge, 9, &tempVector, separation))
			return;
		resultA = extentA.x * absoluteCollisionMatrix.r[1].c[3-2] + extentA.z * absoluteCollisionMatrix.r[1].c[3-0];
		resultB = extentB.x * absoluteCollisionMatrix.r[2].c[3-1] + extentB.z * absoluteCollisionMatrix.r[0].c[3-1];
		separation = fabsf(distanceCentersBA.x * collisionMatrix.r[1].c[3-2] - distanceCentersBA.z * collisionMatrix.r[1].c[3-0]) - (resultA + resultB);
		tempVector = FVec3_New(collisionMatrix.r[1].c[3-2], 0.0f, -collisionMatrix.r[1].c[3-0]);
		if (Collision_TrackFaceAxis(&axisEdge, &normalEdge, &maxEdge, 10, &tempVector, separation))
			return;
		resultA = extentA.x * absoluteCollisionMatrix.r[2].c[3-2] + extentA.z * absoluteCollisionMatrix.r[2].c[3-0];
		resultB = extentB.x * absoluteCollisionMatrix.r[1].c[3-1] + extentB.y * absoluteCollisionMatrix.r[0].c[3-1];
		separation = fabsf(distanceCentersBA.x * collisionMatrix.r[2].c[3-2] - distanceCentersBA.z * collisionMatrix.r[2].c[3-0]) - (resultA + resultB);
		tempVector = FVec3_New(collisionMatrix.r[2].c[3-2], 0.0f, -collisionMatrix.r[2].c[3-0]);
		if (Collision_TrackFaceAxis(&axisEdge, &normalEdge, &maxEdge, 11, &tempVector, separation))
			return;
		resultA = extentA.x * absoluteCollisionMatrix.r[0].c[3-1] + extentA.y * absoluteCollisionMatrix.r[0].c[3-0];
		resultB = extentB.y * absoluteCollisionMatrix.r[2].c[3-2] + extentB.z * absoluteCollisionMatrix.r[1].c[3-2];
		separation = fabsf(distanceCentersBA.y * collisionMatrix.r[0].c[3-0] - distanceCentersBA.x * collisionMatrix.r[0].c[3-1]) - (resultA + resultB);
		tempVector = FVec3_New(-collisionMatrix.r[0].c[3-1], collisionMatrix.r[0].c[3-0], 0.0f);
		if (Collision_TrackFaceAxis(&axisEdge, &normalEdge, &maxEdge, 12, &tempVector, separation))
			return;
		resultA = extentA.x * absoluteCollisionMatrix.r[1].c[3-1] + extentA.y * absoluteCollisionMatrix.r[1].c[3-0];
		resultB = extentB.x * absoluteCollisionMatrix.r[2].c[3-2] + extentB.z * absoluteCollisionMatrix.r[0].c[3-2];
		separation = fabsf(distanceCentersBA.y * collisionMatrix.r[1].c[3-0] - distanceCentersBA.x * collisionMatrix.r[1].c[3-1]) - (resultA + resultB);
		tempVector = FVec3_New(-collisionMatrix.r[1].c[3-1], collisionMatrix.r[1].c[3-0], 0.0f);
		if (Collision_TrackFaceAxis(&axisEdge, &normalEdge, &maxEdge, 13, &tempVector, separation))
			return;
		resultA = extentA.x * absoluteCollisionMatrix.r[2].c[3-1] + extentA.y * absoluteCollisionMatrix.r[2].c[3-0];
		resultB = extentB.x * absoluteCollisionMatrix.r[1].c[3-2] + extentB.y * absoluteCollisionMatrix.r[0].c[3-2];
		separation = fabsf(distanceCentersBA.y * collisionMatrix.r[2].c[3-0] - distanceCentersBA.x * collisionMatrix.r[2].c[3-1]) - (resultA + resultB);
		tempVector = FVec3_New(-collisionMatrix.r[1].c[3-1], collisionMatrix.r[1].c[3-0], 0.0f);
		if (Collision_TrackFaceAxis(&axisEdge, &normalEdge, &maxEdge, 14, &tempVector, separation))
			return;
	}
	const float kRelativeTolerance = 0.95f;
	const float kAbsoluteTolerance = 0.01f;
	int axis;
	float maxSeparation;
	C3D_FVec normal;
	float faceMax = maxA > maxB ? maxA : maxB;
	if (kRelativeTolerance * maxEdge > faceMax + kAbsoluteTolerance)
	{
		axis = axisEdge;
		maxSeparation = maxEdge;
		normal = normalEdge;
	}
	else 
	{
		if (kRelativeTolerance * maxB > maxA + kAbsoluteTolerance)
		{
			axis = axisB;
			maxSeparation = maxB;
			normal = normalB;
		}
		else 
		{
			axis = axisA;
			maxSeparation = maxA;
			normal = normalA;
		}
	}
	if (FVec3_Dot(normal, transformB.position) < 0.0f)
		normal = FVec3_Scale(normal, -1.0f);
	assert(axis != ~0);
	if (axis < 6)
	{
		C3D_Transform referenceTransform;
		C3D_Transform incidentTransform;
		C3D_FVec referenceExtent;
		C3D_FVec incidentExtent;
		bool flip;
		if (axis < 3)
		{
			referenceTransform = transformA;
			incidentTransform = transformB;
			referenceExtent = extentA;
			incidentExtent = extentB;
			flip = false;
		}
		else 
		{
			referenceTransform = transformB;
			incidentTransform = transformA;
			referenceExtent = extentB;
			incidentExtent = extentA;
			flip = true;
			normal = FVec3_Scale(normal, -1.0f);
		}
		C3D_ClipVertex incidentVertices[4];
		Collision_ComputeIncidentFace(incidentVertices, &incidentTransform, &incidentExtent, &normal);
		u8 clipEdges[4];
		C3D_Mtx basisMatrix;
		C3D_FVec extent;
		Collision_ComputeReferenceEdgeAndBasis(clipEdges, &basisMatrix, &extent, &normal, &referenceExtent, &referenceTransform, axis);
		C3D_ClipVertex outClipVertices[8];
		float depths[8];
		int outCount = Collision_Clip(outClipVertices, depths, &referenceTransform.position, &extent, &basisMatrix, clipEdges, incidentVertices);
		if (outCount)
		{
			manifold->contactsCount = outCount;
			manifold->normal = flip ? FVec3_Scale(normal, -1.0f) : normal;
			for (int i = 0; i < outCount; i++)
			{
				C3D_Contact* contactIterator = manifold->contacts + i;
				C3D_FeaturePair pair = outClipVertices[i].featurePair;
				if (flip)
				{
					u8 temp = pair.incomingIncident;
					pair.incomingIncident = pair.incomingReference;
					pair.incomingReference = temp;
					temp = pair.outgoingIncident;
					pair.outgoingIncident = pair.outgoingReference;
					pair.outgoingReference = temp;
				}
				contactIterator->featurePair = outClipVertices[i].featurePair;
				contactIterator->position = outClipVertices[i].vertex;
				contactIterator->penetration = depths[i];
			}
		}
	}
	else 
	{
		normal = Mtx_MultiplyFVec3(&transformA.rotation, normal);
		if (FVec3_Dot(normal, FVec3_Subtract(transformB.position, transformA.position)) < 0.0f)
			normal = FVec3_Scale(normal, -1.0f);
		C3D_FVec PA;
		C3D_FVec QA;
		C3D_FVec PB;
		C3D_FVec QB;
		C3D_FVec CA;
		C3D_FVec CB;
		Collision_SupportEdge(&PA, &QA, &transformA, &extentA, &normal);
		tempVector = FVec3_Scale(normal, -1.0f);
		Collision_SupportEdge(&PB, &QB, &transformB, &extentB, &tempVector);
		Collision_EdgesContact(&CA, &CB, &PA, &QA, &PB, &QB);
		manifold->normal = normal;
		manifold->contactsCount = 1;
		C3D_Contact* contactIterator = manifold->contacts;
		C3D_FeaturePair pair;
		pair.key = axis;
		contactIterator->featurePair = pair;
		contactIterator->penetration = maxSeparation;
		contactIterator->position = FVec3_Scale(FVec3_Add(CA, CB), 0.5f);
	}
}

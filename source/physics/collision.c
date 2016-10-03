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

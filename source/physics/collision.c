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


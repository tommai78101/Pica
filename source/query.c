#include "physics.h"

/**
 * @brief Releases / Destroys the C3D_QueryCallback object. By default, it does nothing. Must manually handle resources.
 * @param[in,out]          queryCallback               Expect to pass in a pointer to the C3D_QueryCallback object.
 */
void QueryCallback_Free(C3D_QueryCallback* queryCallback)
{
	//Nothing goes here.
}

/**
 * @brief User-defined callback function. By default, it reports what shape it is.
 * @param[in,out]          queryCallback               Expect to pass in a pointer to the C3D_QueryCallback object.
 * @param[in]              box                The C3D_Box object to report its shape with.
 */
bool QueryCallback_ReportShape(C3D_QueryCallback* queryCallback, C3D_Box* box)
{
	return false;
}

/**
 * @brief Handles the callback acquired.
 * @param[in,out]        wrapper              The resulting C3D_SceneQueryWrapper object for acquiring the callback.
 * @param[in]            id                   The tree node's ID. Used to gather user data from the tree node. 
 */
bool QueryWrapper_TreeCallback(C3D_SceneQueryWrapper* wrapper, unsigned int id)
{
	switch (wrapper->wrapperType)
	{
		case WrapperType_AABB:
		{
			C3D_AABB aabb;
			C3D_Box* box = (C3D_Box*) Tree_GetUserData(wrapper->broadphase->tree, id);
			Box_ComputeAABB(&aabb, box, &box->body->transform);
			if (AABB_CollidesAABB(&wrapper->aabb, &aabb))
				return (wrapper->callback ? wrapper->callback->vmt->ReportShape(wrapper->callback, box) : false);
			return true;
		}
		case WrapperType_Point:
		{
			C3D_Box* box = (C3D_Box*) Tree_GetUserData(wrapper->broadphase->tree, id);
			if (Box_TestPoint(box, &box->body->transform, wrapper->point))
				(wrapper->callback ? wrapper->callback->vmt->ReportShape(wrapper->callback, box) : false);
			return true;
		}
		case WrapperType_Raycast:
		{
			C3D_Box* box = (C3D_Box*) Tree_GetUserData(wrapper->broadphase->tree, id);
			if (Box_Raycast(box, wrapper->raycastData, &box->body->transform))
				return (wrapper->callback ? wrapper->callback->vmt->ReportShape(wrapper->callback, box) : false);
			return true;
		}
		default:
			break;
	}
	return false;
}

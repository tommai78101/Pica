#include "physics.h"

/*
 * QUERY.C
 */

/**
 * @brief This is where you write your own implementations in the QueryCallback_Init() function. This function's purpose is to initialize the C3D_QueryCallback struct object.
 * @note By default, the default virtual method table (v-table) has been given. If you wish to use your own methods and implementations, you need to use the struct object,
 *       C3D_QueryCallback_FuncTable, to store your function pointers, then pass it to the C3D_QueryCallback object's v-table.
 *       For all derived query callback structs, it is up to the developer(s) to provide their own virtual method tables (VMTs).
 *       They must use the following initialization format given below. After that, it is assigned to the derived query callback struct's "vmt" variable.
 *       
 *       	Format: C3D_QueryCallback_FuncTable QueryCallback_Default_VMT = {QueryCallback_Free, QueryCallback_ReportShape};
 *       
 */
void QueryCallback_Init(C3D_QueryCallback* queryCallback, C3D_QueryCallback_FuncTable* const vmt)
{
	queryCallback->vmt = vmt;
}

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

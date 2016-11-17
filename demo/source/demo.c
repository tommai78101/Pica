#include "demo.h"

void Demos_Init(Demo* d, int id)
{
	//The demo code is not meant to be project templates.
	assert(id <= MAX_DEMO_SIZE);
	
	//Initial data setup.
	d->id = id;
	d->type = DemoType_Null;
	d->accumulatedTime = 2000;
	d->count = 0;
	
	//Depending on the ID value, allocate memory to store the functions for the v-tables, 
	//and set demo type.
	switch (id)
	{
		case -1:
			break;
		case 0:
			d->vmt = (Demo_FuncTable*) linearAlloc(sizeof(Demo_FuncTable));
			d->vmt->Init = Demo_DropBox_Init;
			d->vmt->Update = Demo_DropBox_Update;
			d->vmt->Render = Demo_DropBox_Render;
			d->vmt->Shutdown = Demo_DropBox_Shutdown;
			d->type = DemoType_DropBox;
			break;
		case 1:
			break;
		case 2:
			break;
		case 3:
			break;
	}
	
	//Call upon the initialization function associated by type.
	if (d->type != DemoType_Null)
		d->vmt->Init(d);
}

void Demos_Shutdown(Demo* d)
{
	//Shuts down the demo by type.
	if (d->type != DemoType_Null)
		d->vmt->Shutdown(d);
}

void Demos_Update(Demo* d, u64 deltaTime)
{
	//Updates the demo by type.
	if (d->type != DemoType_Null)
		d->vmt->Update(d, deltaTime);
}

void Demos_Render(Demo* d)
{
	//Renders the demo by type.
	if (d->type != DemoType_Null)
		d->vmt->Render(d);
}

// ------------------------------------------------------------------------------------

void Demo_DropBox_Init(Demo* d)
{
	// TODO: Work on the dropbox initialization.
	
	//Initialize body parameters.
	C3D_BodyParameters bodyParameters = {};
	bodyParameters.position = FVec4_New(0.0f, 3.0f, 0.0f, 1.0f);
	bodyParameters.axis = FVec3_New(RandRange(-1.0f, 1.0f), RandRange(-1.0f, 1.0f), RandRange(-1.0f, 1.0f));
	bodyParameters.radianAngle = (acosf(-1.0f)) * RandRange(-1.0f, 1.0f);
	bodyParameters.bodyType = BodyType_Dynamic;
	bodyParameters.angularVelocity = FVec3_New(RandRange(1.0f, 3.0f), RandRange(1.0f, 3.0f), RandRange(1.0f, 3.0f));
	bodyParameters.angularVelocity = FVec3_Scale(bodyParameters.angularVelocity, (RandRange(-1.0f, 1.0f) < 0.0f ? -1.0f : 1.0f));
	bodyParameters.linearVelocity = FVec3_New(RandRange(1.0f, 3.0f), RandRange(1.0f, 3.0f), RandRange(1.0f, 3.0f));             
	bodyParameters.linearVelocity = FVec3_Scale(bodyParameters.linearVelocity, (RandRange(-1.0f, 1.0f) < 0.0f ? -1.0f : 1.0f));
	
	//Create new body
	C3D_Body* body = Scene_CreateBody(&scene, &bodyParameters);
	
	//Initialize box parameters.
	C3D_BoxParameters boxParameters = {};
	boxParameters.restitution = 0.0f;
	
	C3D_Transform transform = {};
	Mtx_Identity(&transform.rotation);
	
	BoxParameters_Init(&boxParameters, transform, FVec3_New(50.0f, 1.0f, 50.0f));
	
	//Add box.
	Body_AddBox(body, &boxParameters);
}

void Demo_DropBox_Update(Demo* d, u64 deltaTime)
{
	d->accumulatedTime += deltaTime;
	if ((d->accumulatedTime / 1000) > 1.0 && d->count < 10)
	{
		DBG("Adding new dropbox.\n");
		
		//Reset accumulated delta time.
		d->accumulatedTime = 0.0;
		
		C3D_BodyParameters bodyParameters = {};
		bodyParameters.position = FVec3_New(0.0f, 3.0f, 0.0f);
		bodyParameters.axis = FVec3_New(RandRange(-1.0f, 1.0f), RandRange(-1.0f, 1.0f), RandRange(-1.0f, 1.0f));
		bodyParameters.radianAngle = M_PI * RandRange(-1.0f, 1.0f);
		bodyParameters.bodyType = BodyType_Dynamic;
		bodyParameters.angularVelocity = FVec3_New(RandRange(1.0f, 3.0f), RandRange(1.0f, 3.0f), RandRange(1.0f, 3.0f));
		bodyParameters.angularVelocity = FVec3_Scale(bodyParameters.angularVelocity, (RandRange(-1.0f, 1.0f) > 0.0f ? 1.0f : -1.0f));
		bodyParameters.linearVelocity = FVec3_New(RandRange(1.0f, 3.0f), RandRange(1.0f, 3.0f), RandRange(1.0f, 3.0f));
		bodyParameters.linearVelocity = FVec3_Scale(bodyParameters.angularVelocity, (RandRange(-1.0f, 1.0f) > 0.0f ? 1.0f : -1.0f));
		
		C3D_Body* body = Scene_CreateBody(&scene, &bodyParameters);
		
		C3D_Transform transform = {};
		Mtx_Identity(&transform.rotation);
		C3D_BoxParameters boxParameters = {};
		BoxParameters_Init(&boxParameters, transform, FVec3_New(1.0f, 1.0f, 1.0f));
		
		Body_AddBox(body, &boxParameters);
		
		d->count++;
	}
}

void Demo_DropBox_Render(Demo* d)
{
}

void Demo_DropBox_Shutdown(Demo* d)
{
	//Remove everything.
	Scene_RemoveAllBodies(&scene);
}

// ------------------------------------------------------------------------------------

/**
 * @brief Renders the C3D_Box box.
 * @param[in]         box              The C3D_Box to render.
 * @param[in]         transform        The C3D_Transform transform to convert from local C3D_Box transform to world transform.
 */
void Box_Render(C3D_Box* box, C3D_Transform* const transform)
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
	struct Vertex vertexBuffer[36] = {};
	for (int i = 0; i < 36; i += 3)
	{
		C3D_FVec a = Transform_MultiplyTransformFVec(&world, vertices[kBoxIndices[i]]);
		C3D_FVec b = Transform_MultiplyTransformFVec(&world, vertices[kBoxIndices[i+1]]);
		C3D_FVec c = Transform_MultiplyTransformFVec(&world, vertices[kBoxIndices[i+2]]);
		C3D_FVec n = FVec3_Normalize(FVec3_Cross(FVec3_Subtract(b, a), FVec3_Subtract(c, a)));
		
		vertexBuffer[i].position = a;
		vertexBuffer[i].position.w = 1.0f;
		vertexBuffer[i].normal = n;
		vertexBuffer[i+1].position = b;
		vertexBuffer[i+1].position.w = 1.0f;
		vertexBuffer[i+1].normal = n;
		vertexBuffer[i+2].position = c;
		vertexBuffer[i+2].position.w = 1.0f;
		vertexBuffer[i+2].normal = n;
	}
	
	//Configure attributes
	C3D_AttrInfo* attrInfo = C3D_GetAttrInfo();
	AttrInfo_Init(attrInfo);
	AttrInfo_AddLoader(attrInfo, 0, GPU_FLOAT, 4);
	AttrInfo_AddLoader(attrInfo, 1, GPU_FLOAT, 4);
	
	C3D_BufInfo* bufferInfo = C3D_GetBufInfo();
	BufInfo_Init(bufferInfo);
	BufInfo_Add(bufferInfo, &vertexBuffer, sizeof(struct Vertex), 2, 0x10);
	
	C3D_Mtx projection;
	Mtx_PerspStereoTilt(&projection, 40.0f * M_PI / 180.0f, 400.0f / 240.0f, 0.01f, 1000.0f, 0.0f, 2.0f, false);
	
	C3D_Mtx modelview;
	Mtx_Identity(&modelview);
	Mtx_Translate(&modelview, world.position.x, world.position.y, world.position.z, false);
	Mtx_Multiply(&modelview, &modelview, &world.rotation);
	
	C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, uLoc_modelView, &modelview);
	C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, uLoc_projection, &projection);
	
	C3D_DrawArrays(GPU_TRIANGLES, 0, 12);
}

/**
 * @brief Renders the C3D_Body to the screen.
 * @param[in]        body       Uses the C3D_Body object that contains the C3D_Box objects needed to render.
 */
void Body_Render(C3D_Body* body)
{
	C3D_Box* box = body->boxes;
	while (box)
	{
		Box_Render(box, &body->transform);
		box = box->next;
	}
}

/**
 * @brief Render the C3D_ContactConstraint objects.
 * @param[in,out]      manager        The resulting C3D_ContactManager manager object.
 */
void Manager_RenderConstraints(C3D_ContactManager* manager)
{
	const C3D_ContactConstraint* constraint = manager->contactList;
	while (constraint)
	{
		const C3D_Manifold* manifold = &constraint->manifold;
		if (!(constraint->flags & ConstraintFlag_Colliding))
		{
			constraint = constraint->next;
			continue;
		}
		for (int j = 0; j < manifold->contactsCount; j++)
		{
			const C3D_Contact* contact = manifold->contacts + j;
			
			//Initialize.
			C3D_FVec buffer[4];
			void* vertexBuffer = (void*) &buffer;
			buffer[0] = FVec3_Add(contact->position, FVec3_Scale(FVec3_New(1.0f, 1.0f, 1.0f), 0.5f));
			buffer[1] = FVec3_Add(contact->position, FVec3_Scale(FVec3_New(1.0f, 1.0f, 1.0f), -0.5f));
			buffer[2] = FVec3_Add(contact->position, FVec3_Scale(manifold->normal, 0.5f));
			buffer[3] = FVec3_Add(contact->position, FVec3_Scale(manifold->normal, -0.5f));
			
			C3D_AttrInfo* attrInfo = C3D_GetAttrInfo();
			AttrInfo_Init(attrInfo);
			AttrInfo_AddLoader(attrInfo, 0, GPU_FLOAT, 4);
			
			C3D_BufInfo* bufferInfo = C3D_GetBufInfo();
			BufInfo_Init(bufferInfo);
			BufInfo_Add(bufferInfo, vertexBuffer, sizeof(C3D_FVec), 1, 0x0);
			
			//Draw lines here.
			C3D_DrawArrays(GPU_TRIANGLE_STRIP, 0, 4);
		}
		constraint = constraint->next;
	}
}

/**
 * @brief Renders the C3D_Scene object.
 * @param[in]        scene         Uses the C3D_Scene object to render.
 */
void Scene_Render(C3D_Scene* scene)
{
	C3D_Body* body = scene->bodyList;
	while (body)
	{
		Body_Render(body);
		body = body->next;
	}
	Manager_RenderConstraints(&scene->contactManager);
}


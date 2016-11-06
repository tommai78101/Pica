#include "demo.h"

void Demos_Init(Demo* d, int id)
{
	//The demo code is not meant to be project templates.
	assert(id <= MAX_DEMO_SIZE);
	
	//Initial data setup.
	d->id = id;
	d->type = DemoType_Null;
	
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

void Demos_Update(Demo* d)
{
	//Updates the demo by type.
	if (d->type != DemoType_Null)
		d->vmt->Update(d);
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
	//Initialize box parameters.
	C3D_BoxParameters boxParameters = {};
	boxParameters.restitution = 0.0f;
	C3D_Transform transform = {};
	Mtx_Identity(&transform.rotation);
	BoxParameters_Init(&boxParameters, transform, FVec3_New(50.0f, 1.0f, 50.0f));
	
	//Initialize body parameters for the floor.
	C3D_BodyParameters floorParameters = {};
	BodyParameters_Init(&floorParameters);
	
	//Initialize the body and adds a box to the body using the box parameters.
	C3D_Body* body = Scene_CreateBody(&scene, &floorParameters);
	Body_AddBox(body, &boxParameters);
}

void Demo_DropBox_Update(Demo* d)
{
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
	C3D_Transform transform = {};
	Mtx_Identity(&transform.rotation);
	C3D_BoxParameters boxParameters = {};
	BoxParameters_Init(&boxParameters, transform, FVec3_New(50.0f, 1.0f, 50.0f));
	
	//Add box.
	Body_AddBox(body, &boxParameters);
}

void Demo_DropBox_Render(Demo* d)
{
	
}

void Demo_DropBox_Shutdown(Demo* d)
{
	//Remove everything.
	Scene_RemoveAllBodies(&scene);
}

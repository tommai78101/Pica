#include "demo.h"

void Demo_Create(Demo* d, int id)
{
	//The demo code is not meant to be project templates.
	assert(id <= MAX_DEMO_SIZE);
	d->type = DemoType_Null;
	switch (id)
	{
		case 0:
			break;
		case 1:
			d->vmt = (Demo_FuncTable*) linearAlloc(sizeof(Demo_FuncTable));
			d->vmt->Init = Demo_DropBox_Init;
			d->vmt->Update = Demo_DropBox_Update;
			d->vmt->Shutdown = Demo_DropBox_Shutdown;
			break;
		case 2:
			break;
		case 3:
			break;
		case 4:
			break;
	}
}

void Demo_Destroy(Demo* d)
{
	linearFree(d->vmt);
}

void Demo_DropBox_Init(Demo* d)
{
	//Total accumulated time.
	d->accumulatedTime = 0;

	//Initialize box parameters.
	C3D_BoxParameters boxParameters = {};
	boxParameters.restitution = 0.0f;
	C3D_Transform transform = {};
	Mtx_Identity(&transform.rotation);
	BoxParameters_Init(&boxParameters, transform, FVec3_New(50.0f, 1.0f, 50.0f));
	
	//Initialize body parameters for the floor.
	C3D_BodyParameters floorParameters;
	BodyParameters_Init(&floorParameters);
	
	//Initialize the body and adds a box to the body using the box parameters.
	C3D_Body* body = Scene_CreateBody(&scene, &floorParameters);
	Body_AddBox(body, &boxParameters);
}

void Demo_DropBox_Update(Demo* d)
{
	d->accumulatedTime += deltaTime;
	if (d->accumulatedTime > 1.0f)
	{
		//Reset accumulated time.
		d->accumulatedTime = 0;
		
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
}

void Demo_DropBox_Shutdown(Demo* d)
{
	//Remove everything.
	Scene_RemoveAllBodies(&scene);
	d->accumulatedTime = 0;
}

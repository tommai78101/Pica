#pragma once

#ifndef DEMO_H
#	define DEMO_H
#	define MAX_DEMO_SIZE 4
#	define DEBUG

#include "common.h"

//Please note that the 3DS screens are sideways (thus 240x400 and 240x320)
static const int SCREEN_WIDTH = 240;
static const int UPPER_SCREEN_HEIGHT = 400;
static const int LOWER_SCREEN_HEIGHT = 320;

//This is used for drawing vertices by the indices.
static const int kBoxIndices[ 36 ] = {
	0, 6, 4,
	0, 2, 6,
	0, 3, 2,
	0, 1, 3,
	2, 7, 6,
	2, 3, 7,
	4, 6, 7,
	4, 7, 5,
	0, 4, 5,
	0, 5, 1,
	1, 5, 7,
	1, 7, 3
};

typedef enum DemoType 
{
	DemoType_Null,
	DemoType_DropBox,
	DemoType_RayPush,
	DemoType_BoxStack,
	DemoType_Test
} DemoType; 

typedef struct Demo 
{
	int id;
	DemoType type;
	struct Demo_FuncTable* vmt;
} Demo;

typedef struct Demo_FuncTable 
{
	void (*Init)(struct Demo*);
	void (*Update)(struct Demo*);
	void (*Render)(struct Demo*);
	void (*Shutdown)(struct Demo*);
} Demo_FuncTable;

extern bool paused;
extern bool singleStep;
extern int demoCount;
extern int currentDemoIterator;
extern C3D_Scene scene;
extern C3D_RenderTarget* mainTarget;

void Demos_Init(Demo* d, int id);
void Demos_Shutdown(Demo* d);
void Demos_Update(Demo* d);
void Demos_Render(Demo* d);

void Demo_DropBox_Init(Demo* d);
void Demo_DropBox_Update(Demo* d);
void Demo_DropBox_Render(Demo* d);
void Demo_DropBox_Shutdown(Demo* d);

/**
 * @brief Renders the C3D_Box box.
 * @param[in]         box              The C3D_Box to render.
 * @param[in]         transform        The C3D_Transform transform to convert from local C3D_Box transform to world transform.
 */
// TODO: Consider whether to remove the rendering functions, to allow the developers come up with their own implementations.
void Box_Render(C3D_Box* box, C3D_Transform* const transform);

/**
 * @brief Renders the C3D_Body to the screen.
 * @param[in]        body       Uses the C3D_Body object that contains the C3D_Box objects needed to render.
 */
// TODO: Consider whether to remove the rendering functions, to allow the developers come up with their own implementations.
void Body_Render(C3D_Body* body);

/**
 * @brief Renders the C3D_ContactConstraint objects.
 * @param[in,out]      manager        The resulting C3D_ContactManager manager object.
 */
// TODO: Consider whether to remove the rendering functions, to allow the developers come up with their own implementations.
void Manager_RenderConstraints(C3D_ContactManager* manager);

/**
 * @brief Renders the C3D_Scene object.
 * @param[in]        scene         Uses the C3D_Scene object to render.
 */
// TODO: Consider whether to remove the rendering functions, to allow the developers come up with their own implementations.
void Scene_Render(C3D_Scene* scene);



#endif

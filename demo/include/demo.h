#pragma once

#ifndef DEMO_H
#	define DEMO_H
#	define MAX_DEMO_SIZE 4

#include "common.h"

//Please note that the 3DS screens are sideways (thus 240x400 and 240x320)
static const int SCREEN_WIDTH = 240;
static const int UPPER_SCREEN_HEIGHT = 400;
static const int LOWER_SCREEN_HEIGHT = 320;

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
	float accumulatedTime;
	DemoType type;
	struct Demo_FuncTable* vmt;
} Demo;

typedef struct Demo_FuncTable 
{
	void (*Init)(struct Demo*);
	void (*Update)(struct Demo*);
	void (*Shutdown)(struct Demo*);
} Demo_FuncTable;

extern bool singleStep;
extern int demoCount;
extern int currentDemo;
extern float deltaTime;
extern struct C3D_Scene scene;

void Demo_Create(Demo* d, int id);
void Demo_Destroy(Demo* d);

void Demo_DropBox_Init(Demo* d);
void Demo_DropBox_Update(Demo* d);
void Demo_DropBox_Shutdown(Demo* d);



#endif

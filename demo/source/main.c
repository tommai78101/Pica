#include "demo.h"

bool paused = false;
bool singleStep = false;
double deltaTime = 0.0;
int demoCount = 0;
int currentDemoIterator = 0;
C3D_Scene scene = {};

void Initialize(Demo* d)
{
	for (int i = 0; i < demoCount; i++)
	{
		Demo* current = d + i;
		Demos_Init(current, i);
	}
	
	//TODO: InitApp() - https://github.com/RandyGaul/qu3e/blob/master/demo/Demo.cpp#L476
}

void Update(Demo* currentDemo, double deltaTime)
{
	while (deltaTime >= 1.0)
	{
		if (!paused)
		{
			Scene_Step(&scene);
			Demos_Update(currentDemo);
		}
		else 
		{
			if (singleStep)
			{
				Scene_Step(&scene);
				Demos_Update(currentDemo);
				singleStep = false;
			}
		}
		deltaTime -= 1.0;
	}
}

void Render(Demo* d)
{
	Scene_Render(&scene);
	if (d->type != DemoType_Null)
		d->vmt->Render(d);
}

void Shutdown(Demo* d)
{
	for (int i = 0; i < demoCount; i++)
	{
		//Calculates iterator.
		Demo* current = d + i;
		
		//Destroy the iterated demo.
		if (current->type != DemoType_Null)
			Demos_Shutdown(current);
		
		//Destroys the v-tables to free up the memory.
		linearFree(current->vmt);
	}
}

//int main()
//{
//	Demo demo[MAX_DEMO_SIZE];
//	demoCount = MAX_DEMO_SIZE;
//	Scene_Init(&scene, deltaTime, FVec3_New(0.0f, -1.0f, 0.0f), 1);
//	
//	gfxInitDefault();
//	
//	Initialize(demo);
//	Demo* currentDemo = &demo[0];
//	Demo* previousDemo = currentDemo;
//
//	// Main loop
//	// TODO: https://github.com/RandyGaul/qu3e/blob/master/demo/Demo.cpp#L371
//	
//	u64 previousTime = osGetTime();
//	u64 currentTime;
//	double deltaTime = 0.0;
//	const double nsPerTick = 1000000000.0 / 60.0;
//	while (aptMainLoop())
//	{
//		gspWaitForVBlank();
//		hidScanInput();
//
//		// Your code goes here
//		if (currentDemo != previousDemo)
//		{
//			if (previousDemo)
//				Demos_Shutdown(previousDemo);
//			if (currentDemo)
//				Demos_Init(currentDemo, currentDemoIterator);
//			previousDemo = currentDemo;
//		}
//		
//		//Markus Persson, "notch", method of calculating delta time.
//		currentTime = osGetTime();
//		deltaTime += ((double)(currentTime - previousTime)) / nsPerTick;
//		previousTime = currentTime;
//		
//		//Update
//		Update(currentDemo, deltaTime);
//		
//		u32 kDown = hidKeysDown();
//		//Exit the while loop.
//		if (kDown & KEY_START)
//			break; 
//		//Change demo.
//		if (kDown & KEY_DOWN)
//		{
//			currentDemoIterator = (currentDemoIterator + 1) % demoCount;
//			currentDemo = &demo[currentDemoIterator];
//			continue;
//		}
//
//		//Clears the buffer.
//		u8* fb = gfxGetFramebuffer(GFX_TOP, GFX_LEFT, NULL, NULL);
//		memset(fb, 0, 240*400*3);
//		
//		//Render code
//		Render(currentDemo);
//
//		// Flush and swap framebuffers
//		gfxFlushBuffers();
//		gfxSwapBuffers();
//	}
//	
//	Shutdown(demo);
//	gfxExit();
//	return 0;
//}



// ----------------------------------------
//     Unit Test.	
// ----------------------------------------
int main()
{
	//All good.
	gfxInitDefault();
	PrintConsole console = {};
	consoleInit(GFX_BOTTOM, &console);
	
	Demo demo[4];
	demoCount = 4;
	Scene_Init(&scene, deltaTime, FVec3_New(0.0f, -1.0f, 0.0f), 1);
	bool check = false;
	while (aptMainLoop())
	{
		if (!check)
		{
			//Currently tracking issues.
			Initialize(demo);
			check = true;
		}
		hidScanInput();
		if (hidKeysDown() & KEY_START)
			break;
		u8* fb = gfxGetFramebuffer(GFX_TOP, GFX_LEFT, NULL, NULL);
		memset(fb, 0xAA, 240*400*3);
		gfxFlushBuffers();
		gfxSwapBuffers();
	}
	Shutdown(demo);
	
	gfxExit();
	return 0;
}

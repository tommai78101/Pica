#include "demo.h"

struct C3D_Scene scene;
float deltaTime = 0.0f;

void Initialize(Demo* d, int size)
{
	for (int i = 0; i < size; i++)
	{
		Demo* currentDemo = d + i;
		Demo_Create(currentDemo, i);
	}
}

void Update()
{
	
}

void Render()
{
	
}

void Shutdown(Demo* d, int size)
{
	for (int i = 0; i < size; i++)
	{
		if (d->type != DemoType_Null)
			linearFree(d->vmt);
	}
}

int main()
{
	Demo demo[MAX_DEMO_SIZE];
	Scene_Init(&scene, deltaTime, FVec3_New(0.0f, -1.0f, 0.0f), 1);
	
	gfxInitDefault();
	
	Initialize(demo, MAX_DEMO_SIZE);

	// Main loop
	while (aptMainLoop())
	{
		gspWaitForVBlank();
		hidScanInput();

		// Your code goes here
		Update();
		
		u32 kDown = hidKeysDown();
		if (kDown & KEY_START)
			break; // break in order to return to hbmenu

		//Clears the buffer.
		u8* fb = gfxGetFramebuffer(GFX_TOP, GFX_LEFT, NULL, NULL);
		memset(fb, 0, 240*400*3);
		
		//Render code
		Render();

		// Flush and swap framebuffers
		gfxFlushBuffers();
		gfxSwapBuffers();
	}
	
	Shutdown(demo, MAX_DEMO_SIZE);

	gfxExit();
	return 0;
}

#include "demo.h"
#include "vshader_shbin.h"

bool paused = false;
bool singleStep = false;
double deltaTime = 0.0;
int demoCount = 0;
int currentDemoIterator = 0;
C3D_Scene scene = {};
C3D_RenderTarget* mainTarget = NULL;

static DVLB_s* vshader_dvlb;
static shaderProgram_s program;
static int uLoc_projection, uLoc_modelView;

static C3D_LightEnv lightEnv;
static C3D_Light light;
static C3D_LightLut lut_Phong;

void Initialize(Demo* d)
{
	// Load the vertex shader, create a shader program and bind it
	vshader_dvlb = DVLB_ParseFile((u32*)vshader_shbin, vshader_shbin_size);
	shaderProgramInit(&program);
	shaderProgramSetVsh(&program, &vshader_dvlb->DVLE[0]);
	C3D_BindProgram(&program);

	// Get the location of the uniforms
	uLoc_projection   = shaderInstanceGetUniformLocation(program.vertexShader, "projection");
	uLoc_modelView    = shaderInstanceGetUniformLocation(program.vertexShader, "modelView");

	//Configure attributes
	C3D_AttrInfo* attrInfo = C3D_GetAttrInfo();
	AttrInfo_Init(attrInfo);
	AttrInfo_AddLoader(attrInfo, 0, GPU_FLOAT, 3);
	AttrInfo_AddLoader(attrInfo, 1, GPU_FLOAT, 3);
	
	
	// Configure the first fragment shading substage to blend the fragment primary color
	// with the fragment secondary color.
	// See https://www.opengl.org/sdk/docs/man2/xhtml/glTexEnv.xml for more insight
	C3D_TexEnv* env = C3D_GetTexEnv(0);
	C3D_TexEnvSrc(env, C3D_Both, GPU_FRAGMENT_PRIMARY_COLOR, GPU_FRAGMENT_SECONDARY_COLOR, 0);
	C3D_TexEnvOp(env, C3D_Both, 0, 0, 0);
	C3D_TexEnvFunc(env, C3D_Both, GPU_ADD);

	static const C3D_Material material =
	{
		{ 0.2f, 0.2f, 0.2f }, //ambient
		{ 0.4f, 0.4f, 0.4f }, //diffuse
		{ 0.8f, 0.8f, 0.8f }, //specular0
		{ 0.0f, 0.0f, 0.0f }, //specular1
		{ 0.0f, 0.0f, 0.0f }, //emission
	};

	C3D_LightEnvInit(&lightEnv);
	C3D_LightEnvBind(&lightEnv);
	C3D_LightEnvMaterial(&lightEnv, &material);

	LightLut_Phong(&lut_Phong, 30);
	C3D_LightEnvLut(&lightEnv, GPU_LUT_D0, GPU_LUTINPUT_LN, false, &lut_Phong);

	C3D_FVec lightVec = { { 1.0, -0.5, 0.0, 0.0 } };

	C3D_LightInit(&light, &lightEnv);
	C3D_LightColor(&light, 1.0, 1.0, 1.0);
	C3D_LightPosition(&light, &lightVec);
	//TODO: InitApp() - https://github.com/RandyGaul/qu3e/blob/master/demo/Demo.cpp#L476
	
	for (int i = 0; i < demoCount; i++)
	{
		Demo* current = d + i;
		Demos_Init(current, i);
	}
}

void Update(Demo* currentDemo)
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
	shaderProgramFree(&program);
	DVLB_Free(vshader_dvlb);
}

int main()
{
	gfxInitDefault();
	consoleInit(GFX_BOTTOM, NULL);
	
	C3D_Init(C3D_DEFAULT_CMDBUF_SIZE);
	
	C3D_RenderTarget* target = C3D_RenderTargetCreate(240, 400, GPU_RB_RGB8, GPU_RB_DEPTH24_STENCIL8);
	C3D_RenderTargetSetClear(target, C3D_CLEAR_ALL, CLEAR_COLOR, 0);
	C3D_RenderTargetSetOutput(target, GFX_TOP, GFX_LEFT, DISPLAY_TRANSFER_FLAGS);
	
	Demo demo[MAX_DEMO_SIZE];
	demoCount = MAX_DEMO_SIZE;
	
	Scene_Init(&scene, deltaTime, FVec3_New(0.0f, -1.0f, 0.0f), 1);
	Initialize(demo);
	
	Demo* currentDemo = demo;
	Demo* previousDemo = currentDemo;
	
	int count = 0;
	while (aptMainLoop())
	{
		hidScanInput();

		// Your code goes here
		if (currentDemo != previousDemo)
		{
			if (previousDemo)
				Demos_Shutdown(previousDemo);
			if (currentDemo)
				Demos_Init(currentDemo, currentDemoIterator);
			previousDemo = currentDemo;
		}
		
		//Update
		Update(currentDemo);
		
		u32 kDown = hidKeysDown();
		//Exit the while loop.
		if (kDown & KEY_START)
			break; 
		//Change demo.
		if (kDown & KEY_DOWN)
		{
			currentDemoIterator = (currentDemoIterator + 1) % demoCount;
			currentDemo = &demo[currentDemoIterator];
			continue;
		}

		//Clears the buffer.
		u8* fb = gfxGetFramebuffer(GFX_TOP, GFX_LEFT, NULL, NULL);
		memset(fb, 0xDF, 240*400*3); //Sets a value between 0x0 to 0xFF. It's a byte-based memset().
		
		//Render code
		Render(currentDemo);

		// Flush and swap framebuffers
		gfxFlushBuffers();
		gfxSwapBuffers();
	}
	Shutdown(demo);
	
	C3D_Fini();
	gfxExit();
	return 0;
}
	



	
	

	



// ----------------------------------------
//     Unit Test.	
// ----------------------------------------
//int main()
//{
//	//All good.
//	gfxInitDefault();
//	PrintConsole console = {};
//	consoleInit(GFX_BOTTOM, &console);
//	
//	Demo demo[4];
//	demoCount = 4;
//	Scene_Init(&scene, deltaTime, FVec3_New(0.0f, -1.0f, 0.0f), 1);
//	bool check = false;
//	while (aptMainLoop())
//	{
//		if (!check)
//		{
//			//Currently tracking issues.
//			Initialize(demo);
//			check = true;
//		}
//		hidScanInput();
//		if (hidKeysDown() & KEY_START)
//			break;
//		u8* fb = gfxGetFramebuffer(GFX_TOP, GFX_LEFT, NULL, NULL);
//		memset(fb, 0xAA, 240*400*3);
//		gfxFlushBuffers();
//		gfxSwapBuffers();
//	}
//	Shutdown(demo);
//	
//	gfxExit();
//	return 0;
//}

#pragma once
#ifndef COMMON_H
#	define COMMON_H
#	include <string.h>
#	include <3ds.h>
#	include <citro3d.h>
#	include <physics.h>
#	include <assert.h>
#	include <stdlib.h>
#	include <stdio.h>
#	include <stdarg.h>
#	include "demo.h"

#define CLEAR_COLOR 0x68B0D8FF

#define DISPLAY_TRANSFER_FLAGS \
	(GX_TRANSFER_FLIP_VERT(0) | GX_TRANSFER_OUT_TILED(0) | GX_TRANSFER_RAW_COPY(0) | \
	GX_TRANSFER_IN_FORMAT(GX_TRANSFER_FMT_RGBA8) | GX_TRANSFER_OUT_FORMAT(GX_TRANSFER_FMT_RGB8) | \
	GX_TRANSFER_SCALING(GX_TRANSFER_SCALE_NO))

static inline float RandRange(float l, float h)
{
	float a = (float) rand();
	a /= (float) RAND_MAX;
	a = (h - 1) * a + l;
	return a;
}

static inline void DBG(char* message, ...){
	va_list arg;
	va_start(arg, message);
	vprintf(message, arg);
	va_end(arg);
	gspWaitForVBlank();
}
#endif

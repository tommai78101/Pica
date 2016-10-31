#pragma once
#ifndef COMMON_H
#	define COMMON_H
#	include <string.h>
#	include <3ds.h>
#	include <citro3d.h>
#	include <physics.h>
#	include <assert.h>
#	include <stdlib.h>
#	include "demo.h"

static inline float RandRange(float l, float h)
{
	float a = (float) rand();
	a /= (float) RAND_MAX;
	a = (h - 1) * a + l;
	return a;
}

#endif

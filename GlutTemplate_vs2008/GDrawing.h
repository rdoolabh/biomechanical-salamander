/***********************************************************************

Class that implement drawing functions in OpenGL, using composite meshes.

***********************************************************************/

#pragma once

#ifdef _WIN32
#include <windows.h>
#endif

#include <gl/gl.h>
#include <gl/glu.h>
#include "GL\glut.h"

#include <iostream>

using namespace std;

class GDrawing
{
public:
	GDrawing(void);
	~GDrawing(void);

	static const double pi;

	static void drawCylinder();
	static void drawCone();
	static void drawSquareTex();
	static void drawCube();
	static void drawSphereT();
	static void initTexture();
	static void removeTexture();
	static void drawSphere();
	static void setColor(float r, float g, float b);
	static void plotInstructions();
};

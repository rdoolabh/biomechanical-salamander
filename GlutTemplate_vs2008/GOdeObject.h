/*******************************************************************************
Class to implement ODE objects and operations on them.
*******************************************************************************/

#pragma once

#include "ode/ode.h"
#include "GDrawing.h"

#include <vector>

using namespace std;

class GOdeObject
{
private:
	float R, G, B;				//Color coordinates for this object.

	/************************* Private interface ******************************/
	void drawGeom( dGeomID g, const dReal *position, const dReal *orientation ) const;
	void renderBox( const dReal sides[3], const dReal position[3], const dReal orientation[12] ) const;
	void renderSphere( const dReal radius, const dReal position[3], const dReal orientation[12] ) const;
	void renderCapsule(const dReal radius, const dReal length, const dReal position[3], const dReal orientation[12]) const;

public:
	GOdeObject(void);
	~GOdeObject(void);

	dBodyID body;					//Object's dynamics body.
	vector< dGeomID > geometries;	//Object's geometries potentially attached to its body.
	int userData;					//This allows to store a key number to identify geometries during collisions.

	static void ODEToOpenGLMatrix( const dReal* p, const dReal* R, dReal* M );
	void setObjectColor( float r, float g, float b );
	void drawGeometries() const;
};


/*******************************************************************************
Class that implements all functions related to ODE.
*******************************************************************************/

#pragma once

#include <vector>
#include <assert.h>

#include "ode/ode.h"
#include "GOdeObject.h"
#include "GSalamander.h"

using namespace std;

class GOde
{
private:
	dReal simulationTime;					//Simulator time variables.
	dReal simulationStep;
	vector< GOdeObject > objects;			//Objects subject to dynamics.
	dSpaceID Space;							//A space that defines collisions.
	vector< dJointGroupID > jointGroups;	//Group of joints for objects connected.
	vector< dJointID > joints;				//Joints between bodies.
	vector< GSalamander > salamanders;		//Salamanders living in ODE.
	GOdeObject ground;						//Special object for ground.
	
public:
	GOde(void);
	~GOde(void);

	dReal getSimulationTime();
	void initODE();
	void closeODE();
	void simulationLoop();
	void drawObjects( bool draw = true );
	static void drawGeometries( const vector< GOdeObject >* objectsPointer );
	static void drawJoints( const vector< dJointID >* jointsPointer );
	dJointID getJoint( int id );
	GOdeObject* getObject( int id );
	GSalamander* getSalamander( int id );
};


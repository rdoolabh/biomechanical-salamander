/*******************************************************************************
Class that implements the virtual salamander.
*******************************************************************************/

#pragma once

#include "GOdeObject.h"
#include "ode\ode.h"
#include "GDrawing.h"
#include <vector>
#include <assert.h>

using namespace std;

class GSalamander
{
	struct GSJoint						//Structure that holds all joint data.
	{
		dJointID joint;					//Actual ODE joint object.
		dReal angleDeformation;			//Change of angle with respect to initial joint angle.
		dReal lambda;					//Phase for this joint w.r.t. to the previous one (in body order).
		char type;						//b -> body joint.
	};

	dReal sPosition[3];					//Salamander's head position in real world coordinates.
	vector< GOdeObject > links;			//Salamander's segments.
	vector< GSJoint > gsJoints;			//Salamander's joints.
	dReal freq;							//Neurosignal frequency.
	dReal amplitude;					//Typical neurosignal amplitude.
	dReal amplitudeLeft;				//Amplitudes that allow turning.
	dReal amplitudeRight;				
	bool lander;						//If true, salamander will walk, otherwise it will swim.

public:
	GSalamander(void);
	GSalamander( dVector3 position, dReal f, bool l );
	~GSalamander(void);
	void createSalamander( dWorldID world, dSpaceID space, dJointGroupID jointGroup );
	void computeForces( dReal simulationTime, dReal simulationStep );
	const vector< GOdeObject >* getLinks();
	void getJoints( vector< dJointID >* joints );
};


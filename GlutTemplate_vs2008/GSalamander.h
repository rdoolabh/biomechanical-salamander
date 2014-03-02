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
		dReal lambda;					//Phase for this joint w.r.t. to the previous one (in body order).
		dReal deltaAngle;				//Whole angle that the joint must sweep while M1 or M2 are active (like a target distance).
		dReal M1;						//Stores current activation value for 'muscle 1' on this joint (left muscle in body).
		dReal M2;						//Stores current activation value for 'muscle 2' on this joint (right muscle in body).
		dReal amplitudeLeft;			//Amplitudes that allow turning (defined only for body segments).
		dReal amplitudeRight;
		char type;						//b -> body joint, l to o -> leg joint.
	};

	dReal sPosition[3];					//Salamander's head position in real world coordinates.
	vector< GOdeObject > links;			//Salamander's segments.
	vector< GSJoint > gsJoints;			//Salamander's joints.
	dReal freq;							//Neurosignal frequency.
	dReal amplitude;					//Typical neurosignal amplitude.				
	bool lander;						//If true, salamander will walk, otherwise it will swim.
	dReal bodyMaxAngleDeformation;		//Global max angle deformation for body hinge joints.
	dReal legMaxAngleDeformation;		//Global max angle deformation for legs hinges.
<<<<<<< HEAD
	public: dReal targetDeformationAngle;		//It is defined every time that a change of activation from 0 to >0 happens.
=======
	dReal kneeMaxAngleDeformation;		//Global max angle deformation for knees.
>>>>>>> Last Salamander Working

public:
	GSalamander(void);
	GSalamander( dVector3 position, dReal f, bool l );
	~GSalamander(void);
	void createSalamander( dWorldID world, dSpaceID space, dJointGroupID jointGroup );
	void computeForces( dReal simulationTime, dReal simulationStep );
	const vector< GOdeObject >* getLinks();
	void getJoints( vector< dJointID >* joints );
	void turn( dReal signal );
};


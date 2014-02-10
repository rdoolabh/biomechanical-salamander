#include "GOde.h"

/*******************************************************************************
Special variables that cannot be included inside the GOde class due to problems
with the nearCallBack function, which only requires static elements to work.
*******************************************************************************/
dWorldID World;						//Dynamics world.
dJointGroupID ContactGroup;			//Group of contact joints for collision detection/handling.

/*******************************************************************************
Function to handle potential collisions between geometries.
*******************************************************************************/
void nearCallBack( void *data, dGeomID o1, dGeomID o2 )
{
	int I;					//A temporary index for each contact.
	const int MAX_CONTACTS = 5;

	//Get the dynamics body for each potentially colliding geometry.
	dBodyID b1 = dGeomGetBody( o1 );
	dBodyID b2 = dGeomGetBody( o2 );

	//Create an array of dContact objects to hold the contact joints.
	dContact contacts[ MAX_CONTACTS ];

	//Initialize contact structures.
	for( I = 0; I < MAX_CONTACTS; I++ )
	{
		contacts[I].surface.mode = dContactBounce | dContactSoftCFM;
		contacts[I].surface.mu = dInfinity;		//0: frictionless, dInfinity: never slips.
		contacts[I].surface.mu2 = 0;			//Friction in direction 2 to mu.
		contacts[I].surface.bounce = 0.01;		//0: not bouncy, 1: max. bouncyness.
		contacts[I].surface.bounce_vel = 0.1;	//Minimum incoming velocity for producting bouncyness.
		contacts[I].surface.soft_cfm = 0.01;	//Softness for maintaining joint constraints.
	}

	//Now, do the actual collision test, passing as parameters the address of
	//a dContactGeom structure, and the offset to the next one in the list.
	if( int numc = dCollide( o1, o2, MAX_CONTACTS, &contacts[0].geom, sizeof(dContact) ) )
	{
		//Add contacts detected to the contact joint group.
		for( I = 0; I < numc; I++ )
		{
			//Add contact joint by knowing its world, to a contact group. The last parameter
			//is the contact itself.
			dJointID c = dJointCreateContact( World, ContactGroup, contacts + I );
			dJointAttach( c, b1, b2 );		//Attach two bodies to this contact joint.
		}
	}
}

/*******************************************************************************
Constructor.
*******************************************************************************/
GOde::GOde(void)
{
	simulationTime = 0.0;		//Initialize simulator variables.
	simulationStep = 0.001;

	psi = 0;					//Initial angle change in for joint.
	psi2 = 0;
	psi3 = 0;
}

/*******************************************************************************
Destructor.
*******************************************************************************/
GOde::~GOde(void)
{
}

/*******************************************************************************
Function to get the current simulation time.
*******************************************************************************/
dReal GOde::getSimulationTime()
{
	return simulationTime;
}

/*******************************************************************************
Function to initialize ODE.
*******************************************************************************/
void GOde::initODE()
{
	///////////////// Initializing the ODE general features ////////////////////

	dInitODE();								//Initialize library.
	World = dWorldCreate();					//Crate a new dynamics, empty world.
	Space = dSimpleSpaceCreate(0);			//Create a new space for collision (independent).
	ContactGroup = dJointGroupCreate(0);	//Create a joints container, without specifying size.

	dWorldSetGravity( World, 0.0, 0.0, 0 );	//Add gravity to this World.

	//Define error conrrection constants.
	dWorldSetERP( World, 0.2 );
	dWorldSetCFM( World, 1e-5 );

	//Set the velocity that interpenetrating objects will separate at.
	dWorldSetContactMaxCorrectingVel( World, 0.9 );

	//Set the safety area for contacts to be considered at rest.
	dWorldSetContactSurfaceLayer( World, 0.001 );

	//Automatically disable objects that have come to a rest.
	dWorldSetAutoDisableFlag( World, 1 );

	//Create a collision plane and add it to space. The next parameters are the
	//literal components of ax + by + cz = d.
	dCreatePlane( Space, 0.0, 1.0, 0.0, 0.0 );

	/////////////// Initializing the rigid body 1 in the world /////////////////
	
	GOdeObject o1;
	o1.body = dBodyCreate( World );							//Create a new body and attach it to object.
	dBodySetPosition( o1.body, -0.03, 0.1, 0.0 );			//Assign position to body.
	dBodySetLinearVel( o1.body, 0.0, 0.0, 0.0 );			//Object begins stationary.

	//dMatrix3 Orient;										//Assign orientation.
	//dRFromAxisAndAngle( Orient, 0.0, 1.0, 0.0, 0.0 );
	//dBodySetRotation( o1.body, Orient );

	dBodySetData( o1.body, (void *)0 );						//No data associated with data.

	dMass M;												//Assign mass distribution to body 1.
	dReal sides[] = {0.03, 0.03, 0.01};						//Side lenghts along the x, y, and z axes.
	dMassSetBoxTotal( &M, 0.0177, sides[0], sides[1], sides[2] );		//Use automatic generation of moment of inertia and mass.
	dBodySetMass( o1.body, &M );							//Apply mass distribution to rigid body.

	dGeomID g;
	g = dCreateBox( Space, sides[0], sides[1], sides[2] );	//Create a box geometry.
	o1.geometries.push_back( g );
	dGeomSetBody( o1.geometries[0], o1.body );				//Link geometry with body.

	o1.setObjectColor( 0.4, 0.6, 0.8 );						//Change their color.
	objects.push_back( o1 );								//Add object t list.

	////////////// Initializing the rigid body 2 in the world //////////////////

	GOdeObject o2;
	o2.body = dBodyCreate( World );							//Attach new body to world.
	dBodySetPosition( o2.body, 0.03, 0.1, 0.0 );			//Position.
	dBodySetLinearVel( o2.body, 0.0, 0.0, 0.0 );			//Initial linear velocity.
	dBodySetData( o2.body, (void *)0 );						//Data.
	//dMatrix3 Orient2;
	//dRFromAxisAndAngle( Orient2, 0.0, 1.0, 0.0, 0 );
	//dBodySetRotation( o2.body, Orient2 );					//Orientation.
	dMass M2;
	dMassSetBoxTotal( &M2, 0.0177, sides[0], sides[1], sides[2] );
	dBodySetMass( o2.body, &M2 );							//Mass distribution.
	dGeomID g2;
	g2 = dCreateBox( Space, sides[0], sides[1], sides[2] );	//Object's geometry.
	o2.geometries.push_back( g2 );
	dGeomSetBody( o2.geometries[0], o2.body );				//Link body and geometry in object.
	o2.setObjectColor( 0.8, 0.6, 0.4 );						//Change object's color.
	objects.push_back( o2 );								//Add object to list.

	/////////////// Initializing rigid body 3 in the world /////////////////////

	objects.push_back( GOdeObject() );
	objects[2].body = dBodyCreate( World );					//Attach new body to world.
	dBodySetPosition( objects[2].body, 0.09, 0.1, 0.0 );	//Position.
	dBodySetLinearVel( objects[2].body, 0.0, 0.0, 0.0 );	//Initial linear velocity.
	dBodySetData( objects[2].body, (void *)0 );				//Data.
	dMass M3;
	dMassSetBoxTotal( &M3, 0.0177, sides[0], sides[1], sides[2]);
	dBodySetMass( objects[2].body, &M3 );					//Mass distribution.
	dGeomID g3;
	g3 = dCreateBox( Space, sides[0], sides[1], sides[2] );	//Geometry.
	objects[2].geometries.push_back( g3 );
	dGeomSetBody( objects[2].geometries[0], objects[2].body );	//Link body to geom.
	objects[2].setObjectColor( 0.5, 0.8, 0.5 );				//Change color.

	////////////////// Initializing rigid body 4 in the world //////////////////

	objects.push_back( GOdeObject() );
	objects[3].body = dBodyCreate( World );
	dBodySetPosition( objects[3].body, 0.15, 0.1, 0.0 );
	dBodySetLinearVel( objects[3].body, 0.0, 0.0, 0.0 );
	dBodySetData( objects[3].body, (void *)0 );				//Data.
	dMass M4;
	dMassSetBoxTotal( &M4, 0.0177, sides[0], sides[1], sides[2]);
	dBodySetMass( objects[3].body, &M4 );					//Mass distribution.
	dGeomID g4;
	g4 = dCreateBox( Space, sides[0], sides[1], sides[2] );	//Geometry.
	objects[3].geometries.push_back( g4 );
	dGeomSetBody( objects[3].geometries[0], objects[3].body );	//Link body to geom.
	objects[3].setObjectColor( 0.0, 0.8, 0.5 );				//Change color.

	///////////////// Create hinge joint for both bodies ///////////////////////
	
	jointGroups.push_back( dJointGroupCreate( 0 ) );	//Create a group of joints.
	
	joints.push_back( dJointCreateHinge( World, jointGroups[0] ) );
	dJointAttach( joints[0], o1.body, o2.body );		//Attach objects to hinge joint.
	dJointSetHingeAnchor( joints[0], 0.0, 0.1, 0.0 );	//A point where bodies are joined.
	dJointSetHingeAxis( joints[0], 0.0, 1.0, 0.0 );		//Set the hinge axis.
	
	joints.push_back( dJointCreateHinge( World, jointGroups[0] ) );
	dJointAttach( joints[1], o2.body, objects[2].body );	//Join objects 2 and 3.
	dJointSetHingeAnchor( joints[1], 0.06, 0.1, 0.0 );		//Point where bodies are joined.
	dJointSetHingeAxis( joints[1], 0.0, 1.0, 0.0 );			//Hinge axis.

	joints.push_back( dJointCreateHinge( World, jointGroups[0] ) );
	dJointAttach( joints[2], objects[2].body, objects[3].body );	//Join objects 2 and 3.
	dJointSetHingeAnchor( joints[2], 0.12, 0.1, 0.0 );		//Point where bodies are joined.
	dJointSetHingeAxis( joints[2], 0.0, 1.0, 0.0 );			//Hinge axis.
	
	///////////////////////// Apply external, initial torque ///////////////////
	
}

/*******************************************************************************
Function to clean the ODE system.
*******************************************************************************/
void GOde::closeODE()
{
	dJointGroupDestroy( ContactGroup );		//Remove the contact joints.

	for( unsigned I = 0; I < jointGroups.size(); I++ )
		dJointGroupDestroy( jointGroups[I] );		//Destroy joint attaching both bodies.

	dSpaceDestroy( Space );					//Remove the space and all of its geoms.
	dWorldDestroy( World );					//Destroy all bodies and joints (not in a group).

	//Empty arrays of objects, joints, and joint groups.
	vector< GOdeObject >().swap( objects );
	vector< dJointID >().swap( joints );
	vector< dJointGroupID >().swap( jointGroups );
}

/*******************************************************************************
Function to perform the simulation loop for ODE.
*******************************************************************************/
void GOde::simulationLoop()
{
	//First, accumulate forces and torques in objects (bodies).

	dReal alpha = 0.001;				//Constants to compute torques.
	dReal beta = 0.00015;
	dReal gamma = 5.0;
	dReal delta = 0.0001;

	dReal A = 0.5;						//Amplitude for neuro signals.
	dReal w = 10.0;						//Frequency.

	//////////////////////////////// Joint 1 ///////////////////////////////////

	dReal lambda = 0.0;					//Phase for this joint's first link.
	dReal Ml = A*sin( w*simulationTime - lambda );					//Neural signals.
	dReal Mr = A*sin( w*simulationTime - lambda + GDrawing::pi );

	dReal oldPsi = psi;		//Maintain previous angle (or change of angle from "restlength").
	psi = dJointGetHingeAngle( joints[0] );
	dReal dPsi = (psi - oldPsi) / simulationStep;
	dReal torque = alpha*(Ml - Mr) - beta*(Ml + Mr + gamma)*psi - delta*dPsi;
	
	dJointAddHingeTorque( joints[0], torque );

	//////////////////////////////// Joint 2 ///////////////////////////////////

	dReal lambda2 = 0;	//Phase for this joint's first link.
	dReal Ml2 = A*sin( w*simulationTime - lambda2 );					//Neural signals.
	dReal Mr2 = A*sin( w*simulationTime - lambda2 + GDrawing::pi );

	dReal oldPsi2 = psi2;			//Maintain previous angle (or change of angle from "restlength").
	psi2 = dJointGetHingeAngle( joints[1] );
	dReal dPsi2 = (psi2 - oldPsi2) / simulationStep;
	dReal torque2 = alpha*(Ml2 - Mr2) - beta*(Ml2 + Mr2 + gamma)*psi2 - delta*dPsi2;
	
	dJointAddHingeTorque( joints[1], torque2 );

	//////////////////////////////// Joint 3 ///////////////////////////////////

	dReal lambda3 = 0;	//Phase for this joint's first link.
	dReal Ml3 = A*sin( w*simulationTime - lambda3 );					//Neural signals.
	dReal Mr3 = A*sin( w*simulationTime - lambda3 + GDrawing::pi );

	dReal oldPsi3 = psi3;			//Maintain previous angle (or change of angle from "restlength").
	psi3 = dJointGetHingeAngle( joints[2] );
	dReal dPsi3 = (psi3 - oldPsi3) / simulationStep;
	dReal torque3 = alpha*(Ml3 - Mr3) - beta*(Ml3 + Mr3 + gamma)*psi3 - delta*dPsi3;
	
	dJointAddHingeTorque( joints[2], torque3 );

	cout << "Psi: (" << psi << ", " << psi2 << ") Torques: (" << torque << ", " << torque2 << ")" << endl;
	cout << "Ml: " << Ml << "      Mr: " << Mr << endl;

	////////////////////////////////////////////////////////////////////////////

	//Second, determine which geoms inside the space are potentially colliding.
	//The nearCallBack function will be responsible for analizing those potential collisions.
	//The second parameter indicates that no user data is being sent the callback routine.
	dSpaceCollide( Space, 0, &nearCallBack );

	//Third, advance the simulation, based on the step size given.
	dWorldStep( World, simulationStep );

	//Last, remove the temporary contact joints.
	dJointGroupEmpty( ContactGroup );

	//At this point, all geometries have been updated, so they can be drawn from display().
	simulationTime += simulationStep;
}

/*******************************************************************************
Function to draw the objects inside ODE world.
*******************************************************************************/
void GOde::drawObjects( bool draw )
{
	if( draw )				//Draw by default, unless user passes a false.
	{
		//Draw geometries.
		for( unsigned I = 0; I < objects.size(); I++ )
			objects[I].drawGeometries();

		//Draw joints.
		for( unsigned I = 0; I < joints.size(); I++ )
		{
			dVector3 anchor;	//Position in world coordinates that correspond to the anchor.
			if( dJointGetType( joints[I] ) == dJointTypeHinge )	//Is it a hinge?
				dJointGetHingeAnchor( joints[I], anchor );

			glPushMatrix();						//Store current ModelView matrix.
			GDrawing::setColor( 1.0, 1.0, 1.0 );
			glTranslated( anchor[0], anchor[1], anchor[2] );
			glScaled( 0.01, 0.01, 0.01 );
			GDrawing::drawSphere();				//Draw a sphere wher the anchor is.
			glPopMatrix();						//Restore previous pipeline state.
		}
	}
}

/*******************************************************************************
Function to get the pointer of a joint, given its index in the vector of joints.
*******************************************************************************/
dJointID GOde::getJoint( int id )
{
	assert( id >= 0 && id < joints.size() );	//Check availability.

	return joints[id];
}

/*******************************************************************************
Function to get a reference to an object, given its index in the array of objs.
*******************************************************************************/
GOdeObject* GOde::getObject( int id )
{
	assert( id >= 0 && id < objects.size() );	//Check availability.

	return &(objects[id]);
}
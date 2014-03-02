#include "GOde.h"
#include "GSalamander.h"


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
	//GOde ode;

	//Get the dynamics body for each potentially colliding geometry.
	dBodyID b1 = dGeomGetBody( o1 );
	dBodyID b2 = dGeomGetBody( o2 );
	unsigned short *ptr1 = (unsigned short*)dGeomGetData( o1 );		//Get data from first geometry.
	unsigned short *ptr2 = (unsigned short*)dGeomGetData( o2 );		//Get data from second geometry.

	//Create an array of dContact objects to hold the contact joints.
	dContact contacts[ MAX_CONTACTS ];

	//cout << "Here is the *ptr number: " << *ptr << endl;
	//Initialize contact structures.
	for( I = 0; I < MAX_CONTACTS; I++ )
	{
		contacts[I].surface.mode = dContactBounce | dContactSoftCFM;
<<<<<<< HEAD

		if(*ptr==0)
		{
			contacts[I].surface.mu = 0.0;		//0: frictionless, dInfinity: never slips.
			contacts[I].surface.mu2 = 0;			//Friction in direction 2 to mu.
		}
		
		else if(*ptr==12)
		{
			contacts[I].surface.mu = 0.04;		//0: frictionless, dInfinity: never slips.
			contacts[I].surface.mu2 = 0.04;			//Friction in direction 2 to mu.
		}
		else if(*ptr==11)
		{
			contacts[I].surface.mu = 0;		//0: frictionless, dInfinity: never slips.
			contacts[I].surface.mu2 = 0;			//Friction in direction 2 to mu.
		}
		else if(*ptr==22)
		{
			contacts[I].surface.mu = 0;		//0: frictionless, dInfinity: never slips.
			contacts[I].surface.mu2 = 0;			//Friction in direction 2 to mu.
		}
		else if(*ptr==21)
		{
			contacts[I].surface.mu = 0.04;		//0: frictionless, dInfinity: never slips.
			contacts[I].surface.mu2 = 0.04;			//Friction in direction 2 to mu.
		}
		else if(*ptr==32)
		{
			contacts[I].surface.mu = 0;		//0: frictionless, dInfinity: never slips.
			contacts[I].surface.mu2 = 0.;			//Friction in direction 2 to mu.
		}
		else if(*ptr==31)
		{
			contacts[I].surface.mu = 0.04;		//0: frictionless, dInfinity: never slips.
			contacts[I].surface.mu2 = 0.04;			//Friction in direction 2 to mu.
		}
		else if(*ptr==42)
		{
			contacts[I].surface.mu = 0.04;		//0: frictionless, dInfinity: never slips.
			contacts[I].surface.mu2 = 0.04;			//Friction in direction 2 to mu.
		}
		else if(*ptr==41)
		{
			contacts[I].surface.mu = 0;		//0: frictionless, dInfinity: never slips.
			contacts[I].surface.mu2 = 0;			//Friction in direction 2 to mu.
		}
		else
		{
			contacts[I].surface.mu = 0;		//0: frictionless, dInfinity: never slips.
			contacts[I].surface.mu2 = 0;			//Friction in direction 2 to mu.
=======
		contacts[I].surface.mu = 0.0;			//0 friction for general collisions.
		contacts[I].surface.mu2 = 0.0;			//Update this value according to special cases (look below).

		//Check all possible cases of collision among geometries.
		if( ptr1 != NULL && ptr2 != NULL )		//Are data attached to geometries that are colliding?
		{
			if( *ptr1 == 0x00FF || *ptr2 == 0x00FF )	//Collision with ground?
			{
				int salamanderPart;					//Indicates if it is a calf or body/upper leg segment.
				int friction;						//Get the friction 4-bit code from 0xABCD.
				unsigned short *ptr;				//Just to recognize which part of the salamander is colliding.

				if( *ptr1 != 0x00FF)				//Is the first body part of salamander?
					ptr = ptr1;
				else								//Or is it the second body?
					ptr = ptr2;

				salamanderPart = (int)( ((*ptr) & 0xF000) >> 12 );	//Shift bits: (0=body, 1-4=upper-leg, 5-8=calf)
				friction = (int)( ((*ptr) & 0x0F00) >> 8 );			//Shift bits to the right here too.

				if( friction > 0 )					//Should we apply friction to this part of the salamander?
				{
					if( salamanderPart >= 5 && salamanderPart <= 8 )//Is it a calf?
					{
						contacts[I].surface.mu = 1.05;				//0: frictionless, dInfinity: never slips.
						contacts[I].surface.mu2 = 1.05;				//Friction in direction 2 to mu.
					}
				}
			}
>>>>>>> Last Salamander Working
		}
		
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
	simulationStep = 0.01;
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
	jointGroups.push_back( dJointGroupCreate( 0 ) );	//Create a group of joints for any living object in ODE.

	dWorldSetGravity( World, 0.0, -9.81, 0.0 );	//Add gravity to this World.

	//Define error conrrection constants.
	dWorldSetERP( World, 0.2 );
	dWorldSetCFM( World, 1e-5 );

	//Set the velocity that interpenetrating objects will separate at.
	dWorldSetContactMaxCorrectingVel( World, 0.9 );

	//Set the safety area for contacts to be considered at rest.
	dWorldSetContactSurfaceLayer( World, 0.001 );

	//Automatically disable objects that have come to a rest.
	dWorldSetAutoDisableFlag( World, 0 );

	//Create a collision plane and add it to space. The next parameters are the
	//literal components of ax + by + cz = d.
	ground.body = NULL;			//Plane has not body (with mass).
	ground.userData = 0x00FF;	//The code for collisions for ground is 255.
	ground.geometries.push_back( dCreatePlane( Space, 0.0, 0.045, 0.0, 0.0 ) );
	dGeomSetData( ground.geometries[0], &ground.userData );

	//////////////////////// Initializing salamander 1 /////////////////////////
	
<<<<<<< HEAD
	dVector3 position = { -0.345, 0.1, 0.0 };
	GSalamander s1( position, 2.0, false );	//Position lander salamander, with frequency 1.0.
=======
	dVector3 position = { -0.345, 0.045, 0.0 };
	GSalamander s1( position, 2.0, true );	//Position lander salamander, with frequency 1.0.
>>>>>>> Last Salamander Working
	s1.createSalamander( World, Space, jointGroups[0] );
	salamanders.push_back( s1 );
	
	///////////////////////// Apply external, initial torque ///////////////////
	/* Here */
	
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
	vector< GSalamander >().swap( salamanders );
}

/*******************************************************************************
Function to perform the simulation loop for ODE.
*******************************************************************************/
void GOde::simulationLoop()
{
	//First, accumulate forces and torques in objects (bodies).
	
	for( unsigned I = 0; I < salamanders.size(); I++ )					//Compute internal torques in salamanders.
		salamanders[I].computeForces( simulationTime, simulationStep );

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
		//Draw geometries that belong to the world (not to any other entity).
		drawGeometries( &objects );

		//Draw geometries that belong to the salamanders.
		for( unsigned I = 0; I < salamanders.size(); I++ )
		{
			const vector< GOdeObject>* sLinks = salamanders[I].getLinks();
			drawGeometries( sLinks );
		}

		//Draw joints that belong to the world (no to any other entity).
		drawJoints( &joints );

		//Draw joints that belong to salamanders.
		vector< dJointID > sJoints;
		for( unsigned I = 0; I < salamanders.size(); I++ )	//Draw joints in salamanders.
		{
			salamanders[I].getJoints( &sJoints );
			drawJoints( &sJoints );
		}
	}
}

/*******************************************************************************
Function to draw the geometries associated with a collection of objects.
*******************************************************************************/
void GOde::drawGeometries( const vector< GOdeObject >* objectsPointer )
{
	for( unsigned I = 0; I < objectsPointer->size(); I++ )
		objectsPointer->at( I ).drawGeometries();
}

/*******************************************************************************
Function to draw joints given in a collection.
*******************************************************************************/
void GOde::drawJoints( const vector< dJointID >* jointsPointer )
{
	for( unsigned I = 0; I < jointsPointer->size(); I++ )
	{
		dVector3 anchor;	//Position in world coordinates for the given joint.
				
		if( dJointGetType( jointsPointer->at( I ) ) == dJointTypeHinge )	//Is it a hinge?
			dJointGetHingeAnchor( jointsPointer->at( I ), anchor );

		glPushMatrix();						//Store current ModelView matrix.
		GDrawing::setColor( 1.0, 1.0, 1.0 );
		glTranslated( anchor[0], anchor[1], anchor[2] );
		glScaled( 0.006, 0.006, 0.006 );
		GDrawing::drawSphere();				//Draw a sphere wher the anchor is.
		glPopMatrix();						//Restore previous pipeline state.
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

/*******************************************************************************
Function to get a pointer to a Salamander object, given its index.
*******************************************************************************/
GSalamander* GOde::getSalamander( int id )
{
	assert( id >= 0 && id < salamanders.size() );	//Check vector boundaries.
	return &salamanders[id];						//Return the pointer.
}

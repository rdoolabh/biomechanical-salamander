/*******************************************************************************
Composite objects in ODE.
*******************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include "ode/ode.h"

#include <gl/gl.h>
#include <gl/glu.h>
#include "GL/glut.h"

#include <math.h>
#include <iostream>

using namespace std;

#include "Ball.h"
#include "FrameSaver.h"
#include "Timer.h"
#include "GDrawing.h"

///////////////////////////  OpenGL global variables ///////////////////////////
FrameSaver g_frameSaver;
Timer g_timer;

BallData *g_arcBall = NULL;
int g_width = 700;
int g_height = 700;
int g_button = -1;
float g_zoom = 1;
int g_previousY = 0;

int g_animate = 0;
int g_recording = 0;

void resetArcball();
void save_image();

const int STRLEN = 100;
typedef char STR[STRLEN];

#define X 0
#define Y 1
#define Z 2

// The eye point and look-at point.
double g_eye[3] = {0.0, 0.0, 50.0};
double g_ref[3] = {0.0, 0.0, 0.0};
double g_time = 0.0 ;
////////////////////////////////////////////////////////////////////////////////

/////////////////////////// ODE Global Variables ///////////////////////////////
dReal simulationTime = 0.0;
dReal simulationStep = 0.01;
struct MyObject
{
	dBodyID Body;		//The dynamics body.
	dGeomID Geom[3];	//Geometries representing this body.
};

MyObject Object[1];			//The rigid bodies.
dWorldID World;				//Dynamics world.
dSpaceID Space;				//A space that defines collisions.
dJointGroupID ContactGroup;	//Group of contact joints for collision detection/handling.
dJointGroupID JointGroup;	//Group of joints for objects connected.
dJointID Joint;				//Joint for both bodies.
dReal localPoint[] = {1.5, 0.0, 0.0};		//A local point to the body to apply a force.
////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
Function to initialize ODE.
*******************************************************************************/
void initODE()
{
	///////////////// Initializing the ODE general features ////////////////////

	dInitODE();								//Initialize library.
	World = dWorldCreate();					//Crate a new dynamics, empty world.
	Space = dSimpleSpaceCreate(0);			//Create a new space for collision (independent).
	ContactGroup = dJointGroupCreate(0);	//Create a joints container, without specifying size.

	dWorldSetGravity( World, 0.0, -9.81, 0 );	//Add gravity to this World.

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
	dCreatePlane( Space, 0.0, 5.0, 0.0, 0.0 );

	/////////////// Initializing the rigid body 1 in the world /////////////////

	Object[0].Body = dBodyCreate( World );		//Create a new body and attach it to object.
	dBodySetPosition( Object[0].Body, 0.0, 5.5, 0.0 );		//Assign position to body.
	dBodySetLinearVel( Object[0].Body, 0.0, 0.0, 0.0 );		//Object begins stationary.

	dMass m, M;									//Start accumulating mass.
	dMassSetZero( &M );

	dReal positions[3][3];						//Positions for the three geometries.
	dMatrix3 rotations[3];						//Rotations for geoms.

	dReal radius = 0.5;						//All three geoms have the same measurements.
	dReal length = 6.0;
	int a=0;
	for( int I = 0; I < 3; I++ )				//Iterate over three geoms.
	{
		//Define positions for geom.
		for( int J = 0; J < 3; J++)				//Iterate over X, Y, Z.
			positions[I][J] = 0.0;
		
		//Define orientations for geom.
		dReal axis[] = {0.0, 0.0, 0.0};
		if(I==2) axis[1]=1;
		else axis[I] = 1.0;
		dRFromAxisAndAngle( rotations[I], 
			axis[0],				//Axis of rotation (to create a 3D asterisk).
			axis[1], 
			axis[2], 
			M_PI/2 );		//Angle.
		
		//Create the geometry.
		
		Object[0].Geom[I] = dCreateCapsule( Space, radius, length );
		
		//Create mass distribution for a capsule.
		dMassSetCapsule( &m, 0.1, 3, radius, length );
		if(I==0)length =3.0;
		//Rotate and translate the capsule-mass distribution in the body frame.
		dMassRotate( &m, rotations[I] );
		dMassTranslate( &m, positions[I][0], positions[I][1], positions[I][2] );

		//Add geometry mass distribution to body.
		dMassAdd( &M, &m );
	}

	for( int I = 0; I < 3; I++)				//Move geomes inside the body.
	{
		//Attach geometry to body.
		dGeomSetBody( Object[0].Geom[I], Object[0].Body );

		//Set position and orientation for the geoms.
		dGeomSetOffsetRotation( Object[0].Geom[I], rotations[I] );
		if(I==1)
		{
			dGeomSetOffsetPosition( Object[0].Geom[I], 
			positions[I][0] - M.c[0],
			positions[I][1] - M.c[1]+3.0,
			positions[I][2] - M.c[2] );	
		}
		else if(I==2)
		{
			dGeomSetOffsetPosition( Object[0].Geom[I], 
			positions[I][0] - M.c[0],
			positions[I][1] - M.c[1]-3.0,
			positions[I][2] - M.c[2] );	
		}
		else{
		dGeomSetOffsetPosition( Object[0].Geom[I], 
			positions[I][0] - M.c[0],
			positions[I][1] - M.c[1],
			positions[I][2] - M.c[2] );		//Subtract center of mass location because we want it to be at origin.
		}
	}

	//Move center of mass and apply it to body.
	dMassTranslate( &M, -M.c[0], -M.c[1], -M.c[2] );
	dBodySetMass( Object[0].Body, &M );

	dMatrix3 Orient;										//Assign orientation.
	dRFromAxisAndAngle( Orient, 1.0, 1.0, 0.0, 0.8 );
	dBodySetRotation( Object[0].Body, Orient );

	dBodySetData( Object[0].Body, (void *)0 );				//No data associated with object.
	
}

/*******************************************************************************
Function to clean the ODE system.
*******************************************************************************/
void closeODE()
{
	dJointGroupDestroy( ContactGroup );		//Remove the contact joints.
	dJointGroupDestroy( JointGroup );		//Destroy joint attaching both bodies.
	dSpaceDestroy( Space );					//Remove the space and all of its geoms.
	dWorldDestroy( World );					//Destroy all bodies and joints (not in a group).
}
char frictCoeff='f';
/*******************************************************************************
Function to handle potential collisions between geometries.
*******************************************************************************/
static void nearCallBack( void *data, dGeomID o1, dGeomID o2 )
{
	int I;					//A temporary index for each contact.
	const int MAX_CONTACTS = 5;
	dContact contacts[ MAX_CONTACTS ];

	//Get the dynamics body for each potentially colliding geometry.
	dBodyID b1 = dGeomGetBody( o1 );
	dBodyID b2 = dGeomGetBody( o2 );

	//Create an array of dContact objects to hold the contact joints.

	//Initialize contact structures.
	for( I = 0; I < MAX_CONTACTS; I++ )
	{
		contacts[I].surface.mode = dContactBounce | dContactSoftCFM;
		if(I==1){
			if(frictCoeff=='l') contacts[I].surface.mu = 0;
			else if (frictCoeff=='r') contacts[I].surface.mu = dInfinity;
			else contacts[I].surface.mu = 0;}
		else if(I==2){
			if(frictCoeff=='l') contacts[I].surface.mu = dInfinity;
			else if (frictCoeff=='r') contacts[I].surface.mu = 0;
			else contacts[I].surface.mu = 0;}
		else if(I==0)	contacts[I].surface.mu = 0;		//0: frictionless, dInfinity: never slips.

		contacts[I].surface.slip1=1.0;
		contacts[I].surface.slip2=1.0;
		contacts[I].surface.bounce = 0.01;		//0: not bouncy, 1: max. bouncyness.
		contacts[I].surface.bounce_vel = 0.5;	//Minimum incoming velocity for producting bouncyness.
		contacts[I].surface.soft_cfm = 0.01;	//Softness for maintaining joint constraints.
	}

	//Now, do the actual collision test, passing as parameters the address of
	//a dContactGeom structure, and the offset to the next one in the list.
	
	if( int numc = dCollide( o1, o2, MAX_CONTACTS, &contacts[0].geom, sizeof(dContact) ) )
	{
		//Add contacts detected to the contact joint group.
		for( I = 0; I < numc; I++ ) // REDUCING THIS to numc-1 removes the getting stuck problem, but raises other problems
		{
			//Add contact joint by knowing its world, to a contact group. The last parameter
			//is the contact itself.
			dJointID c = dJointCreateContact( World, ContactGroup, contacts + I );
			dJointAttach( c, b1, b2 );		//Attach two bodies to this contact joint.
		}
	}
	cout <<"--------Frictions-------"<<endl;
	cout <<"frictCoeff: "<<frictCoeff<<endl;
	cout <<"object 1: " <<contacts[1].surface.mu;
	cout <<"object 2: " <<contacts[2].surface.mu;
}

/*******************************************************************************
Function to perform the simulation loop for ODE.
*******************************************************************************/
void simulationLoop()
{
	//First, determine which geoms inside the space are potentially colliding.
	//The nearCallBack function will be responsible for analizing those potential collisions.
	//The second parameter indicates that no user data is being sent the callback routine.
	dSpaceCollide( Space, 0, &nearCallBack );

	//Next, advance the simulation, based on the step size given.
	dWorldStep( World, simulationStep );

	//Then, remove the temporary contact joints.
	dJointGroupEmpty( ContactGroup );

	//At this point, all geometries have been updated, so they can be drawn from display().
}

/*******************************************************************************
Function to transform an ODE orientation matrix into an OpenGL transformation
matrix (orientation plus position).
p -> a vector of three elements.
R -> the rotation matrix, in row vector format, with 12 elements.
M -> the resulting matrix, in vector format, with 16 elements.
*******************************************************************************/
void ODEToOpenGLMatrix( const dReal* p, const dReal* R, dReal* M)
{
	M[0]  = R[0]; M[1]  = R[4]; M[2]  = R[8];  M[3]  = 0;
    M[4]  = R[1]; M[5]  = R[5]; M[6]  = R[9];  M[7]  = 0;
    M[8]  = R[2]; M[9]  = R[6]; M[10] = R[10]; M[11] = 0;
    M[12] = p[0]; M[13] = p[1]; M[14] = p[2];  M[15] = 1;
}

/*******************************************************************************
Function to render a box, given it sides length, position and orientation.
*******************************************************************************/
void renderBox( const dReal sides[3], const dReal position[3], const dReal orientation[12] )
{
	glPushMatrix();					//Save current ModelView.
	
	dReal Matrix[16];				//The OpenGL version of the transformation matrix.
	ODEToOpenGLMatrix( position, orientation, Matrix );
	glMultMatrixd( Matrix );
	glScaled( sides[0], sides[1], sides[2] );	//Scale to have the right measure in sides.
	GDrawing::setColor( 0.5, 0.6, 0.7 );
	GDrawing::drawCube();

	glPopMatrix();					//Restore ModelView.
}

/*******************************************************************************
Function to render a sphere, given its radius, position and orientation.
*******************************************************************************/
void renderSphere( const dReal radius, const dReal position[3], const dReal orientation[12] )
{
	glPushMatrix();					//Save current ModelView.

	dReal Matrix[16];				//OpenGL version of the transormation matrix.
	ODEToOpenGLMatrix( position, orientation, Matrix );
	glMultMatrixd( Matrix );
	glScaled( radius, radius, radius );		//Scale to the sphere radius.
	GDrawing::setColor( 0.0, 1.0, 0.0 );
	GDrawing::drawSphere();

	glPopMatrix();
}

/*******************************************************************************
Function to render a capsule, given its radius, length, position and orientation.
*******************************************************************************/
void renderCapsule( const dReal radius, const dReal length, const dReal position[3], const dReal orientation[12] )
{
	glPushMatrix();					//Save current ModelView.

	dReal Matrix[16];				//OpenGL equivalen version of ODE orientation.
	ODEToOpenGLMatrix( position, orientation, Matrix );
	glMultMatrixd( Matrix );

	glPushMatrix();					//Draw cylinder.
	glScaled( radius, radius, length );
	glTranslated( 0.0, 0.0, -0.5 );
	GDrawing::drawCylinder();
	glPopMatrix();

	//glTranslated( 0.0, 0.0, length/2.0 );	//Draw first cap.
	//glPushMatrix();
	//glScaled( radius, radius, radius );
	//GDrawing::drawSphere();
	//glPopMatrix();

	//glTranslated( 0.0, 0.0, -length );		//Draw second cap.
	//glScaled( radius, radius, radius );
	//GDrawing::drawSphere();

	//glPopMatrix();
}

/*******************************************************************************
Function to draw a geometry object.
*******************************************************************************/
void drawGeom( dGeomID g, const dReal *position, const dReal *orientation )
{
	if( !g )		//If the geometry object is missing, end the function.
		return;

	if( !position )	//Position was not passed?
		position = dGeomGetPosition( g );		//Then, get the geometry position.

	if( !orientation )	//Orientation was not given?
		orientation = dGeomGetRotation( g );	//And get existing geometry orientation.

	int type = dGeomGetClass( g );				//Get the type of geometry.
	
	if( type == dBoxClass )						//Is it a box?
	{
		dReal sides[3];
		dGeomBoxGetLengths( g, sides );				//Get length of sides.
		renderBox( sides, position, orientation );	//Render the actual box in environment.
	}

	if( type == dSphereClass )					//Is it a sphere?
	{
		dReal radius;
		radius = dGeomSphereGetRadius( g );				//Get the radius.
		renderSphere( radius, position, orientation );	//Render sphere in environment.
	}

	if( type == dCapsuleClass )
	{
		dReal radius;
		dReal length;
		dGeomCapsuleGetParams( g, &radius, &length );	//Get both radius and length.
		renderCapsule( radius, length, position, orientation );	//Render capsule in environment.
	}

	if( type == dGeomTransformClass )					//Is it an embeded geom in a composite body.
	{
		dGeomID g2 = dGeomTransformGetGeom( g );		//Get the actual geometry inside the wrapper.
		const dReal *position2 = dGeomGetPosition( g2 );	//Get position and orientation of wrapped geometry.
		const dReal *orientation2 = dGeomGetRotation( g2 );
		
		dVector3 actualPosition;						//Real world coordinated position and orientation
		dMatrix3 actualOrientation;						//of the wrapped geometry.
		
		dMultiply0_331( actualPosition, orientation, position2 );	//Get world coordinates of geometry position.
		actualPosition[0] += position[0];
		actualPosition[1] += position[1];
		actualPosition[2] += position[2];

		dMultiply0_333( actualOrientation, orientation, orientation2 );	//Get world coordinates of geom orientation.

		drawGeom( g2, actualPosition, actualOrientation );	//Draw embeded geometry.
	}
}

/*******************************************************************************
Function to reset the zoom-in zoom-out effect.
*******************************************************************************/
void resetArcball()
{
	Ball_Init(g_arcBall);
	Ball_Place(g_arcBall, qOne, 0.75);
}
	
/*******************************************************************************
Function that gets called for any keypresses.
*******************************************************************************/
void myKey(unsigned char key, int x, int y)
{
	float time;
	dVector3 result;
	switch (key) 
	{
		case 'q':
		case 27:
			exit(0); 
		case 's':
			g_frameSaver.DumpPPM(g_width,g_height);
			break;
		case 'r':
			resetArcball();
			break;
		case 'a':
			g_animate = 1 - g_animate;
			//Reset the timer to point to the current time.		
			time = g_timer.GetElapsedTime();
			g_timer.Reset();
			break;
		case '0':
			//Reset your object.
			break ;
		case 'm':
			if( g_recording == 1 )
			{
				cout << "Frame recording disabled." << endl;
				g_recording = 0;
			}
			else
			{
				cout << "Frame recording enabled." << endl;
				g_recording = 1 ;
			}
			g_frameSaver.Toggle();
			break ;
		case 'h':
		case '?':
			GDrawing::plotInstructions();
			break;
		case '1':
			//Body coordinates of a point in world coordinates.
			dBodyGetPosRelPoint( Object[0].Body, 1.0, 0.0, 0.0, result );
			cout<<"Body coordinates of Pw [1,0,0] are Pb = ["<<result[0]<<","<<result[1]<<","<<result[2]<<"]"<<endl;
			break;
		case '2':
			//World coordinates of a point in body coordinates.
			dBodyGetRelPointPos( Object[0].Body, 1.0, 0.0, 0.0, result );
			cout<<"World coordinates of Pb [1,0,0] are Pw = ["<<result[0]<<","<<result[1]<<","<<result[2]<<"]"<<endl;
			break;
		case '3':
			//Apply force to a point given in body coordinates.
			frictCoeff='l';
			dBodyAddForceAtRelPos( Object[0].Body, 0.0, 0.0, 100.0, localPoint[0], localPoint[1], localPoint[2] );
			break;
			//dJointAddHingeTorque( ode.getJoint(0), -10.0 );
			//break;
		case '4':
			//Apply force to a point given in body coordinates.
			frictCoeff='r';
			dBodyAddForceAtRelPos( Object[0].Body, 0.0, 0.0, 100.0, localPoint[0], localPoint[1], localPoint[2] );
			break;
		case '5':
			//Apply force to a point given in body coordinates.
			frictCoeff='f';
			dBodyAddForceAtRelPos( Object[0].Body, 0.0, 0.0, 100.0, localPoint[0], localPoint[1], localPoint[2] );
			break;
	}
	glutPostRedisplay();
}

/*******************************************************************************
Function that performs most of the OpenGL intialization.
*******************************************************************************/
void myinit(void)
{
    GLfloat ambient[] = { 0.2, 0.2, 0.2, 1.0 };
    GLfloat diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat specular[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat position[] = { -3.0, 3.0, 3.0, 0.0 };
	//GLfloat position[] = { 0.0, 0.0, 30.0, 1.0 };
	GLfloat diffuse2[] = { 0.3, 0.3, 0.3, 1.0 };
    GLfloat specular2[] = { 0.3, 0.3, 0.3, 1.0 };
	GLfloat position2[] = { 0.0, 100.0, 00.0, 1.0 };
    
    GLfloat lmodel_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
    GLfloat local_view[] = { 0.0 };

    /**** Set lighting parameters ****/
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
	glLightf(GL_LIGHT0, GL_SHININESS, 100) ;
    glLightfv(GL_LIGHT0, GL_POSITION, position);
	
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
    glLightModelfv(GL_LIGHT_MODEL_LOCAL_VIEWER, local_view);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE) ;

	glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse2);
	glLightfv(GL_LIGHT1, GL_SPECULAR, specular2);
    glLightfv(GL_LIGHT1, GL_POSITION, position2);
	glLightf(GL_LIGHT1, GL_SHININESS, 500) ;
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
    glEnable(GL_AUTO_NORMAL);
    glEnable(GL_NORMALIZE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

	glPixelStorei(GL_PACK_ALIGNMENT,1);
	glPixelStorei(GL_UNPACK_ALIGNMENT,1);
	glShadeModel(GL_SMOOTH);

	g_arcBall = new BallData;
	Ball_Init(g_arcBall);
	Ball_Place(g_arcBall,qOne,0.75);
}

/*******************************************************************************
Function that gets called by the event handler to draw the scene.
*******************************************************************************/
void display(void)
{
	//glClearColor (red, green, blue, alpha)
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);		//Set the background color.
	
	//OK, now clear the screen with the background color.
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//Load initial matrix transformation.
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//Locate the camera.
	gluLookAt (g_eye[X], g_eye[Y], g_eye[Z], g_ref[X], g_ref[Y], g_ref[Z], 0.0, 1.0, 0.0);

	HMatrix arcball_rot;
	Ball_Value(g_arcBall,arcball_rot);
	glMultMatrixf((float *)arcball_rot);

	//Scale the scene in response to mouse commands.
	glScalef(g_zoom, g_zoom, g_zoom); 

	////////////////////// Draw the geometries in World ////////////////////////
	GDrawing::setColor( 0.3, 0.3, 0.8 );
	drawGeom( Object[0].Geom[0], NULL, NULL );
	drawGeom( Object[0].Geom[1], NULL, NULL );
	drawGeom( Object[0].Geom[2], NULL, NULL );

	dVector3 result;					//Draw point where a force is applied.
	GDrawing::setColor( 1.0, 1.0, 1.0 );
	dBodyGetRelPointPos( Object[0].Body, localPoint[0], localPoint[1], localPoint[2], result );
	glPushMatrix();
	glTranslated( result[0], result[1], result[2] );
	glScaled( 0.3, 0.3, 0.3 );
	GDrawing::drawSphere();
	glPopMatrix();


	glPushMatrix();						//Draw the collision plane.
	glTranslated( 0.0, -0.05, 0.0 );
	glScaled( 20.0, 0.1, 20.0 );
	GDrawing::setColor( 1.0, 0.2, 0.3 );
	GDrawing::drawCube();
	glPopMatrix();

	////////////////////////////////////////////////////////////////////////////

	glutSwapBuffers();
	if( g_recording == 1)
		g_frameSaver.DumpPPM(g_width,g_height);
}

/*******************************************************************************
Function that handles the window being resized
*******************************************************************************/
void myReshape(int w, int h)
{
	g_width = w;
	g_height = h;

	float asp = (float) w / (float) h ;

	cout << "New aspect ratio " << asp << endl;

	glViewport(0, 0, w,h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
    
	// This defines the field of view of the camera.
    // Making the first 4 parameters larger will give a larger field of 
	// view, therefore making the objects in the scene appear smaller.
	glOrtho(-asp*15, asp*15, -15, 15, -500,500);

	// Use either of the following functions to set up a perspective view
	//gluPerspective(20,(float) w/(float) h,1,100) ;
	//glFrustum(-1,1,-1,1,4,100);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

    // This sets the virtual camera:
    // gluLookAt( x,y,z,   x,y,z   x,y,z );
    //            camera  look-at camera-up
    //            pos'n    point   vector
    gluLookAt(g_eye[X], g_eye[Y], g_eye[Z],	g_ref[X], g_ref[Y], g_ref[Z], 0, 1, 0);

	HMatrix arcball_rot;
	Ball_Value(g_arcBall,arcball_rot);
	glMultMatrixf((float *)arcball_rot);
}

/*******************************************************************************
Event handler for mouse buttons.
*******************************************************************************/
void myMouseCB(int button, int state, int x, int y)
{
	g_button = button ;
	if( g_button == GLUT_LEFT_BUTTON && state == GLUT_DOWN )
	{
		HVect arcball_coords;
		arcball_coords.x = 2.0*(float)x/(float)g_width-1.0;
		arcball_coords.y = -2.0*(float)y/(float)g_height+1.0;
		Ball_Mouse(g_arcBall, arcball_coords) ;
		Ball_Update(g_arcBall);
		Ball_BeginDrag(g_arcBall);

	}
	if( g_button == GLUT_LEFT_BUTTON && state == GLUT_UP )
	{
		Ball_EndDrag(g_arcBall);
		g_button = -1 ;
	}
	if( g_button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN )
	{
		g_previousY = y ;
	}

	// Tell the system to redraw the window.
	glutPostRedisplay();
}

/*******************************************************************************
Event handler for mouse motion.
*******************************************************************************/
void myMotionCB(int x, int y)
{
	if( g_button == GLUT_LEFT_BUTTON )
	{
		HVect arcball_coords;
		arcball_coords.x = 2.0*(float)x/(float)g_width - 1.0 ;
		arcball_coords.y = -2.0*(float)y/(float)g_height + 1.0;
		Ball_Mouse(g_arcBall,arcball_coords);
		Ball_Update(g_arcBall);
		glutPostRedisplay();
	}
	else if( g_button == GLUT_RIGHT_BUTTON )
	{
		if( y - g_previousY > 0 )
			g_zoom  = g_zoom * 1.03;
		else 
			g_zoom  = g_zoom * 0.97;
		g_previousY = y;
		glutPostRedisplay();
	}
}

/*******************************************************************************
Function that hanles the idle time of the OpenGL main loop.
*******************************************************************************/
void idleCB(void)
{
	if( g_animate == 1 )
	{
		//if( g_recording == 0 )
		//	g_time = g_timer.GetElapsedTime() ;
		//else
		//	g_time += 0.033 ; // save at 30 frames per second.
		
		simulationLoop();		//Execute simulation.
		simulationTime += simulationStep;
		cout << "Current time: " << simulationTime << endl;

		glutPostRedisplay() ; 
	}
}

/*******************************************************************************
Main function: calls initialization, then hands over control to the event
handler, which calls display() whenever the screen needs to be redrawn.
*******************************************************************************/
int main(int argc, char** argv) 
{
	initODE();						//Initialize the dynamics world.
	
	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowPosition (0, 0);
	glutInitWindowSize(g_width,g_height);
	glutCreateWindow(argv[0]);

	myinit();

	glutIdleFunc(idleCB) ;
	glutReshapeFunc (myReshape);
	glutKeyboardFunc( myKey );
	glutMouseFunc(myMouseCB);
	glutMotionFunc(myMotionCB);
	GDrawing::plotInstructions();

	glutDisplayFunc(display);
	glutMainLoop();

	g_timer.Reset();
	return 0;         //Never reached
}
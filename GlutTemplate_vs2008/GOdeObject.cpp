#include "GOdeObject.h"

/*******************************************************************************
Constructor.
*******************************************************************************/
GOdeObject::GOdeObject(void)
{
	R = G = B = 0.9;			//Set a default color for this object.
	userData = 0;				//No user data associated with the object.
}

/*******************************************************************************
Destructor.
*******************************************************************************/
GOdeObject::~GOdeObject(void)
{
}

/*******************************************************************************
Function to transform an ODE orientation matrix into an OpenGL transformation
matrix (orientation plus position).
p -> a vector of three elements.
R -> the rotation matrix, in row vector format, with 12 elements.
M -> the resulting matrix, in vector format, with 16 elements.
*******************************************************************************/
void GOdeObject::ODEToOpenGLMatrix( const dReal* p, const dReal* R, dReal* M)
{
	M[0]  = R[0]; M[1]  = R[4]; M[2]  = R[8];  M[3]  = 0;
    M[4]  = R[1]; M[5]  = R[5]; M[6]  = R[9];  M[7]  = 0;
    M[8]  = R[2]; M[9]  = R[6]; M[10] = R[10]; M[11] = 0;
    M[12] = p[0]; M[13] = p[1]; M[14] = p[2];  M[15] = 1;
}

/*******************************************************************************
Function to render a box, given it sides length, position and orientation.
*******************************************************************************/
void GOdeObject::renderBox( const dReal sides[3], const dReal position[3], const dReal orientation[12] ) const
{
	glPushMatrix();					//Save current ModelView.
	
	dReal Matrix[16];				//The OpenGL version of the transformation matrix.
	ODEToOpenGLMatrix( position, orientation, Matrix );
	glMultMatrixd( Matrix );
	glScaled( sides[0], sides[1], sides[2] );	//Scale to have the right measure in sides.
	GDrawing::drawCube();

	glPopMatrix();					//Restore ModelView.
}

/*******************************************************************************
Function to render a sphere, given its radius, position and orientation.
*******************************************************************************/
void GOdeObject::renderSphere( const dReal radius, const dReal position[3], const dReal orientation[12] ) const
{
	glPushMatrix();					//Save current ModelView.

	dReal Matrix[16];				//OpenGL version of the transormation matrix.
	ODEToOpenGLMatrix( position, orientation, Matrix );
	glMultMatrixd( Matrix );
	glScaled( radius, radius, radius );		//Scale to the sphere radius.
	GDrawing::drawSphere();

	glPopMatrix();
}

/*******************************************************************************
Function to render a capsule, given its radius, length, position and orientation.
*******************************************************************************/
void GOdeObject::renderCapsule( const dReal radius, const dReal length, const dReal position[3], const dReal orientation[12] ) const
{
	glPushMatrix();					//Save current ModelView.

	double Matrix[16];				//OpenGL equivalen version of ODE orientation.
	ODEToOpenGLMatrix( position, orientation, Matrix );
	glMultMatrixd( Matrix );

	glPushMatrix();					//Draw cylinder.
	glScaled( radius, radius, length );
	glTranslated( 0.0, 0.0, -0.5 );
	GDrawing::drawCylinder();
	glPopMatrix();

	glTranslated( 0.0, 0.0, length/2.0 );	//Draw first cap.
	glPushMatrix();
	glScaled( radius, radius, radius );
	GDrawing::drawSphere();
	glPopMatrix();

	glTranslated( 0.0, 0.0, -length );		//Draw second cap.
	glScaled( radius, radius, radius );
	GDrawing::drawSphere();

	glPopMatrix();
}

/*******************************************************************************
Function to draw a geometry object.
*******************************************************************************/
void GOdeObject::drawGeom( dGeomID g, const dReal *position, const dReal *orientation ) const
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
Function to draw all geometries associated to object's body.
*******************************************************************************/
void GOdeObject::drawGeometries() const
{
	GDrawing::setColor( R, G, B );				//Draw geometries using object's color.
	for( int I = 0; I < geometries.size(); I++ )
	{
		drawGeom( geometries[I], NULL, NULL );	//Iterate over all geometries and draw them.
	}
}

/*******************************************************************************
Function to change the color of this object.
*******************************************************************************/
void GOdeObject::setObjectColor( float r, float g, float b )
{
	R = r;
	G = g;
	B = b;
}
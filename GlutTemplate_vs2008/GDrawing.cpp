#include "GDrawing.h"

const double GDrawing::pi = 3.14159265359;

/**************************** Constructor *****************************/
GDrawing::GDrawing(void)
{
}

/****************************** Destructor ****************************/
GDrawing::~GDrawing(void)
{
}

/***********************************************************************
Function to render a solid cylinder oriented along the Z axis. 
Both bases are of radius 1. 
The bases of the cylinder are placed at Z = 0, and at Z = 1.
***********************************************************************/
void GDrawing::drawCylinder()
{
	static GLUquadricObj *cyl = NULL ;
	if( cyl == NULL )
	{
		cyl = gluNewQuadric() ;
	}
	if( cyl == NULL )
	{

		cerr << "Cannot allocate cylinder." << endl;

		return ;
	}
	gluQuadricDrawStyle(cyl,GLU_FILL) ;
	gluQuadricNormals(cyl,GLU_SMOOTH) ;
	gluCylinder(cyl,1.0,1.0,1.0,10,10) ;
}

/***********************************************************************
Function to render a solid cone oriented along the Z axis with base 
radius 1. 
The base of the cone is placed at Z = 0, and the appex at Z = 1. 
***********************************************************************/
void GDrawing::drawCone()
{
	glutSolidCone(1,1,20,20);
}

/***********************************************************************
Function that draws a tesselated square.
***********************************************************************/
void GDrawing::drawSquareTex()
{
	float i,j ;

	int NSUB = 50 ;
	float d = 1.0 / NSUB ;
	
	for( i = -0.5 ; i < 0.5 ; i += d )
		for( j = -0.5 ; j < 0.5 ; j+= d )
		{
			glBegin(GL_POLYGON) ;
			glNormal3f(0,0,1) ;
			float r = 1.0 ;
			
			//r = 0.5 + i+j ;
			float radius = 0.5 ;
			r = 0.5 + cos((i*i+j*j)*20*3.14) ;
			setColor(r,0.5,0.1) ;
			glVertex3f(i,j,0) ;
			glVertex3f(i+d,j,0) ;
			glVertex3f(i+d,j+d,0) ;
			glVertex3f(i,j+d,0) ;
			glEnd() ;
		}
}

/***********************************************************************
Function that draws a cube with dimensions 1,1,1 centered at the origin.
***********************************************************************/
void GDrawing::drawCube()
{
	glutSolidCube(1.0);
}

/***********************************************************************
Function that draws a sphere with radius 1 centered around the origin.
***********************************************************************/
void GDrawing::drawSphere()
{
	glutSolidSphere(1.0, 50, 50);
}

/***********************************************************************
Function that sets all material properties to the given RGB color.
***********************************************************************/
void GDrawing::setColor(float r, float g, float b)
{
	float ambient = 0.2f;
	float diffuse = 0.7f;
	float specular = 0.7f;
	GLfloat mat[4];
      
	/**** set ambient lighting parameters ****/
    mat[0] = ambient*r;
    mat[1] = ambient*g;
    mat[2] = ambient*b;
    mat[3] = 1.0;
    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, mat);

    /**** set diffuse lighting parameters ******/
    mat[0] = diffuse*r;
    mat[1] = diffuse*g;
    mat[2] = diffuse*b;
    mat[3] = 1.0;
    glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, mat);

    /**** set specular lighting parameters *****/
    mat[0] = specular*r;
    mat[1] = specular*g;
    mat[2] = specular*b;
    mat[3] = 1.0;
    glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, mat);
    glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 1.0);
}

/***********************************************************************
Function to display instructions in the command window.
***********************************************************************/
void GDrawing::plotInstructions()
{
	cout << "Keyboard:" << endl;
	cout << "  s to save the image." << endl;
	cout << "  r to restore the original view." << endl;
	cout << "  0 to set it to the zero state." << endl;
	cout << "  a to toggle the animation." << endl;
	cout << "  m to toggle frame dumping." << endl;
	cout << "  q to quit." << endl;
	cout << endl;

	cout << "Mouse:" << endl;
	cout << "right-click drag to hover view." << endl;
	cout << "lLeft-click drag to zoom view" << endl;
	cout << endl;
}
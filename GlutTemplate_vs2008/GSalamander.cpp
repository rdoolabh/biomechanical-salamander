/*******************************************************************************
Implementation of the Salamander class functions.
*******************************************************************************/

#include "GSalamander.h"
#include <math.h>
/*******************************************************************************
Default constructor: set a new salamander at the origin of the world frame.
*******************************************************************************/
GSalamander::GSalamander(void)
{
	dVector3 position = {0.0, 0.0, 0.0, 0.0};	//Set default position.
	GSalamander( position, 1.0, true );			//Initialize a walking salamander with frequency=1.
}

/*******************************************************************************
Customized constructor to locate the salamander at a given position.
*******************************************************************************/
GSalamander::GSalamander( dVector3 position, dReal f, bool l )
{
	sPosition[0] = position[0];
	sPosition[1] = position[1];
	sPosition[2] = position[2];

	freq = f;									//Assign frequency for neurosignals.
	lander = l;									//Determine if it is a lander or swimmer.

	amplitude = 0.5;							//This amplitude will be used for muscles 1 and 2.
	amplitudeLeft = amplitudeRight = amplitude;	//Make turning amplitudes for body joints equal.
}

/*******************************************************************************
Destructor.
*******************************************************************************/
GSalamander::~GSalamander(void)
{
}

/*******************************************************************************
Function to create the salamander, from head to tail, where head is at the
sPosition location in real world coordinates.
/******************************************************************************/
void GSalamander::createSalamander( dWorldID world, dSpaceID space, dJointGroupID jointGroup )
{
	/*//////////////////////////////////////////////////////////////////////////
	Salamander measurements for body links.
	LINK		LENGTH (m)		WIDTH (m)		MASS (Kg)
	Head		0.03			0.03			0.0177
	Neck		0.03			0.025			0.0147
	Trunk 1		0.03			0.03			0.0177
	Trunk 2		0.03			0.03			0.0177
	Trunk 3		0.03			0.03			0.0177
	Hips		0.03			0.022			0.013
	Tail 1		0.03			0.02			0.0118
	Tail 2		0.03			0.016			0.0094
	Tail 3		0.03			0.01			0.0059
	Tail 4		0.03			0.005			0.0029
	Leg- FL		0.25			0.01			0.002
	Leg- FR		0.25			0.01			0.002
	Leg- BL		0.15			0.01			0.0012
	Leg- BR		0.15			0.01			0.0012
	//////////////////////////////////////////////////////////////////////////*/
	dReal length[] = { 0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03 };
	dReal width[] = { 0.03, 0.025, 0.03, 0.03, 0.03, 0.022, 0.02, 0.016, 0.01, 0.005 };	 //Encoding of biometrics.
	dReal height[] = { 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04 };
	dReal mass[] = { 0.0177, 0.0147, 0.0177, 0.0177, 0.0177, 0.013, 0.0118, 0.0094, 0.0059, 0.0029 };
	dReal lLength[]= {0.025, 0.025, 0.025,0.025};
	dReal lWidth[]= {0.02, 0.02, 0.02, 0.02};
	dReal lMass[]= {0.002, 0.002, 0.002, 0.002};

	//Create links for head, neck, trunk, and tail; all with length 0.03.
	//Salamander's body grows along the an axis parallel to x.
	int nSegments = 10;
	dReal prevX = sPosition[0];						//To know where the previous link ended (considering joint space too).
	dReal jointSpace = 0.03;						//Empty space to accomodate internal joint.

	dReal tempx1=0.0;								//temp variables for positions of links 1 and 4
	dReal tempy1=0.0;
	dReal tempz1=0.0;
	dReal tempx5=0.0;
	dReal tempy5=0.0;
	dReal tempz5=0.0;

	for( int I = 0; I < nSegments; I++ )			//10 links exactly.
	{
		GOdeObject link;										//Create a new body link.
		link.body = dBodyCreate( world );						//Attach new body to world.
		link.userData = 1;										//1 means salamander's head/neck/trunk/tail link.
		
		dReal x = prevX + length[I]/2.0;
		dReal y = sPosition[1];
		dReal z = sPosition[2];
		dBodySetPosition( link.body, x, y, z );					//Position link.
		if(I==1){tempx1=x;tempy1=y;tempz1=z;}
		if(I==5){tempx5=x;tempy5=y;tempz5=z;}
		
		dBodySetLinearVel( link.body, 0.0, 0.0, 0.0 );			//Initial linear velocity.
		
		dMass M;
		dMassSetBoxTotal( &M, mass[I], length[I], height[I], width[I]);
		dBodySetMass( link.body, &M );							//Mass distribution.
		
		dGeomID g;
		g = dCreateBox( space, length[I], height[I], width[I] );//Geometry.
		dGeomSetData( g, &(link.userData) );					//Attach a user data pointer to geometry.
		link.geometries.push_back( g );
		dGeomSetBody( link.geometries[0], link.body );			//Link body to geometry.
		
		if( I < 2 )			//Head or neck?
			link.setObjectColor( 0.0, 0.8, 0.5 );				//Change color.
		else
		{
			if( I < 5 )		//Trunk?
				link.setObjectColor( 0.5, 0.8, 0.0 );
			else			//Tail.
				link.setObjectColor( 0.7, 0.0, 0.7 );
		}

		links.push_back( link );								//Attach new link to the list of links.
		prevX += length[I] + jointSpace;						//Making space for joint in between links.
	}
	
	for( int i=1; i<5;i++){
		GOdeObject link;										//Create a new body link.
		link.body = dBodyCreate( world );						//Attach new body to world.
		link.userData = 1;										//1 means salamander's head/neck/trunk/tail link.
		if(i<3)
			dBodySetPosition( link.body, tempx1, tempy1, tempz1+(pow(-1.0,i))*lLength[0]*2.0 );	
		else
			dBodySetPosition( link.body, tempx5, tempy5, tempz5+(pow(-1.0,i))*lLength[0]*2.0 ); //Position link.
		
		dBodySetLinearVel( link.body, 0.0, 0.0, 0.0 );			//Initial linear velocity.
		
		dMass M;
		dMassSetBoxTotal( &M, lMass[i-1], lLength[i-1], height[i-1], lWidth[i-1]);
		dBodySetMass( link.body, &M );							//Mass distribution.
		
		dGeomID g;
		g = dCreateBox( space, lLength[i-1], height[i-1], lWidth[i-1] );//Geometry.
		dGeomSetData( g, &(link.userData) );					//Attach a user data pointer to geometry.
		link.geometries.push_back( g );
		dGeomSetBody( link.geometries[0], link.body );			//Link body to geometry.

		links.push_back( link );	
	}

	//Create hinge joints for the 10 links in the body.
	dReal hingeAxis[] = { 0.0, 1.0, 0.0 };						//Hinge axis is parallell to y axis.
	dReal phaseStep = 2.0*GDrawing::pi/9.0;						//In case it is swimmer. 

	for( int I = 0; I < nSegments-1; I++ )
	{
		dJointID joint;
		joint = dJointCreateHinge( world, jointGroup );			//Create a hinge.
		dJointAttach( joint, links[I].body, links[I+1].body );	//Attach two consecutive bodies.
		
		dReal anchor[3];										//Anchor position for this joint.
		const dReal *pos1, *pos2;								//Positions for object 1 and 2.
		pos1 = dBodyGetPosition( links[I].body );
		pos2 = dBodyGetPosition( links[I+1].body );
		anchor[0] = ( pos1[0] + pos2[0] ) / 2.0;				//Get the mid point.
		anchor[1] = ( pos1[1] + pos2[1] ) / 2.0;
		anchor[2] = ( pos1[2] + pos2[2] ) / 2.0;
		dJointSetHingeAnchor( joint, anchor[0], anchor[1], anchor[2] );

		dJointSetHingeAxis( joint, hingeAxis[0], hingeAxis[1], hingeAxis[2] );	//Set axis for hing joint.
		dJointSetHingeParam( joint, dParamLoStop, -GDrawing::pi/4.0 );			//Sets limits for hinge angle.
		dJointSetHingeParam( joint, dParamHiStop, GDrawing::pi/4.0 );

		GSJoint gsJoint;										//Create object to hold information on this joint.
		gsJoint.angleDeformation = 0;							//Initialize angle deformation.
		gsJoint.joint = joint;									//Attach joint just created.

		if( lander )
		{
			if( I == 0 || I >= 5 )		//Head and tail are in phase = pi.
				gsJoint.lambda = GDrawing::pi;
			else
				gsJoint.lambda = 0.0;	//Joints in trunk are in phase.
		}
		else
			gsJoint.lambda = I * phaseStep;						//If it is a swimmer, joints have different phase.

		gsJoint.type = 'b';										//'b' is for body joint.

		gsJoints.push_back( gsJoint );							//Attach joint information to the general list.
	}
	
	for( int I = nSegments; I < nSegments+4; I++ ) // legs -1
	{
		dJointID joint;
		joint = dJointCreateHinge( world, jointGroup );			//Create a hinge.
		if(I<nSegments+2)	dJointAttach( joint, links[1].body, links[I].body );	//Attach two consecutive bodies.
		else dJointAttach( joint, links[5].body, links[I].body );
		dReal anchor[3];										//Anchor position for this joint.
		const dReal *pos1, *pos2;								//Positions for object 1 and 2.
		pos1 = dBodyGetPosition( links[I].body );
		if(I<nSegments+2)	pos2 = dBodyGetPosition( links[1].body );
		else pos2  =dBodyGetPosition( links[5].body );
		anchor[0] = ( pos1[0] + pos2[0] ) / 2.0;				//Get the mid point.
		anchor[1] = ( pos1[1] + pos2[1] ) / 2.0;
		anchor[2] = ( pos1[2] + pos2[2] ) / 2.0;
		dJointSetHingeAnchor( joint, anchor[0], anchor[1], anchor[2] );

		dJointSetHingeAxis( joint, hingeAxis[0], hingeAxis[1], hingeAxis[2] );	//Set axis for hing joint.
		dJointSetHingeParam( joint, dParamLoStop, 0 );			//Sets limits for hinge angle.
		dJointSetHingeParam( joint, dParamHiStop, 0 );

		GSJoint gsJoint;										//Create object to hold information on this joint.
		gsJoint.angleDeformation = 0;							//Initialize angle deformation.
		gsJoint.joint = joint;									//Attach joint just created.
		
		if( lander )
		{
			if( I == 0 || I >= 5 )		//Head and tail are in phase = pi.
				gsJoint.lambda = GDrawing::pi;
			else
				gsJoint.lambda = 0.0;	//Joints in trunk are in phase.
		}
		else
			gsJoint.lambda = I * phaseStep;						//If it is a swimmer, joints have different phase.

		gsJoint.type = 'l';										//'b' is for body joint.

		gsJoints.push_back( gsJoint );							//Attach joint information to the general list.
	}
}

/*******************************************************************************
Function to compute internal torques according to neurosignals.
*******************************************************************************/
void GSalamander::computeForces( dReal simulationTime, dReal simulationStep )
{
	//////////////////// Accumulating torques on every joint ///////////////////

	dReal Ml, Mr;									//Neurosignals.
	dReal desiredVelocity;							//Resulting desired velocity.
	dReal maxVelocity = 0.5;						
	for( unsigned I = 0; I < gsJoints.size(); I ++ )
	{
		if( gsJoints[I].type == 'b' )				//Is it a body joint?
		{
			Ml = max( amplitudeLeft*sin( freq*simulationTime - gsJoints[I].lambda ), 0.0 );	//Neurosignals.
			Mr = max( amplitudeRight*sin( freq*simulationTime - gsJoints[I].lambda + GDrawing::pi ), 0.0 );
			
			if( freq*simulationTime < GDrawing::pi )
			{
				Ml *= 0.5;
				Mr *= 0.5;
			}
			
			desiredVelocity = maxVelocity * ( Ml - Mr );				//Compute desired velocity.
			dJointSetHingeParam( gsJoints[I].joint, dParamVel, desiredVelocity );
			dJointSetHingeParam( gsJoints[I].joint, dParamFMax, 0.005 );

			//assert( abs( torque ) < 0.0001 );
			
			/*cout << "Joint: (" << I << ") Psi: (" << dJointGetHingeAngle(gsJoints[I].joint) << ") Velocity: (" << desiredVelocity << ")" << endl;
			cout << "Ml: " << Ml << "      Mr: " << Mr << endl;*/
			

		}
		 
	}
	
	//Reset turning amplitudes to their typical value.
	amplitudeLeft = amplitudeRight = amplitude;
}

/*******************************************************************************
Function to get a pointer to a const vector of segments.
*******************************************************************************/
const vector< GOdeObject >* GSalamander::getLinks()
{
	return &links;
}

/*******************************************************************************
Function to dump/copy only dJointID joints into an input vector.
*******************************************************************************/
void GSalamander::getJoints( vector< dJointID >* joints )
{
	joints->clear();		//Remove any previous content of recipient.

	for( unsigned I = 0; I < gsJoints.size(); I++ )
		joints->push_back( gsJoints[I].joint );
}

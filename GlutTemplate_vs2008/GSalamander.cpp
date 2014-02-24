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

	bodyMaxAngleDeformation = GDrawing::pi/4.0;	//Tell ODE that body hinges should not deform beyond pi/4.
	legMaxAngleDeformation = 0.0;				//Tell ODE that legs' joints should not go beyond this.
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
	dReal lLength[] = { 0.025, 0.025, 0.025,0.025};
	dReal lWidth[] = {0.01, 0.01, 0.01, 0.01 };
	dReal lMass[] = { 0.002, 0.002, 0.002, 0.002 };

	//Create links for head, neck, trunk, and tail; all with length 0.03.
	//Salamander's body grows along the an axis parallel to x.
	int nSegments = 10;
	dReal prevX = sPosition[0];						//To know where the previous link ended (considering joint space too).
	dReal jointSpace = 0.03;						//Empty space to accomodate internal joint.

	dReal tempx1=0.0;								//temp variables for positions of leg links 1 and 4
	dReal tempy1=0.0;
	dReal tempz1=0.0;
	dReal tempx5=0.0;
	dReal tempy5=0.0;
	dReal tempz5=0.0;

	//creating 10 segments for trunk
	for( int I = 0; I < nSegments; I++ )			//10 links exactly.
	{
		GOdeObject *link;											//Create a new body link.
		link = new GOdeObject;
		(*link).body = dBodyCreate( world );						//Attach new body to world.
		(*link).userData = 0;										//0 means salamander's head/neck/trunk/tail link.
		
		dReal x = prevX + length[I]/2.0;
		dReal y = sPosition[1];
		dReal z = sPosition[2];
		dBodySetPosition( (*link).body, x, y, z );					//Position (*link).
		if(I==1){tempx1=x;tempy1=y;tempz1=z;}
		if(I==5){tempx5=x;tempy5=y;tempz5=z;}
		
		dBodySetLinearVel( (*link).body, 0.0, 0.0, 0.0 );			//Initial linear velocity.
		
		dMass M;
		dMassSetBoxTotal( &M, mass[I], length[I], height[I], width[I]);
		dBodySetMass( (*link).body, &M );							//Mass distribution.
		
		dGeomID g;
		g = dCreateBox( space, length[I], height[I], width[I] );	//Geometry.
		dGeomSetData( g, &((*link).userData) );						//Attach a user data pointer to geometry.

		(*link).geometries.push_back( g );
		dGeomSetBody( (*link).geometries[0], (*link).body );		//(*link) body to geometry.
		
		if( I < 2 )			//Head or neck?
			(*link).setObjectColor( 0.0, 0.8, 0.5 );				//Change color.
		else
		{
			if( I < 5 )		//Trunk?
				(*link).setObjectColor( 0.5, 0.8, 0.0 );
			else			//Tail.
				(*link).setObjectColor( 0.7, 0.0, 0.7 );
		}

		links.push_back( *link );								//Attach new link to the list of links.
		prevX += length[I] + jointSpace;						//Making space for joint in between links.
	}

	//Create hinge joints for the 10 links in the body.
	dReal hingeAxis[] = { 0.0, 1.0, 0.0 };						//Hinge axis is parallell to y axis.
	dReal phaseStep = 2.0*GDrawing::pi/9.0;						//In case it is swimmer. 

	//create joints for trunk
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
		dJointSetHingeParam( joint, dParamLoStop, -bodyMaxAngleDeformation );	//Sets limits for hinge angles.
		dJointSetHingeParam( joint, dParamHiStop, bodyMaxAngleDeformation );

		GSJoint gsJoint;										//Create object to hold information on this joint.
		gsJoint.M1 = 0.0;										//Initialize neurosignals for this body joint.
		gsJoint.M2 = 0.0;
		gsJoint.amplitudeLeft = amplitude;						//Initialize turning signal to normal amplitude.
		gsJoint.amplitudeRight = amplitude;						//Make turning amplitudes for body joints equal.
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
	
	//Create segments for legs.
	int foreLimbsAttachedTo = 1;								//Indicate which body link limbs are attached to.
	int hindLimbsAttachedTo = 5;
	for( int i = 1; i < 5; i++ )
	{
		GOdeObject *link;										//Create a new leg link.
		link = new GOdeObject;
		(*link).body = dBodyCreate( world );					//Attach new leg's body to world.
		(*link).userData = i* 10;									//1-4 means salamander's leg segments.

		//Decide where to attach legs to main salamander's body.
		if( i < 3 )
			dBodySetPosition( (*link).body, 
				tempx1, 
				tempy1, 
				tempz1 + pow(-1.0, i) * ( width[foreLimbsAttachedTo] + lLength[0] + jointSpace ) );	
		else
			dBodySetPosition( (*link).body, 
				tempx5, 
				tempy5, 
				tempz5 + pow(-1.0, i) * ( width[hindLimbsAttachedTo] + lLength[0] + jointSpace ) ); //Position (*link).
		
		dBodySetLinearVel( (*link).body, 0.0, 0.0, 0.0 );	//Initial linear velocity.
		
		dMass M;
		dMassSetBoxTotal( &M, lMass[i-1], lWidth[i-1], height[i-1], lLength[i-1]);	//Consider here that length is along z axis.
		dBodySetMass( (*link).body, &M );					//Mass distribution.

		dGeomID g;											//Geometry.
		g = dCreateBox( space, lWidth[i-1], height[i-1], lLength[i-1] );			//Consider same change of geometri as in M.
		dGeomSetData( g, &((*link).userData) );				//Attach a user data pointer to geometry.
		(*link).geometries.push_back( g );					//Attach geometry to object geoms' list.
		dGeomSetBody( (*link).geometries[0], (*link).body );		//Link leg's body to geometry.

		(*link).setObjectColor( 0.8, 0.5, 0.0 );			//Color for legs.
			
		links.push_back( (*link) );							//Add leg to salamander's segment list.
	}

	//Create joints for the four leg segments.
	for( int I = nSegments; I < nSegments + 4 ; I++ )		//Start allocating legs
	{
		dJointID joint;
		joint = dJointCreateHinge( world, jointGroup );		//Create a hinge.
		
		if(I < nSegments + 2)
			dJointAttach( joint, links[foreLimbsAttachedTo].body, links[I].body );	//Attach forelimbs to body segments.
		else 
			dJointAttach( joint, links[hindLimbsAttachedTo].body, links[I].body );	//Attach hindlimbs to body segments.
		
		dReal anchor[3];										//Anchor position for this joint.
		const dReal *pos1, *pos2;								//Positions for object 1 and 2.
		pos1 = dBodyGetPosition( links[I].body );				//Leg's position.
		
		if( I < nSegments + 2)	
			pos2 = dBodyGetPosition( links[foreLimbsAttachedTo].body );	//Parent segment of forelimbs.
		else 
			pos2 = dBodyGetPosition( links[hindLimbsAttachedTo].body );	//Parent segment of hindlimbs.

		anchor[0] = ( pos1[0] + pos2[0] ) / 2.0;				//Get the mid point for x and y coordinates.
		anchor[1] = ( pos1[1] + pos2[1] ) / 2.0;
		if( I < nSegments + 2 )									//Check from which segment's width extract needed half.
			anchor[2] = pos2[2] + pow( -1.0, I - nSegments + 1 ) * ( width[foreLimbsAttachedTo]/2.0 + jointSpace/2.0 );
		else
			anchor[2] = pos2[2] + pow( -1.0, I - nSegments + 1 ) * ( width[hindLimbsAttachedTo]/2.0 + jointSpace/2.0 );
		dJointSetHingeAnchor( joint, anchor[0], anchor[1], anchor[2] );

		dJointSetHingeAxis( joint, hingeAxis[0], hingeAxis[1], hingeAxis[2] );	//Set axis for hing joint.
		dJointSetHingeParam( joint, dParamLoStop, -legMaxAngleDeformation );			//Sets limits for hinge angle.
		dJointSetHingeParam( joint, dParamHiStop, legMaxAngleDeformation );

		GSJoint gsJoint;										//Create object to hold information on this joint.
		gsJoint.M1 = 0.0;										//Initialize neurosignals for this leg joint.
		gsJoint.M2 = 0.0;
		gsJoint.amplitudeLeft = amplitude;						//Limbs will not change their amplitude.
		gsJoint.amplitudeRight = amplitude;
		gsJoint.joint = joint;									//Attach joint just created.
		
		gsJoint.lambda = 0.0;									//Joints in legs are in phase.

		gsJoint.type = 'l';										//'l' is for leg joint.

		gsJoints.push_back( gsJoint );							//Attach joint information to the general list.
	}
}

/*******************************************************************************
Function to compute internal torques according to neurosignals.
*******************************************************************************/
void GSalamander::computeForces( dReal simulationTime, dReal simulationStep )
{
	//////////////////// Accumulating torques on every joint ///////////////////

	dReal Ml, Mr;						//Neurosignals for body joints (left and right).
	dReal desiredVelocity;				//Resulting desired velocity.
	//dReal targetDeformationAngle;		//It is defined every time that a change of activation from 0 to >0 happens.

	for( unsigned I = 0; I < gsJoints.size(); I ++ )
	{
		
		Ml = max( sin( freq*simulationTime - gsJoints[I].lambda ), 0.0 );			//Compute new neurosignals.
		Mr = max( sin( freq*simulationTime - gsJoints[I].lambda + GDrawing::pi ), 0.0 );

		//Detect when a change of activation signal has happened from left to right (and viceversa).
		if( Ml > 0.0 && gsJoints[I].M1 <= 0.0 )										//Left signal just got activated beyond 0?
		{
			targetDeformationAngle = gsJoints[I].amplitudeLeft * bodyMaxAngleDeformation;				//New target angle.
			gsJoints[I].deltaAngle = targetDeformationAngle - dJointGetHingeAngle( gsJoints[I].joint );	//New angular distance..
		}
		else
		{
			if( Mr > 0.0 && gsJoints[I].M2 <= 0.0 )									//Right signal just got activated beyond 0?
			{
				targetDeformationAngle = -gsJoints[I].amplitudeRight * bodyMaxAngleDeformation;				//New target angle.
				gsJoints[I].deltaAngle = targetDeformationAngle - dJointGetHingeAngle( gsJoints[I].joint );	//New angular distance.
			}
		}


		if(gsJoints[I].type == 'l')
		{

			dBodyID bodyID = dJointGetBody(gsJoints[I].joint,1);

			for(int i = 0; i < links.size(); i++)
			{
				if(links[i].body == bodyID)
				{

					//cout << "userData: " << links[i].userData << " deltaAngle: " << gsJoints[I].deltaAngle << endl;
					// first leg
					if(gsJoints[I].deltaAngle > 0 && links[i].userData >= 10 && links[i].userData< 20)
					{
						//cout << "userData: " << links[i].userData << endl;
						links[i].userData = 11;
						dGeomSetData(links[i].geometries[0],&links[i].userData);
					}
					else if(gsJoints[I].deltaAngle < 0 && links[i].userData >= 10 && links[i].userData< 20)
					{
						//cout << "userData: " << links[i].userData << endl;
						links[i].userData = 12;
						dGeomSetData(links[i].geometries[0],&links[i].userData);
					}

					// second leg
					if(gsJoints[I].deltaAngle > 0 && links[i].userData >= 20 && links[i].userData< 30)
					{
						links[i].userData = 21;
						dGeomSetData(links[i].geometries[0],&links[i].userData);
					}
					else if(gsJoints[I].deltaAngle < 0 && links[i].userData >= 20 && links[i].userData< 30)
					{
						links[i].userData = 22;
						dGeomSetData(links[i].geometries[0],&links[i].userData);
					}

					// third leg
					if(gsJoints[I].deltaAngle > 0 && links[i].userData >= 30 && links[i].userData< 40)
					{
						links[i].userData = 31;
						dGeomSetData(links[i].geometries[0],&links[i].userData);
					}
					else if(gsJoints[I].deltaAngle < 0 && links[i].userData >= 30 && links[i].userData< 40)
					{
						links[i].userData = 32;
						dGeomSetData(links[i].geometries[0],&links[i].userData);
					}

					// fourth leg
					if(gsJoints[I].deltaAngle > 0 && links[i].userData >= 40 && links[i].userData< 50)
					{
						links[i].userData = 41;
						dGeomSetData(links[i].geometries[0],&links[i].userData);
					}
					else if(gsJoints[I].deltaAngle < 0 && links[i].userData >= 40 && links[i].userData< 50)
					{
						links[i].userData = 42;
						dGeomSetData(links[i].geometries[0],&links[i].userData);
					}

				}

			}
		}
		

		//Update joint neurosignals.
		gsJoints[I].M1 = Ml;
		gsJoints[I].M2 = Mr;
			
		desiredVelocity = gsJoints[I].deltaAngle * freq / 2.0 * ( Ml + Mr );
		dJointSetHingeParam( gsJoints[I].joint, dParamVel, desiredVelocity );
		dJointSetHingeParam( gsJoints[I].joint, dParamFMax, 0.01 );
			
		/*cout << "Delta Angle: (" << gsJoints[I].deltaAngle << ") Psi: (" << dJointGetHingeAngle(gsJoints[I].joint) << ") Velocity: (" << desiredVelocity << ")" << endl;
		cout << "Ml: " << Ml << "      Mr: " << Mr << endl;*/
			 


	}
}

/*******************************************************************************
Function to make salamander turn by changing the appropriate amplitude signals.
signal is a value from -1 to +1. To make a significant turn |signal| > 0.1.
Turn left when signal > 0.1, turn right when signal < -0.1.
To make salamander go straight, |signal| <= 0.1.
*******************************************************************************/
void GSalamander::turn( dReal signal )
{
	dReal deltaTurn = 0.8*(1.0 - amplitude);	//Maximum difference allowed for turning signals.
	dReal targetAmplitude = amplitude + deltaTurn*abs( signal );

	if( signal > 0.1 )		//Turn left?
	{
		for( unsigned I = 0; I < gsJoints.size(); I++ )			//Modify only body joints.
		{
			if( gsJoints[I].type == 'b' )
			{
				gsJoints[I].amplitudeLeft = targetAmplitude;	//Only left amplitude is updated.
				gsJoints[I].amplitudeRight = amplitude;			//Change right amplitude to normal amplitude.
			}
		}
	}
	else
	{
		if( signal < -0.1 )	//Turn right?
		{
			for( unsigned I = 0; I < gsJoints.size(); I++ )			//Modify only body joints.
			{
				if( gsJoints[I].type == 'b' )
				{
					gsJoints[I].amplitudeRight = targetAmplitude;	//Only right amplitude is updated.
					gsJoints[I].amplitudeLeft = amplitude;			//Change left amplitude to normal amplitude.
				}
			}
		}
		else				//If not turning to any side, return to normal amplitudes.
		{
			for( unsigned I = 0; I < gsJoints.size(); I++ )			//Modify only body joints.
			{
				if( gsJoints[I].type = 'b' )
					gsJoints[I].amplitudeLeft = gsJoints[I].amplitudeRight = amplitude;	//Back to default values.
			}
		}
	}
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

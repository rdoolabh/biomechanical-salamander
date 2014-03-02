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

	bodyMaxAngleDeformation = GDrawing::pi/4.0;	//Tell ODE that body hinges should not deform beyond this.
	legMaxAngleDeformation = GDrawing::pi/6.0;	//Tell ODE that legs' joints should not go beyond this.
	kneeMaxAngleDeformation = 0.0;				//Tell ODE that knees should not go beyond this value.
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
	Leg- FL		0.025			0.01			0.002
	Leg- FR		0.025			0.01			0.002
	Leg- BL		0.025			0.01			0.002
	Leg- BR		0.025			0.01			0.002
	Leg2-FL		0.02			0.01			0.0012
	Leg2-FR		0.02			0.01			0.0012
	Leg2-BL		0.02			0.01			0.0012
	Leg2-BR		0.02			0.01			0.0012
	//////////////////////////////////////////////////////////////////////////*/
	dReal length[] = { 0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03 };
	dReal width[] = { 0.03, 0.025, 0.03, 0.03, 0.03, 0.022, 0.02, 0.016, 0.01, 0.005 };	 //Encoding of biometrics.
	dReal height[] = { 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04 };
	dReal mass[] = { 0.0177, 0.0147, 0.0177, 0.0177, 0.0177, 0.013, 0.0118, 0.0094, 0.0059, 0.0029 };
	dReal lLength[] = { 0.025, 0.025, 0.025,0.025};
	dReal lWidth[] = {0.01, 0.01, 0.01, 0.01 };
	dReal lMass[] = { 0.002, 0.002, 0.002, 0.002 };
	dReal lLength2[] = { 0.02, 0.02, 0.02, 0.02 };		//The leg calves.
	dReal lWidth2[] = { 0.01, 0.01, 0.01, 0.01 };
	dReal lMass2[] = { 0.0012, 0.0012, 0.0012, 0.0012 };

	//Create links for head, neck, trunk, and tail; all with length 0.03.
	//Salamander's body grows along the an axis parallel to x.
	int nSegments = 10;
	dReal prevX = sPosition[0];						//To know where the previous link ended (considering joint space too).
	dReal jointSpace = 0.02;						//Empty space to accomodate internal joint.
	int foreLimbsAttachedTo = 1;								//Indicate which body link limbs are attached to.
	int hindLimbsAttachedTo = 5;

	dReal tempx1=0.0;								//temp variables for positions of leg links 1 and 4
	dReal tempy1=0.0;
	dReal tempz1=0.0;
	dReal tempx5=0.0;
	dReal tempy5=0.0;
	dReal tempz5=0.0;

	//Creating 10 segments for trunk
	for( int I = 0; I < nSegments; I++ )			//10 links exactly.
	{
		GOdeObject *link;											//Create a new body link.
		link = new GOdeObject;
		(*link).body = dBodyCreate( world );						//Attach new body to world.

		(*link).userData = (unsigned short)I;						//0 means salamander's head/neck/trunk/tail link.
																	//But also attach its index.
		
		dReal x = prevX + length[I]/2.0;
		dReal y = sPosition[1];
		dReal z = sPosition[2];
		dBodySetPosition( (*link).body, x, y, z );					//Position (*link).
		if(I==foreLimbsAttachedTo){tempx1=x;tempy1=y;tempz1=z;}
		if(I==hindLimbsAttachedTo){tempx5=x;tempy5=y;tempz5=z;}
		
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

	//Create joints for trunk
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
	int lowerLimbsAttachedTo[4];								//Four link indices for upper limbs.
	for( int i = 1; i < 5; i++ )
	{
		GOdeObject *link;										//Create a new leg link.
		link = new GOdeObject;
		unsigned short legNumber = (unsigned short)i;
		unsigned short xCode = 0x0000;							//We will use the following encoding:
																//0xABCD, where A -> 0 = body.
																//				     1 = front left leg.
																//                   2 = front right leg.
																//                   3 = rear left leg.
																//                   4 = rear right leg.
		                                                        //              B -> 0 = no friction.
																//                   1 = friction.
																//             CD -> link index.
		
		xCode += (unsigned short)(i + nSegments - 1);			//Encode link list index.
		legNumber <<= 12;										//Leg encoding is of the form i0, indicating not friction.
		xCode |= legNumber;
		(*link).body = dBodyCreate( world );					//Attach new leg's body to world.
		(*link).userData = xCode;								//Now, user data contains information in the form 0xA0CD.

		lowerLimbsAttachedTo[i-1] = i + nSegments - 1;			//Recall indexes of upper limb links (left, right, left, right).

		//Decide where to attach legs to main salamander's body.
		//It creates leg to the left ( positive z axis ) first, then right.
		if( i < 3 )
			dBodySetPosition( (*link).body, 
				tempx1, 
				tempy1, 
				tempz1 + pow(-1.0, i-1) * ( width[foreLimbsAttachedTo]/2.0 + lLength[i-1]/2.0 + jointSpace ) );	
		else
			dBodySetPosition( (*link).body, 
				tempx5, 
				tempy5, 
				tempz5 + pow(-1.0, i-1) * ( width[hindLimbsAttachedTo]/2.0 + lLength[i-1]/2.0 + jointSpace ) ); //Position (*link).
		
		dBodySetLinearVel( (*link).body, 0.0, 0.0, 0.0 );	//Initial linear velocity.
		
		dMass M;
		dMassSetBoxTotal( &M, lMass[i-1], lWidth[i-1], lWidth[i-1], lLength[i-1]);	//Consider here that length is along z axis.
		dBodySetMass( (*link).body, &M );					//Mass distribution.

		dGeomID g;											//Geometry.
		g = dCreateBox( space, lWidth[i-1], lWidth[i-1], lLength[i-1] );			//Consider same change of geometri as in M.
		dGeomSetData( g, &((*link).userData) );				//Attach a user data pointer to geometry.
		(*link).geometries.push_back( g );					//Attach geometry to object geoms' list.
		dGeomSetBody( (*link).geometries[0], (*link).body );		//Link leg's body to geometry.

		(*link).setObjectColor( 0.8, 0.5, 0.0 );			//Color for legs.
			
		links.push_back( (*link) );							//Add leg to salamander's segment list.
	}

	//Create joints for the four leg segments.
	for( int I = nSegments; I < nSegments + 4 ; I++ )		//Start allocating leg joints.
	{
		dJointID joint;
		joint = dJointCreateHinge( world, jointGroup );		//Create a hinge.
		
		//Attach segments to joints. We will follow the order of first body attached if the leg body is
		//on the left of the salamander, and second body attached if the leg body is to the right.
		//This way, we can use neurosignals with the convention of the paper by assigning same
		//phase to cross-side leg joints, and out-of-phase to leg joints along the same side of the
		//creature.
		if( I == nSegments ) dJointAttach( joint, links[I].body, links[foreLimbsAttachedTo].body );			//To the left.
		if( I == nSegments + 1 ) dJointAttach( joint, links[foreLimbsAttachedTo].body, links[I].body );		//To the right.
		if( I == nSegments + 2 ) dJointAttach( joint, links[I].body, links[hindLimbsAttachedTo].body );		//To the left.
		if( I == nSegments + 3 ) dJointAttach( joint, links[hindLimbsAttachedTo].body, links[I].body );		//To the right.
		
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
			anchor[2] = pos2[2] + pow( -1.0, I - nSegments ) * ( width[foreLimbsAttachedTo]/2.0 + jointSpace/2.0 );
		else
			anchor[2] = pos2[2] + pow( -1.0, I - nSegments ) * ( width[hindLimbsAttachedTo]/2.0 + jointSpace/2.0 );
		dJointSetHingeAnchor( joint, anchor[0], anchor[1], anchor[2] );

		dJointSetHingeAxis( joint, hingeAxis[0], hingeAxis[1], hingeAxis[2] );	//Set axis for hing joint.
		dJointSetHingeParam( joint, dParamLoStop, -legMaxAngleDeformation );			//Sets limits for hinge angle.
		dJointSetHingeParam( joint, dParamHiStop, legMaxAngleDeformation );

		GSJoint gsJoint;										//Create object to hold information on this joint.
		gsJoint.M1 = 0.0;										//M1 activates protactor "muscle" for all leg joints.
		gsJoint.M2 = 0.0;										//M2 activates retractor "muscle" for all leg joints.
		gsJoint.amplitudeLeft = amplitude;						//Limbs will not change their amplitude.
		gsJoint.amplitudeRight = amplitude;
		gsJoint.joint = joint;									//Attach joint just created.
		
		if( lander )											//Synchronize front left leg with trunk, and from there
		{														//synchronize the rest of the limbs according to paper.
			if( I == nSegments ) gsJoint.lambda = 0.0;
			if( I == nSegments + 1 ) gsJoint.lambda = GDrawing::pi;
			if( I == nSegments + 2 ) gsJoint.lambda = GDrawing::pi;
			if( I == nSegments + 3 ) gsJoint.lambda = 0.0;
		}
		else
			gsJoint.lambda = 0.0;								//For swimmer, it doesn't matter phase in legs.

		gsJoint.type = 'l';										//'l', 'm', 'n', 'o' is for leg joint.
		gsJoint.type += I - nSegments;							//'l' and 'm' are front left and right legs, 
																//'n' and 'o' are left and right rear legs.
		gsJoints.push_back( gsJoint );							//Attach joint information to the general list.
	}

	///////////////////// Create the calves for all legs ///////////////////////
	for( int I = 0; I < 4; I++ )								//Start with the segments.
	{
		GOdeObject *link;										//Create a new leg link.
		link = new GOdeObject;
		unsigned short calfNumber = (unsigned short)( I+5 );
		unsigned short xCode = 0x0000;							//We will use the following encoding:
																//0xABCD, where A -> 0 = body.
																//				     5 = front left calf.
																//                   6 = front right calf.
																//                   7 = rear left calf.
																//                   8 = rear right calf.
		                                                        //              B -> 0 = no friction.
																//                   1 = friction.
																//             CD -> link index.
		
		xCode += (unsigned short)(I + nSegments + 4);			//Encode link list index.
		calfNumber <<= 12;										//Calf encoding is of the form (I+5)0, indicating not friction.
		xCode |= calfNumber;
		(*link).body = dBodyCreate( world );					//Attach new leg's body to world.
		(*link).userData = xCode;								//Now, user data contains information in the form 0xA0CD.

		//Decide where to attach calves to upper legs.
		//Follow convention left, right, left, right.
		dMatrix3 rotation;
		dRFromAxisAndAngle( rotation, 1.0, 0.0, 0.0, GDrawing::pi/2.0 );
		dBodySetRotation( (*link).body, rotation );				//Rotate body 90 degrees around positive x axis.
		if( I < 2 )
		{
			dBodySetPosition( (*link).body, 
				tempx1, 
				tempy1 - lLength2[I]/2.0 - jointSpace/2.0, 
				tempz1 + pow(-1.0, I) * ( width[foreLimbsAttachedTo]/2.0 + lLength[I] + 3.0*jointSpace/2.0 ) );	//Position.
		}
		else
		{
			dBodySetPosition( (*link).body, 
				tempx5, 
				tempy5 - lLength2[I]/2.0 - jointSpace/2.0, 
				tempz5 + pow(-1.0, I) * ( width[hindLimbsAttachedTo]/2.0 + lLength[I] + 3.0*jointSpace/2.0 ) ); //Position.
		}
		
		dBodySetLinearVel( (*link).body, 0.0, 0.0, 0.0 );	//Initial linear velocity.
		
		dMass M;
		dMassSetCapsuleTotal( &M, lMass2[I], 3, lWidth2[I]/2.0, lLength2[I] - lWidth2[I] );	//Mass in a capsule goes along local z-axis.
		dBodySetMass( (*link).body, &M );					//Mass distribution (according to formula, we have to do length - 2*radius).

		dGeomID g;											//Geometry.
		g = dCreateCapsule( space, lWidth2[I]/2.0, lLength2[I] - lWidth2[I] );	//It is a capsule oriented along the y axis.
		dGeomSetData( g, &((*link).userData) );				//Attach a user data pointer to geometry.
		(*link).geometries.push_back( g );					//Attach geometry to object geoms' list.
		dGeomSetBody( (*link).geometries[0], (*link).body );		//Link leg's body to geometry.

		(*link).setObjectColor( 0.3, 0.5, 0.75 );			//Color for calves.
			
		links.push_back( (*link) );							//Add calf to salamander's segment list.
	}
	
	//Create joints for knees.
	dReal kneeAxis[] = { 1.0, 0.0, 0.0 };
	for( int I = nSegments + 4; I < nSegments + 8 ; I++ )	//Start allocating knees joints.
	{
		dJointID joint;
		joint = dJointCreateHinge( world, jointGroup );		//Create a hinge.
		
		//For knees on the left, calves are body 0. For knees in the right, calves are body 1.
		if( I == nSegments + 4 ) dJointAttach( joint, links[I].body, links[lowerLimbsAttachedTo[I-nSegments-4]].body );	//To the left.
		if( I == nSegments + 5 ) dJointAttach( joint, links[lowerLimbsAttachedTo[I-nSegments-4]].body, links[I].body );	//To the right.
		if( I == nSegments + 6 ) dJointAttach( joint, links[I].body, links[lowerLimbsAttachedTo[I-nSegments-4]].body );	//To the left.
		if( I == nSegments + 7 ) dJointAttach( joint, links[lowerLimbsAttachedTo[I-nSegments-4]].body, links[I].body );	//To the right.
		
		dReal anchor[3];										//Anchor position for this joint.
		const dReal *pos1, *pos2;								//Positions for object 1 and 2.
		pos1 = dBodyGetPosition( links[I].body );				//Leg's position.
		
		pos2 = dBodyGetPosition( links[lowerLimbsAttachedTo[I-nSegments-4]].body );	//Parent upper leg.

		anchor[0] = pos2[0];									//X and Y coordinate for anchor are the same as parent.
		anchor[1] = pos2[1];									//Displace anchor by half the length of the upper leg from that
		anchor[2] = pos2[2] + pow( -1.0, I-nSegments-4 ) * ( lLength[I-nSegments-4]/2.0 + jointSpace/2.0 );	//segment's position.
		dJointSetHingeAnchor( joint, anchor[0], anchor[1], anchor[2] );

		dJointSetHingeAxis( joint, kneeAxis[0], kneeAxis[1], kneeAxis[2] );				//Set axis for knee joint.
		dJointSetHingeParam( joint, dParamLoStop, -kneeMaxAngleDeformation );			//Sets limits for hinge angle.
		dJointSetHingeParam( joint, dParamHiStop, kneeMaxAngleDeformation );

		GSJoint gsJoint;										//Create object to hold information on this joint.
		gsJoint.M1 = 0.0;										//M1 activates flexor "muscle" for all leg joints.
		gsJoint.M2 = 0.0;										//M2 activates retractor "muscle" for all leg joints.
		gsJoint.amplitudeLeft = amplitude;						//Limbs will not change their amplitude.
		gsJoint.amplitudeRight = amplitude;
		gsJoint.joint = joint;									//Attach joint just created.
		
		if( lander )											//Synchronize knee with its parent upper.
		{
			if( I == nSegments + 4 ) gsJoint.lambda = 0.0;
			if( I == nSegments + 5 ) gsJoint.lambda = GDrawing::pi;
			if( I == nSegments + 6 ) gsJoint.lambda = GDrawing::pi;
			if( I == nSegments + 7 ) gsJoint.lambda = 0.0;
		}
		else
			gsJoint.lambda = 0.0;								//For swimmer, it doesn't matter phase in legs.

		gsJoint.type = 'p';										//'p', 'q', 'r', 's' is for knee joints.
		gsJoint.type += I - nSegments - 4;						//'p' and 'q' are front left and right knees, 
																//'r' and 's' are left and right rear knees.
		gsJoints.push_back( gsJoint );							//Attach joint information to the general list.
	}
}

/*******************************************************************************
Function to compute internal torques according to neurosignals.
*******************************************************************************/
void GSalamander::computeForces( dReal simulationTime, dReal simulationStep )
{
	//////////////////// Accumulating torques on every joint ///////////////////

	dReal M1, M2;						//Neurosignals for body joints (left and right).
	dReal desiredVelocity;				//Resulting desired velocity.
	dReal targetDeformationAngle;		//It is defined every time that a change of activation from 0 to >0 happens.

	for( unsigned I = 0; I < gsJoints.size(); I ++ )
	{
		
		M1 = max( sin( freq*simulationTime - gsJoints[I].lambda ), 0.0 );				//Compute new neurosignals.
		M2 = max( sin( freq*simulationTime - gsJoints[I].lambda + GDrawing::pi ), 0.0 );

		if( gsJoints[I].type == 'b' )	//Is it a head/trunk/tail joint?
		{
			//Detect when a change of activation signal has happened from left to right (and vice versa).
			if( M1 > 0.0 && gsJoints[I].M1 <= 0.0 )										//Left signal just got activated beyond 0?
			{
				targetDeformationAngle = gsJoints[I].amplitudeLeft * bodyMaxAngleDeformation;				//New target angle.
				gsJoints[I].deltaAngle = targetDeformationAngle - dJointGetHingeAngle( gsJoints[I].joint );	//New angular distance..
			}
			else
			{
				if( M2 > 0.0 && gsJoints[I].M2 <= 0.0 )									//Right signal just got activated beyond 0?
				{
					targetDeformationAngle = -gsJoints[I].amplitudeRight * bodyMaxAngleDeformation;				//New target angle.
					gsJoints[I].deltaAngle = targetDeformationAngle - dJointGetHingeAngle( gsJoints[I].joint );	//New angular distance.
				}
			}
		
			//Compute desired velocity, depending on current time within [0, pi] for a given joint.
			desiredVelocity = gsJoints[I].deltaAngle * freq / 2.0 * ( M1 + M2 );
			dJointSetHingeParam( gsJoints[I].joint, dParamVel, desiredVelocity );
			//dJointSetHingeParam( gsJoints[I].joint, dParamFMax, 0.062 );
			dJointSetHingeParam( gsJoints[I].joint, dParamFMax, 0.05 );
		}
		
		//Add motion to shoulders and hips.
		if( gsJoints[I].type >= 'l' && gsJoints[I].type <= 'o' )
		{
			//Detect a change of activation signal from protactor / retractor (and vice versa).
			if( M1 > 0.0 && gsJoints[I].M1 <= 0.0 )										//Retractor signal just got activated beyond 0?
			{
				targetDeformationAngle = gsJoints[I].amplitudeLeft * legMaxAngleDeformation;				//New target angle.
				gsJoints[I].deltaAngle = targetDeformationAngle - dJointGetHingeAngle( gsJoints[I].joint );	//New angular distance.
			}
			else
			{
				if( M2 > 0.0 && gsJoints[I].M2 <= 0.0 )									//Protactor signal just got activated beyond 0?
				{
					targetDeformationAngle = -gsJoints[I].amplitudeRight * legMaxAngleDeformation;				//New target angle.
					gsJoints[I].deltaAngle = targetDeformationAngle - dJointGetHingeAngle( gsJoints[I].joint );	//New angular distance.
				}
			}
		
			//Compute desired velocity, depending on current time within [0, pi] for a given joint.
			desiredVelocity = gsJoints[I].deltaAngle * freq / 2.0 * ( M1 + M2 );
			dJointSetHingeParam( gsJoints[I].joint, dParamVel, desiredVelocity );
			dJointSetHingeParam( gsJoints[I].joint, dParamFMax, 0.072 );
		}
		
		//Now, modify friction on calves depending on which neurosignal is active for their knees.
		if( gsJoints[I].type >= 'p' && gsJoints[I].type <= 's')				//Is it a knee?
		{
			dBodyID legBody;
			dGeomID legGeom;
			unsigned short* xCode;											//User data encoded.
			
			if( gsJoints[I].type == 'p' || gsJoints[I].type == 'r' )		//Joints to the left of the salamander? 
				legBody = dJointGetBody( gsJoints[I].joint, 0 );			//Get the body of the calf segment. which is indicated by 0.
			else
				legBody = dJointGetBody( gsJoints[I].joint, 1 );			//To the right, the calf was the second attached body.

			legGeom = dBodyGetFirstGeom( legBody );							//Get the geometry attached to calf's dynamics body.
			xCode = (unsigned short *)dGeomGetData( legGeom );				//Get the user data.
			//linkIndex = ( (*xCode) & 0x00FF );							//Remember that link index is the lower byte.

			//Update the 4-bit friction code in 0xABCD: which is B; keeping constant A and CD.
			unsigned short friction = 0x0000;
			unsigned short newXCode = (*xCode) & 0xF0FF;					//Erase previous friction code.
			if( M1 > 0.0 )
				friction = 0x0100;
			newXCode |= friction;
			(*xCode) = newXCode;											//New user data.

		}

		//Update joint neurosignals.
		gsJoints[I].M1 = M1;
		gsJoints[I].M2 = M2;
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
		for( unsigned I = 0; I < gsJoints.size(); I++ )			
		{
			if( gsJoints[I].type == 'b' && I > 0 &&  I < 5 )	//Modify only body joints.
			{
				gsJoints[I].amplitudeLeft = targetAmplitude;	//Only left amplitude is updated.
				gsJoints[I].amplitudeRight = amplitude;			//Change right amplitude to normal amplitude.
			}
			
			if( gsJoints[I].type == 'l' && gsJoints[I].type == 'o' )	//Crossed knees and shoulders?
				gsJoints[I].amplitudeLeft = gsJoints[I].amplitudeRight = targetAmplitude;
		}
	}
	else
	{
		if( signal < -0.1 )	//Turn right?
		{
			for( unsigned I = 0; I < gsJoints.size(); I++ )			
			{
				if( gsJoints[I].type == 'b' && I > 0 && I < 5 )		//Modify only body joints.
				{
					gsJoints[I].amplitudeRight = targetAmplitude;	//Only right amplitude is updated.
					gsJoints[I].amplitudeLeft = amplitude;			//Change left amplitude to normal amplitude.
				}

				if( gsJoints[I].type == 'm' && gsJoints[I].type == 'n' )	//Crossed knees and shoulders?
					gsJoints[I].amplitudeLeft = gsJoints[I].amplitudeRight = targetAmplitude;
			}
		}
		else				//If not turning to any side, return to normal amplitudes.
		{
			for( unsigned I = 0; I < gsJoints.size(); I++ )			
			{
				if( gsJoints[I].type == 'b' && I > 0 &&  I < 5 )			//Modify only body joints.
					gsJoints[I].amplitudeLeft = gsJoints[I].amplitudeRight = amplitude;	//Back to default values.

				if( gsJoints[I].type >= 'l' && gsJoints[I].type <= 'o' )	//Knees and shoulders?
					gsJoints[I].amplitudeLeft = gsJoints[I].amplitudeRight = amplitude;
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

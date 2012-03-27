#include "RigidBodyPlanner.hpp"
#include <vector>
#include <math.h>
#include <windows.h>
#include <iostream>

using namespace std;

RigidBodyPlanner::RigidBodyPlanner(RigidBodySimulator * const simulator)
{
    m_simulator = simulator;   
	//I added this to see if I got the windows configuration to work.
}

RigidBodyPlanner::~RigidBodyPlanner(void)
{
    //do not delete m_simulator  
}
RigidBodyMove RigidBodyPlanner::ConfigurationMove(void)
{
	RigidBodyMove move;
	
	//The theta from the configuration space
	double configTheta = m_simulator->GetRobotTheta();

	//The x value of the robot in configuration space
	double configX = m_simulator->GetRobotX();

	//The y value of the robot in configuration space
	double configY = m_simulator->GetRobotY();

	double goalX = m_simulator->GetGoalCenterX();
	double goalY = m_simulator->GetGoalCenterY();

	if (!m_simulator->HasRobotReachedGoal()) {

	//Total sum of the different forces
	double totalAttractiveForce[1][3];
	double totalRepulsiveForce[2][1];
	double totalJacobian[2][3];

	//Initialize the counters
	memset(totalAttractiveForce, 0, sizeof(totalAttractiveForce[0][0]) * 1 * 3);
	memset(totalRepulsiveForce, 0, sizeof(totalRepulsiveForce[0][0]) * 2 * 1);
	memset(totalJacobian, 0, sizeof(totalJacobian[0][0]) * 2 * 3);

	const double *robotVertices = m_simulator->GetRobotVertices();

	// Loop over all of the vertices on the robot
	for(int idx = 0; idx<m_simulator->GetNrRobotVertices(); idx++)
	{
		double pointX = robotVertices[2*idx];
		double pointY = robotVertices[2*idx+1];
		
		/* Calculate the Forward Kinematics for this vertex j
		*       / xj cos(theta) - yj sin(theta) + x  \
		* FKj = |                                      |
		*       \ xj sin(theta) + yj cos(theta) + y  /
		* Where j represents the index of the current vertice on the robot
		 */
		double fk[2][1];
		fk[0][0] = (pointX * cos(configTheta)) - (pointY*sin(configTheta)) + (configX);
		fk[1][0] = (pointX * sin(configTheta)) + (pointY*cos(configTheta)) + (configY);
		
		/* Calculate the attractive force for this vertex j
		*                / Goalx \
		* Uattj = FKj - |         |
		*                \ Goaly /
		*
		 */
		double attractiveForce[2][1];
		attractiveForce[0][0] = fk[0][0]-goalX;
		attractiveForce[1][0] = fk[1][0]-goalY;

		/* Calculate the repulsion force for all obstacles with respect to
		* this vertex
		*           / Oix \
		* Urepij = |       |  - FKj
		*           \ Oiy /
		* Where Oiy and Oix are the closest point to this vertex on obstacle i
		 */
		double vertexRepulsiveForce[2][1];
		vertexRepulsiveForce[0][0] = vertexRepulsiveForce[1][0] = 0;
		for (int obsIdx = 0; obsIdx < m_simulator->GetNrObstacles(); obsIdx++){
			Point point = m_simulator->ClosestPointOnObstacle(obsIdx, fk[0][0], fk[1][0]);
			vertexRepulsiveForce[0][0] += (point.m_x - fk[0][0]);
			vertexRepulsiveForce[1][0] += (point.m_y - fk[1][0]);
		}
		
		/* Calculate the Jacobian
		*
		*       / 1 0 -Xj * sin(theta) - Yj * cos(theta) \
		* Jj = |                                          |
		*       \ 0 1  Xj * cos(theta) - Yj * sin(theta) /
		*
		 */
		double  jacobian[2][3];
		jacobian[0][0] = jacobian[1][1] = 1;
		jacobian[0][1] = jacobian[1][0] = 0;
		jacobian[0][2] = (-pointX * sin(configTheta)) - (pointY*cos(configTheta));
		jacobian[1][2] = (pointX * cos(configTheta)) - (pointY *sin(configTheta));
		
		// Calculate the final values for this vertex based on prior Uatt, Urep, Jacobian, and FK
		double  attractiveResult[1][3];
		for(int i = 0; i < 3; i++)
		{
			attractiveResult[0][i] = ( (attractiveForce[0][0]*jacobian[0][i]) + (attractiveForce[1][0]*jacobian[1][i]) );
		}
		
		for(int i=0;i<1;i++)
		{
			for(int j=0;j<3;j++)
			{
				totalAttractiveForce[i][j] +=attractiveResult[i][j];
			}
		}

		for (int i = 0; i < 2; i++){
			for (int j = 0; j < 3; j++){
				totalJacobian[i][j] += jacobian[i][j];
			}
		}

		for(int i = 0; i < 3; i++)
		{
			totalRepulsiveForce[0][0] += (vertexRepulsiveForce[0][0]*jacobian[0][i]);
			totalRepulsiveForce[1][0] += (vertexRepulsiveForce[1][0]*jacobian[1][i]);
		}

	}

	double thetaScale = PI/64;
	double xyScale = .001;
	double repScale = .001;
	double thetaValue = (totalAttractiveForce[0][2]/abs(totalAttractiveForce[0][2]));

	if (thetaValue > 0){
		move.m_dtheta = PI/64;
	}
	else if (thetaValue < 0){
		move.m_dtheta = -(PI/64);
	}

	if (totalRepulsiveForce[0][0] > totalAttractiveForce[0][0]  || totalRepulsiveForce[1][0] > totalAttractiveForce[0][1]){
		repScale = 0.002;
	}
	cout << "Attrac\t" << totalAttractiveForce[0][0] << "\t" << totalAttractiveForce[0][1] << "\t::\t" 
	 	 << "Repuls\t" << totalRepulsiveForce[0][0]  << "\t" << totalRepulsiveForce[1][0]  << endl;

	move.m_dx = -(xyScale*totalAttractiveForce[0][0] + repScale*totalRepulsiveForce[0][0]);
	move.m_dy = -(xyScale*totalAttractiveForce[0][1] + repScale*totalRepulsiveForce[1][0]);

	// Go in slow mode
	Sleep(110);
	}
	return move;
}

/*
RigidBodyMove RigidBodyPlanner::ConfigurationMove(void)
    
{
    RigidBodyMove move;
	double robotX = m_simulator->GetRobotX();
	double robotY = m_simulator->GetRobotY();
	double robotTheta = m_simulator->GetRobotTheta();
	int numberVert = m_simulator->GetNrRobotVertices();
	vector<RobotJacobian> robotJacobian;
	vector<GoalGradient> goalGradient;
	const double *robotVertices = m_simulator->GetRobotVertices();
	RobotJacobian roboJaco;
	GoalGradient aGrad;
	double goalX = m_simulator->GetGoalCenterX();
	double goalY = m_simulator->GetGoalCenterY();

	// Calculate Jacobian for the center of the robot
	roboJaco.mJacobian[0][0] = roboJaco.mJacobian[1][1] =1;
	roboJaco.mJacobian[0][1] = roboJaco.mJacobian[1][0] =0;
	roboJaco.mJacobian[0][2] = -1 * robotX * sin(robotTheta) - robotY * cos(robotTheta);
	roboJaco.mJacobian[1][2] = robotX * cos(robotTheta) - robotY * sin(robotTheta);

	// Not sure if we'll need the center of the robots jacoby but I'll store it anyways
	robotJacobian.push_back(roboJaco);

	// Calculate the gradient for the center of the robot
	// Maybe this is supposed to be FK instead of the simple calculation below
	aGrad.mGoalGradient[0] = robotX - goalX;
	aGrad.mGoalGradient[1] = robotY - goalY;
	goalGradient.push_back(aGrad);

	// Loop through all vertices on this object and get calculate the Jacobian
	double vertX;
	double vertY;
	double obstX;
	double obstY;
	double fk[2];
	double uAtt[2];
	double uRep[2];
	double scale = .01;  // We need to figure out a good scaling
	
	//These are the various scales
	double attrScaleXY = .25;
	double attrScaleTheta =1;
	double repScaleXY = .5;
	double repScaleTheta =PI/4;

	for (int idx = 0; idx < numberVert; idx++){
		RobotJacobian aJacoby;
		vertX = robotVertices[idx * 2];
		vertY = robotVertices[idx * 2 + 1];

		//Calculate the Jacobian (step 2 from notes)
		aJacoby.mJacobian[0][0] = aJacoby.mJacobian[1][1] = 1;
		aJacoby.mJacobian[0][1] = aJacoby.mJacobian[1][0] = 0;
		aJacoby.mJacobian[0][2] = -1 * vertX * sin(robotTheta) - vertY * cos(robotTheta);
		aJacoby.mJacobian[1][2] = vertX * cos(robotTheta) - vertY * sin(robotTheta);

		robotJacobian.push_back(aJacoby);

		// Calculate the gradient to the goal (step 1 from notes)
		// First calculate Forward Kinematic for this vertex on robot
		fk[0] = vertX * cos(robotTheta) - vertY * sin(robotTheta) + robotX;
		fk[1] = vertX * sin(robotTheta) + vertY * cos(robotTheta) + robotY;

		// Next calculate the attractive force
		uAtt[0] = (fk[0] - goalX) * scale;
		uAtt[1] = (fk[1] - goalY) * scale; 

		// Now calculate the repulsion factor for all obstacles with respect to
		// this vertex
		uRep[0] = uRep[1] = 0;
		for (int obsIdx = 0; obsIdx < m_simulator->GetNrObstacles(); obsIdx++){
			Point point = m_simulator->ClosestPointOnObstacle(obsIdx, vertX, vertY);
			uRep[0] += scale * (point.m_x - fk[0]);
			uRep[1] += scale * (point.m_y - fk[1]);
		}


		//Multiply the Jacobian with the attractive force and add it
		move.m_dx -= attrScaleXY*( (aJacoby.mJacobian[0][0] *uAtt[0]) + (aJacoby.mJacobian[1][0]*uAtt[1]) );
		move.m_dy -= attrScaleXY*( (aJacoby.mJacobian[0][1] *uAtt[0]) + (aJacoby.mJacobian[1][1]*uAtt[1]) );
		move.m_dtheta -= attrScaleTheta*( (aJacoby.mJacobian[0][2] *uAtt[0]) + (aJacoby.mJacobian[1][2]*uAtt[1]) );

		//Multiply the Jacobian with the repulsive force and add it
		move.m_dx += repScaleXY*( (aJacoby.mJacobian[0][0] *uRep[0]) + (aJacoby.mJacobian[1][0]*uRep[1]) );
		move.m_dy += repScaleXY*( (aJacoby.mJacobian[0][1] *uRep[0]) + (aJacoby.mJacobian[1][1]*uRep[1]) );
		move.m_dtheta += repScaleTheta *( (aJacoby.mJacobian[0][2] *uRep[0]) + (aJacoby.mJacobian[1][2]*uRep[1]) );
	}

	double degrees = move.m_dtheta *180/PI;
	printf("Moving to[%4.3f, %4.3f, %4.3f] with heading[%4.3f]\n",move.m_dx, move.m_dy, degrees);
	Sleep(1);
    return move;
}
*/

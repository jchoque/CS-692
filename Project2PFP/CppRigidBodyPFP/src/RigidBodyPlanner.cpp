#include "RigidBodyPlanner.hpp"
#include <vector>
#include <math.h>
#include <windows.h>
#include <iostream>
#include <iomanip>

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
	double totalAttractiveForce[3];
	double totalRepulsiveForce[2];
	double totalJacobian[2][3];

	//Initialize the counters
	memset(totalAttractiveForce, 0, sizeof(totalAttractiveForce[0]) * 1 * 3);
	memset(totalRepulsiveForce, 0, sizeof(totalRepulsiveForce[0]) * 2 * 1);
	memset(totalJacobian, 0, sizeof(totalJacobian[0][0]) * 2 * 3);

	const double *robotVertices = m_simulator->GetRobotVertices();

	// Loop over all of the vertices on the robot
	for(int idx = 0; idx<m_simulator->GetNrRobotVertices(); idx++)
	{
		double pointX = robotVertices[2*idx];
		double pointY = robotVertices[2*idx+1];
		// Used for normalizing calculation
		double goalDistance = sqrt(pow(pointX - goalX, 2) + pow(pointY - goalY, 2));
		
		/* Calculate the Forward Kinematics for this vertex j
		*       / xj cos(theta) - yj sin(theta) + x  \
		* FKj = |                                      |
		*       \ xj sin(theta) + yj cos(theta) + y  /
		* Where j represents the index of the current vertice on the robot
		 */
		double fk[2][1];
		// Normalize these x/y values?
		fk[0][0] = (pointX * cos(configTheta)) - (pointY*sin(configTheta)) + (configX);
		fk[1][0] = (pointX * sin(configTheta)) + (pointY*cos(configTheta)) + (configY);
		
		/* Calculate the attractive force for this vertex j
		*                / Goalx \
		* Uattj = FKj - |         |
		*                \ Goaly /
		*
		 */
		double attractiveForce[2][1];
		attractiveForce[0][0] = pointX - goalX;
		attractiveForce[1][0] = pointY - goalY;

		/* Calculate the repulsion force for all obstacles with respect to
		* this vertex
		*           / Oix \
		* Urepij = |       |  - FKj
		*           \ Oiy /
		* Where Oiy and Oix are the closest point to this vertex on obstacle i
		 */
		double vertexRepulsiveForce[2];
		memset(vertexRepulsiveForce, 0, sizeof(vertexRepulsiveForce[0]) * 2 * 1);
		for (int obsIdx = 0; obsIdx < m_simulator->GetNrObstacles(); obsIdx++){

			//The closest point should pass in the pointX, pointY
			//Point point = m_simulator->ClosestPointOnObstacle(obsIdx, fk[0][0], fk[1][0]);
			Point point = m_simulator->ClosestPointOnObstacle(obsIdx, pointX, pointY);
			// Used for normalizing calculation
			double obsDistance = sqrt(pow(pointX - point.m_x, 2) + pow(pointY - point.m_y, 2));	

			// Normalize these x/y values?
			vertexRepulsiveForce[0] += (pointX - point.m_x);
			vertexRepulsiveForce[1] += (pointY - point.m_y);
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
		double  attractiveResult[3];
		memset(attractiveResult, 0, sizeof(attractiveResult[0]) * 1 * 3);
		for(int i = 0; i < 3; i++)
		{
			attractiveResult[i] = ( (attractiveForce[0][0]*jacobian[0][i]) + (attractiveForce[1][0]*jacobian[1][i]) );
		}
		

		//CHRIS: What is this line doing?
		for(int i=0;i<1;i++)
		{
			for(int j=0;j<3;j++)
			{
				totalAttractiveForce[j] +=attractiveResult[j];
			}
		}

		//CHRIS: ALso what is this? 
		for (int i = 0; i < 2; i++){

			for (int j = 0; j < 3; j++){
				totalJacobian[i][j] += jacobian[i][j];
			}
		}


		//BRIAN: NEED TO UPDATE THIS. IT LOOKS LIKE THE THETA VALUES IS BEING ADDED TO THE FORCES
		for(int i = 0; i < 3; i++)
		{
			totalRepulsiveForce[0] += (vertexRepulsiveForce[0]*jacobian[0][i]);
			totalRepulsiveForce[1] += (vertexRepulsiveForce[1]*jacobian[1][i]);
		}

	}

	double thetaScale = PI/64;
	double xyScale = .001;
	double repScale = .001;
	double thetaValue = ((totalAttractiveForce[2]/abs(totalAttractiveForce[2])) > 0) ? 
		move.m_dtheta = PI/64: move.m_dtheta = -PI/64;;

	/*
	if (totalRepulsiveForce[0]  + totalRepulsiveForce[1] < 
		totalAttractiveForce[0] + totalAttractiveForce[1]){
		repScale = 0.003;
		xyScale = 0.0009;
	}
	*/
	
	cout << "Att\t" << setprecision(4) << totalAttractiveForce[0] << "\t" << setprecision(4) << totalAttractiveForce[1] << "\t::\t" 
	 	 << "Rep\t" << setprecision(4) << totalRepulsiveForce[0]  << "\t" << setprecision(4) << totalRepulsiveForce[1]  << "\t"
		 << "repScale\t" << repScale << endl;

	move.m_dx = (-xyScale*totalAttractiveForce[0] + repScale*totalRepulsiveForce[0]);
	move.m_dy = (-xyScale*totalAttractiveForce[1] + repScale*totalRepulsiveForce[1]);

	// Go in slow mode
	Sleep(100);
	}
	return move;
}

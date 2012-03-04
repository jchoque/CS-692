#include "RigidBodyPlanner.hpp"
#include <vector>

using namespace std;

RigidBodyPlanner::RigidBodyPlanner(RigidBodySimulator * const simulator)
{
    m_simulator = simulator;   
}

RigidBodyPlanner::~RigidBodyPlanner(void)
{
    //do not delete m_simulator  
}


RigidBodyMove RigidBodyPlanner::ConfigurationMove(void)
    
{
    RigidBodyMove move;
	double robotX = m_simulator->GetRobotX();
	double robotY = m_simulator->GetRobotY();
	double robotTheta = m_simulator->GetRobotTheta();
	int numberVert = m_simulator->GetNrRobotVertices();
	vector<double[2][3]> robotJacobian;
	vector<double[2]> goalGradient;
	const double *robotVertices = m_simulator->GetRobotVertices();
	double jacoby[2][3];
	double gradient[2];
	double goalX = m_simulator->GetGoalCenterX();
	double goalY = m_simulator->GetGoalCenterY();

	// Calculate Jacobian for the center of the robot
	jacoby[0][0] = jacoby[1][1] = 1;
	jacoby[0][1] = jacoby[1][0] = 0;
	jacoby[0][2] = -1 * robotX * sin(robotTheta) - robotY * cos(robotTheta);
	jacoby[1][2] = robotX * cos(robotTheta) - robotY * sin(robotTheta);

	// Not sure if we'll need the center of the robots jacoby but I'll store it anyways
	robotJacobian.push_back(jacoby);

	// Calculate the gradient for the center of the robot
	// Maybe this is supposed to be FK instead of the simple calculation below
	gradient[0] = robotX - goalX;
	gradient[1] = robotY - goalY;
	goalGradient.push_back(gradient);

	// Loop through all vertices on this object and get calculate the Jacobian
	double vertX;
	double vertY;
	
	for (int idx = 0; idx < numberVert; idx++){
		vertX = robotVertices[idx * 2];
		vertY = robotVertices[idx * 2 + 1];

		//Calculate the Jacobian
		jacoby[0][2] = -1 * vertX * sin(robotTheta) - vertY * cos(robotTheta);
		jacoby[1][2] = vertX * cos(robotTheta) - vertY * sin(robotTheta);
		robotJacobian.push_back(jacoby);

		//Calculate the gradient to the goal
		gradient[0] = vertX - goalX;
		gradient[1] = vertY - goalY;
		goalGradient.push_back(gradient);
	}

	// 
	

    return move;
}



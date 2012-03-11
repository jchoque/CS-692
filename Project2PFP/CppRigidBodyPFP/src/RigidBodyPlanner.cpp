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
	
	for (int idx = 0; idx < numberVert; idx++){
		RobotJacobian aJacoby;
		vertX = robotVertices[idx * 2];
		vertY = robotVertices[idx * 2 + 1];

		//Calculate the Jacobian
		aJacoby.mJacobian[0][0] = aJacoby.mJacobian[1][1] =1;
		aJacoby.mJacobian[0][1] = aJacoby.mJacobian[1][0] =0;
		aJacoby.mJacobian[0][2] = -1 * vertX * sin(robotTheta) - vertY * cos(robotTheta);
		aJacoby.mJacobian[1][2] = vertX * cos(robotTheta) - vertY * sin(robotTheta);

		robotJacobian.push_back(aJacoby);

		//Calculate the gradient to the goal
		GoalGradient aGradient;
		aGradient.mGoalGradient[0] = vertX - goalX;
		aGradient.mGoalGradient[1] = vertY - goalY;
		goalGradient.push_back(aGradient);
	}

	// 
	

    return move;
}



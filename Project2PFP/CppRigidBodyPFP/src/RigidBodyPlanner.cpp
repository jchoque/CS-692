#include "RigidBodyPlanner.hpp"
#include <vector>

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
	double scale = 3;  // We need to figure out a good scaling
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

		GoalGradient aGradient;
		// Calculate the gradient using opposite value of attractive +
		// repulsive forces
		aGradient.mGoalGradient[0] = -1 * (uAtt[0] + uRep[0]);
		aGradient.mGoalGradient[1] = -1 * (uAtt[1] + uRep[1]);
		goalGradient.push_back(aGradient);
	}

	// We now have our Jacobian 2x3 Matrix and our Gradient 1x2 Matrix
	// Calculate the overall gradient and select movement
	for (int idx = 0; idx < 3; idx++){

	}
	

    return move;
}



#include "ManipPlanner.hpp"
#include "Windows.h"

ManipPlanner::ManipPlanner(ManipSimulator * const manipSimulator)
{
    m_manipSimulator = manipSimulator;   
}

ManipPlanner::~ManipPlanner(void)
{
    //do not delete m_simulator  
}

void ManipPlanner::ConfigurationMove(double allLinksDeltaTheta[])
{

	double thetaScale = M_PI/64;
	int numJoints = m_manipSimulator->GetNrLinks()-1;
   
	double fkX = m_manipSimulator->GetLinkEndX(numJoints) - m_manipSimulator->GetGoalCenterX();
	double fkY = m_manipSimulator->GetLinkEndY(numJoints)- m_manipSimulator->GetGoalCenterY();
	
	double aJaco[2][1];
	for(int i=0;i<numJoints;i++)
	{
		aJaco[0][0] = -m_manipSimulator->GetLinkEndY(numJoints) + -m_manipSimulator->GetLinkStartY(i);
		aJaco[1][0] = -m_manipSimulator->GetLinkEndX(numJoints) -  m_manipSimulator->GetLinkStartX(i);

		double jacoMultiply = (fkX *aJaco[0][0]) + (fkY * aJaco[1][0]);

		allLinksDeltaTheta[i] = ( jacoMultiply / (abs(jacoMultiply)) * thetaScale);
	}

	//Now multiply out
}


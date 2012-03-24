#include "ManipPlanner.hpp"
#include "Windows.h"
#include <vector>

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
	
	std::vector<Jacobian2x2>allJacos;
	double aJaco[2][1];
	for(int i=0;i<numJoints+1;i++)
	{
		Jacobian2x2 aJaco;
		//aJaco[0][0] = -m_manipSimulator->GetLinkEndY(numJoints) + m_manipSimulator->GetLinkStartY(i);
		//aJaco[1][0] = m_manipSimulator->GetLinkEndX(numJoints) -  m_manipSimulator->GetLinkStartX(i);

		aJaco.jacoX = -m_manipSimulator->GetLinkEndY(numJoints) + m_manipSimulator->GetLinkStartY(i);
		aJaco.jacoY = m_manipSimulator->GetLinkEndX(numJoints) -  m_manipSimulator->GetLinkStartX(i);

		double jacoMultiply = (fkX *aJaco.jacoX) + (fkY * aJaco.jacoY);

		//allLinksDeltaTheta[i] = -( jacoMultiply / (abs(jacoMultiply)) * thetaScale);
		allLinksDeltaTheta[i] = -jacoMultiply;
		allJacos.push_back(aJaco);
	}

	//Now compute repulsive forces

	for(int i=0;i<numJoints+1;i++)
	{
		//First "calculate" the FK on each joint
		double fkXi = m_manipSimulator->GetLinkEndX(i);
		double fkYi = m_manipSimulator->GetLinkEndY(i);
	
		double repPoX = 0;
		double repPoY = 0;
		for(int j=0;j<m_manipSimulator->GetNrObstacles(); j++)
		{
			Point closePoint = m_manipSimulator->ClosestPointOnObstacle(j,fkXi,fkYi);
			double obstacle_x = closePoint.m_x;
			double obstacle_y = closePoint.m_y;

			repPoX += obstacle_x - fkXi;
			repPoY += obstacle_y - fkYi;
		}

		//Jacobian2x2 aJaco = allJacos[i];
		Jacobian2x2 aJaco;
		aJaco.jacoX = -m_manipSimulator->GetLinkEndY(i) + m_manipSimulator->GetLinkStartY(0);
		aJaco.jacoY = m_manipSimulator->GetLinkEndX(i) -  m_manipSimulator->GetLinkStartX(0);		
		double jacoMultiply = (repPoX *aJaco.jacoX) + (repPoY * aJaco.jacoY);

	
		allLinksDeltaTheta[i] +=jacoMultiply;
		//allLinksDeltaTheta[i]-= (thetaScale)*(jacoMultiply/abs(jacoMultiply));
		if(allLinksDeltaTheta[i] != 0)
		{
			allLinksDeltaTheta[i] = allLinksDeltaTheta[i]/abs(allLinksDeltaTheta[i]) *thetaScale;
		} 
		
	}


	
	printf("Delta theta is: %4.3f\n", allLinksDeltaTheta[numJoints]);

	//Now multiply out
}


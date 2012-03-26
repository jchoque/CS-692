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
		Jacobian2x2 aJaco(0,0);
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

	for(int i=0;i<m_manipSimulator->GetNrObstacles();i++)
	{
		for(int j=0;j<m_manipSimulator->GetNrLinks(); j++)
		{
			
			Jacobian2x2 aJaco(0,0);
			for(int k=0;k<=j;k++)
			{
				aJaco.jacoX += (-m_manipSimulator->GetLinkEndY(j) + m_manipSimulator->GetLinkStartY(k));
				aJaco.jacoY += (m_manipSimulator->GetLinkEndX(j) -m_manipSimulator->GetLinkStartX(k));
			}

			double fkX = m_manipSimulator->GetLinkEndX(j);
			double fkY = m_manipSimulator->GetLinkEndY(j);

			Point closestPoint = m_manipSimulator->ClosestPointOnObstacle(i,fkX,fkY);

			double uRepX = closestPoint.m_x - fkX;
			double uRepY = closestPoint.m_y - fkY;

			double jacoMultiply = 2*(uRepX *aJaco.jacoX) + (uRepY * aJaco.jacoY);
			
			allLinksDeltaTheta[j] +=jacoMultiply;

	}


	}
	
	for(int j=0;j<m_manipSimulator->GetNrLinks();j++) 
	{
		if(allLinksDeltaTheta[j] != 0)
		{
			allLinksDeltaTheta[j] = allLinksDeltaTheta[j]/abs(allLinksDeltaTheta[j]) *thetaScale;
		}
	}



	
	printf("Delta theta is: %4.3f\n", allLinksDeltaTheta[numJoints]);

	//Now multiply out
}


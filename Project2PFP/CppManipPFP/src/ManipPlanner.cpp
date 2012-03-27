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
	
	for(int i=0;i<numJoints+1;i++)
	{
		Jacobian2x2 aJaco(0,0);

		aJaco.jacoX = -m_manipSimulator->GetLinkEndY(numJoints) + m_manipSimulator->GetLinkStartY(i);
		aJaco.jacoY = m_manipSimulator->GetLinkEndX(numJoints) -  m_manipSimulator->GetLinkStartX(i);

		double jacoMultiply = (fkX *aJaco.jacoX) + (fkY * aJaco.jacoY);

		//allLinksDeltaTheta[i] = -( jacoMultiply / (abs(jacoMultiply)) * thetaScale);
		allLinksDeltaTheta[i] = jacoMultiply;
		
	}

	//Now compute repulsive forces
	CalculateRepulsion(allLinksDeltaTheta);
	
	double mag = norm(allLinksDeltaTheta);

	for(int i=0;i<m_manipSimulator->GetNrLinks();i++)
	{
		allLinksDeltaTheta[i] = -0.01 * allLinksDeltaTheta[i] / mag;
		//allLinksDeltaTheta[i] =allLinksDeltaTheta[i] / mag;
	}

	printf("Delta theta is: %4.3f\n", allLinksDeltaTheta[numJoints]);

	//Now multiply out
}

 void ManipPlanner::CalculateRepulsion(double allLinksDeltaTheta[])
 {
	 for(int j=0;j<m_manipSimulator->GetNrLinks(); j++)
	 {
		for(int k=0;k<=j; k++)
		{
			Jacobian2x2 aJaco(0,0);
			aJaco.jacoX += (-m_manipSimulator->GetLinkEndY(j) + m_manipSimulator->GetLinkStartY(k));
			aJaco.jacoY += (m_manipSimulator->GetLinkEndX(j) -m_manipSimulator->GetLinkStartX(k));
			
			double fkX = m_manipSimulator->GetLinkEndX(j);
			double fkY = m_manipSimulator->GetLinkEndY(j);

			for(int i=0;i<m_manipSimulator->GetNrObstacles();i++)
			{
				Point closestPoint = m_manipSimulator->ClosestPointOnObstacle(i,fkX,fkY);
				double deltaX = closestPoint.m_x - fkX;
				double deltaY = closestPoint.m_y - fkY;
				double distance = sqrt(pow(deltaX,2) + pow(deltaY,2));
				double uRepX = deltaX/distance;
				double uRepY = deltaY/distance;
				
				double jacoMultiply = (uRepX *aJaco.jacoX) + (uRepY * aJaco.jacoY);
				allLinksDeltaTheta[j] -=jacoMultiply;
			}

		}
	}
 }

 double ManipPlanner::norm(double allLinksDeltaTheta[])
 {
		double sum = 0;

		for(int i=0;i<m_manipSimulator->GetNrLinks(); i++)
		{
			sum+=abs(allLinksDeltaTheta[i]);
		}

		return sum;

 }


#include "BugAlgorithms.hpp"
#include <iostream>

using namespace std;

BugAlgorithms::BugAlgorithms(Simulator * const simulator)
{
    m_simulator = simulator;   
    m_mode = STRAIGHT;
    m_hit[0] = m_hit[1] = HUGE_VAL;
    m_leave[0] = m_leave[1] = 0;
    m_distLeaveToGoal = HUGE_VAL;    
}

BugAlgorithms::~BugAlgorithms(void)
{
    //do not delete m_simulator  
}

Move BugAlgorithms::Bug0(Sensor sensor)
{
    Move move ={0,0};
	//add your implementation
    
    return move;
}

Move BugAlgorithms::Bug1(Sensor sensor)
{
    Move move ={0,0};
	//add your implementation
    
    return move;
}

Move BugAlgorithms::Bug2(Sensor sensor)
{
	Move move ={0,0};

	//Generate the mLine

	if(!m_simulator->HasRobotReachedGoal())
	{

		switch(m_mode)
		{
			case STRAIGHT:
				if(sensor.m_dmin <=m_simulator->GetWhenToTurn())
				{
					m_mode = AROUND;
					m_hit[0] = m_simulator->GetRobotCenterX();
					m_hit[1] = m_simulator->GetRobotCenterY();
					double dx = (sensor.m_xmin-m_simulator->GetRobotCenterX());
					double dy = (sensor.m_ymin-m_simulator->GetRobotCenterY());
					
					double magnitude = sqrt((dx*dx)+(dy*dy));
					dx/=magnitude;
					dy/=magnitude;
					dx*=m_simulator->GetStep();
					dy*=m_simulator->GetStep();
					move={-dy,dx};

				}
				else
				{
					double initX = m_simulator->GetRobotInitX();
					double initY = m_simulator->GetRobotInitY();
					
					double goalX = m_simulator->GetGoalCenterX();
					double goalY = m_simulator->GetGoalCenterY();
			
					double deltaX = goalX-initX;
					double deltaY = goalY-initY;
			
					double magnitude = sqrt( (deltaX*deltaX)+(deltaY*deltaY));
			
					deltaX /=magnitude;
					deltaY /=magnitude;
			
					deltaX*=m_simulator->GetStep();
					deltaY*=m_simulator->GetStep();
			
					move= {deltaX, deltaY};
				}
			break;
			case AROUND: 

					if(m_simulator->IsPointNearLine(
						m_simulator->GetRobotCenterX(), 
						m_simulator->GetRobotCenterY(), 
						m_simulator->GetRobotInitX(), 
						m_simulator->GetRobotInitY(), 
						m_simulator->GetGoalCenterX(), 
						m_simulator->GetGoalCenterY()))
					{
						m_mode = STRAIGHT;
						m_leave[0] = m_simulator->GetRobotCenterX();
						m_leave[1] = m_simulator->GetRobotCenterY();
						double initX = m_simulator->GetRobotInitX();
						double initY = m_simulator->GetRobotInitY();
						
						double goalX = m_simulator->GetGoalCenterX();
						double goalY = m_simulator->GetGoalCenterY();
				
						double deltaX = goalX-initX;
						double deltaY = goalY-initY;
				
						double magnitude = sqrt( (deltaX*deltaX)+(deltaY*deltaY));
				
						deltaX /=magnitude;
						deltaY /=magnitude;
				
						deltaX*=m_simulator->GetStep();
						deltaY*=m_simulator->GetStep();
				
						move= {deltaX, deltaY};
					}
					else
					{
						double dx = (sensor.m_xmin-m_simulator->GetRobotCenterX());
						double dy = (sensor.m_ymin-m_simulator->GetRobotCenterY());
					
						double magnitude = sqrt((dx*dx)+(dy*dy));
						dx/=magnitude;
						dy/=magnitude;
						dx*=m_simulator->GetStep();
						dy*=m_simulator->GetStep();
						move = {-dy,dx};
					}
			break;


		}

	}

	cout<<"Current state: "<<m_mode<<", currentMove["<<move.m_dx<<", "<<move.m_dy<<"]"<<endl;
    return move;
}

       



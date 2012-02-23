/*
 * CS 689
 * Brian Nolan
 * Chris O'Connell
 * Homework 1
 *
 */

#include "BugAlgorithms.hpp"
#include <iostream>
#include <math.h>

using namespace std;

BugAlgorithms::BugAlgorithms(Simulator * const simulator)
{
    m_simulator = simulator;   
    m_mode = STRAIGHT;
    m_hit[0] = m_hit[1] = HUGE_VAL;
    m_leave[0] = m_leave[1] = 0;
    m_distLeaveToGoal = HUGE_VAL;  
	m_steps = 0;
	m_stepsToLeave = 0;
}

BugAlgorithms::~BugAlgorithms(void)
{
    //do not delete m_simulator  
}

Move BugAlgorithms::Bug0(Sensor sensor)
{
	Move move ={0,0};

	// Determine my location and goal location
	double goalX = m_simulator->GetGoalCenterX();
	double goalY = m_simulator->GetGoalCenterY();
	double myX = m_simulator->GetRobotCenterX();
	double myY = m_simulator->GetRobotCenterY();

	// Debug calculation
	double goalSlope = (goalY - myY) / (goalX - myX);
	double goalAngle = (atan2(goalY - myY, goalX - myX) * 180 / 3.14159265);
	double distance = sqrt(pow(goalX - myX,2) + pow(goalY - myY, 2));

	// Get the direction to the goal
	double dirX = (goalX - myX) / distance;
	double dirY = (goalY - myY) / distance;
	double obstacleDistance = sqrt(pow(sensor.m_ymin - myY, 2) + pow(sensor.m_xmin - myX, 2));

	// Compute tangent if we are hitting the obstacle
	if (sensor.m_dmin <= m_simulator->GetWhenToTurn()){
		// Calculate path to the obstacle
		
		double obstacleDirX = (sensor.m_xmin - myX) / obstacleDistance;
		double obstacleDirY = (sensor.m_ymin - myY) / obstacleDistance * -1;
		double obstacleAngle = (atan2(sensor.m_ymin - myY, sensor.m_xmin - myX) * 180 / 3.14159265);

		double angleDiff = (int)(goalAngle - obstacleAngle) % 360;

		// Don't break away and go to the obstacle if the the angle is too close
		if ( abs(angleDiff) <= 90 ){
			dirX = obstacleDirY;
			dirY =  obstacleDirX;
		} 
		// The difference between the angle to the goal and obstacle is more than 90
		// so lets try going to the obstacle, but first take a step away from the obstacle
		else {
			// We want to take a step backwards so caclulate the direction as such
			obstacleDirX = myX - sensor.m_xmin;
			obstacleDirY = myY - sensor.m_ymin;
			obstacleDistance = sqrt(pow(obstacleDirY, 2) + pow(obstacleDirX, 2));
			// Normalize direction
			obstacleDirX = obstacleDirX / obstacleDistance;
			obstacleDirY = obstacleDirY / obstacleDistance;

			dirX = obstacleDirX;
			dirY = obstacleDirY;
		}
	}

	if (obstacleDistance >= (m_simulator->GetWhenToTurn() / 2)){
		move.m_dx = dirX * m_simulator->GetStep();
		move.m_dy = dirY * m_simulator->GetStep();
	}
    
    return move;
}

Move BugAlgorithms::Bug1(Sensor sensor)
{
	Move move ={0,0};
	
	// Determine my location and goal location
	double goalX = m_simulator->GetGoalCenterX();
	double goalY = m_simulator->GetGoalCenterY();
	double myX = m_simulator->GetRobotCenterX();
	double myY = m_simulator->GetRobotCenterY();

	// Get the direction to the goal
	double dirX = goalX - myX;
	double dirY = goalY - myY;
	double distance = sqrt(dirY * dirY + dirX * dirX);
	double obstacleDirX = sensor.m_xmin - myX;
	double obstacleDirY = sensor.m_ymin - myY;
	double obstacleDistance = sqrt(pow(obstacleDirY, 2) + pow(obstacleDirX, 2));
	double obstacleAngle = (atan2(obstacleDirY, obstacleDirX) * 180 / 3.14159265);
	obstacleDirX = obstacleDirX / obstacleDistance;
	obstacleDirY = obstacleDirY / obstacleDistance;
	obstacleDirY = -1 * obstacleDirY;

	// Normalize direction
	dirX = dirX / distance;
	dirY = dirY / distance;
	switch (m_mode) {
		case AROUND:
			dirX = obstacleDirY;
			dirY = obstacleDirX; 
			m_mode = AROUND_AND_AWAY_FROM_HIT_POINT;
			dirX = obstacleDirY;
			m_steps++;
		
		break;

		case AROUND_AND_AWAY_FROM_HIT_POINT:
			// Make sure we are near the hit point and our angle is far enough different
			if (m_simulator->ArePointsNear(myX, myY, m_hit[0], m_hit[1]) && m_steps > 20){ 
			//	cout << m_steps << " " << m_stepsToLeave << " " << (double)m_stepsToLeave / m_steps << endl;
				m_mode = AROUND_AND_TOWARD_LEAVE_POINT;
				dirX = obstacleDirY;
				dirY = obstacleDirX; 
				m_steps++;
			}
		break;

		case AROUND_AND_TOWARD_LEAVE_POINT:
			if (m_simulator->ArePointsNear(myX, myY, m_leave[0], m_leave[1])){
				m_mode = STRAIGHT_AND_AWAY_FROM_LEAVE_POINT;
				
				dirX = obstacleDirY;
				dirY = obstacleDirX;
				
			} 
		break;

		default:

		break;

	}

	// Compute tangent if necessary
	if ((sensor.m_dmin <= m_simulator->GetWhenToTurn() && m_mode != STRAIGHT_AND_AWAY_FROM_LEAVE_POINT) ||
		m_mode == AROUND_AND_TOWARD_LEAVE_POINT || m_mode == AROUND_AND_AWAY_FROM_HIT_POINT){
		m_steps++;
		if (m_mode == STRAIGHT && m_hit[0] != m_leave[0] && m_hit[1] != m_leave[1]){
			m_distLeaveToGoal = distance;
			m_hit[0] = myX;
			m_hit[1] = myY;
			m_leave[0] = myX;
			m_leave[1] = myY;
			m_mode = AROUND;
			m_steps = 0;
			m_stepsToLeave = 0;
		}
		if (m_mode != AROUND_AND_TOWARD_LEAVE_POINT
			&& m_distLeaveToGoal > distance){
			m_distLeaveToGoal = distance;
			m_leave[0] = myX;
			m_leave[1] = myY;
			m_stepsToLeave = m_steps;
		} else if (m_mode == AROUND_AND_TOWARD_LEAVE_POINT){
			m_steps--;
			
			if (((double)m_stepsToLeave / m_steps) < 0.5){
					dirX = obstacleDirY;
					dirY = obstacleDirX;
				} else {
					dirX = obstacleDirY * -1;
					dirY = obstacleDirX * -1;
				}
			}

		if (m_mode != AROUND_AND_TOWARD_LEAVE_POINT){
			dirX = obstacleDirY;
			dirY = obstacleDirX; 
		}
	} else {
		if (m_mode == STRAIGHT_AND_AWAY_FROM_LEAVE_POINT){
			m_mode = STRAIGHT;
			m_steps = 0;
			m_stepsToLeave = 0;
		}
	}

	move.m_dx = dirX * m_simulator->GetStep();
	move.m_dy = dirY * m_simulator->GetStep();
	

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
					move.m_dx = -dy;
					move.m_dy = dx;

				}
				else
				{
					double initX = m_simulator->GetRobotCenterX();
					double initY = m_simulator->GetRobotCenterY();
					
					double goalX = m_simulator->GetGoalCenterX();
					double goalY = m_simulator->GetGoalCenterY();
			
					double deltaX = goalX-initX;
					double deltaY = goalY-initY;
			
					double magnitude = sqrt( (deltaX*deltaX)+(deltaY*deltaY));
			
					deltaX /=magnitude;
					deltaY /=magnitude;
			
					deltaX*=m_simulator->GetStep();
					deltaY*=m_simulator->GetStep();
			
					move.m_dx = deltaX;
					move.m_dy = deltaY;

				}
			break;
			case AROUND: 
				// Add check for whether our angle to the goal is the same as the angle from our init point to the goal.
				// If not then check to see if the obstacle is in the way of our path to the goal.  If it is then continue
				// going around the obstacle, if it isn't then head to the goal.
				double obstacleAngle = (atan2(sensor.m_ymin - m_simulator->GetRobotCenterY(), sensor.m_xmin - m_simulator->GetRobotCenterX()) * 180 / 3.14159265);
				double goalAngle = (atan2(m_simulator->GetGoalCenterY() - m_simulator->GetRobotCenterY(), m_simulator->GetGoalCenterX() - m_simulator->GetRobotCenterX()) * 180 / 3.14159265);
				int angleDiff = abs((int)(goalAngle - obstacleAngle) % 360);
				
					if(m_simulator->IsPointNearLine(
						m_simulator->GetRobotCenterX(), 
						m_simulator->GetRobotCenterY(), 
						m_simulator->GetRobotInitX(), 
						m_simulator->GetRobotInitY(), 
						m_simulator->GetGoalCenterX(), 
						m_simulator->GetGoalCenterY()) &&
						angleDiff > 80 && // Only go toward goal if obstacle isn't in the way
						!m_simulator->ArePointsNear(m_leave[0], m_leave[1], m_simulator->GetRobotCenterX(), m_simulator->GetRobotCenterY()) // Only go to obstacle
						// if we didn't just leave from this leave point last time
						)
					{
						m_mode = STRAIGHT;
						m_leave[0] = m_simulator->GetRobotCenterX();
						m_leave[1] = m_simulator->GetRobotCenterY();
						double initX = m_simulator->GetRobotInitX();
						double initY = m_simulator->GetRobotInitY();
						
						double goalX = m_simulator->GetGoalCenterX();
						double goalY = m_simulator->GetGoalCenterY();
				
						double deltaX = goalX-m_simulator->GetRobotCenterX();
						double deltaY = goalY-m_simulator->GetRobotCenterY();
				
						double magnitude = sqrt( (deltaX*deltaX)+(deltaY*deltaY));
				
						deltaX /=magnitude;
						deltaY /=magnitude;
				
						deltaX*=m_simulator->GetStep();
						deltaY*=m_simulator->GetStep();
				
						move.m_dx = deltaX;
						move.m_dy = deltaY;
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
						
						move.m_dx = -dy;
						move.m_dy = dx;
					}
			break;


		}

	}


//	cout <<"Current state: "<<m_mode<<", currentMove["<<move.m_dx<<", "<<move.m_dy<<"]"<<endl;
    return move;
}

       



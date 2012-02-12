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
}

BugAlgorithms::~BugAlgorithms(void)
{
    //do not delete m_simulator  
}

Move BugAlgorithms::Bug0(Sensor sensor)
{
    bool DEBUG_MODE = false;
	Move move ={0,0};

	// Determine my location and goal location
	double goalX = m_simulator->GetGoalCenterX();
	double goalY = m_simulator->GetGoalCenterY();
	double myX = m_simulator->GetRobotCenterX();
	double myY = m_simulator->GetRobotCenterY();

	// Get the direction to the goal
	double directionX = goalX - myX;
	double directionY = goalY - myY;

	// Debug calculation
	double goalSlope = directionY / directionX;
	double goalAngle = (atan2(directionY, directionX) * 180 / 3.14159265);
	double distance = sqrt(pow(directionX,2) + pow(directionY, 2));

	// Normalize direction
	directionX = directionX / distance;
	directionY = directionY / distance;

	// Compute tangent if we are hitting the obstacle
	if (sensor.m_dmin <= m_simulator->GetWhenToTurn()){
		// Calculate path to the obstacle
		double arroundDirectionX = sensor.m_xmin - myX;
		double arroundDirectionY = sensor.m_ymin - myY;
		double arroundDistance = sqrt(pow(arroundDirectionY, 2) + pow(arroundDirectionX, 2));
		arroundDirectionX = arroundDirectionX / arroundDistance;
		arroundDirectionY = arroundDirectionY / arroundDistance;
		
		// Debug calculation
		double obstacleSlope = arroundDirectionY / arroundDirectionX;
		double obstacleAngle = (atan2(arroundDirectionY, arroundDirectionX) * 180 / 3.14159265);
		arroundDirectionY = -1 * arroundDirectionY;
		double obstacleTanAngle = (atan2(arroundDirectionY, arroundDirectionX) * 180 / 3.14159265);
		if (DEBUG_MODE){
			cout << "Goal: " << goalAngle << ", " << goalSlope << " Obstacle: " << obstacleAngle << ", " 
				 << obstacleSlope << " Angle Difference: " << abs(goalAngle - obstacleAngle) << endl;
		}

		double angleDiff = (int)(goalAngle - obstacleAngle) % 360;
		if (DEBUG_MODE){
			cout << "Angle Diff is " << angleDiff << endl;
		}

		// Don't break away and go to the obstacle if the the angle is too close
		if ( abs(angleDiff) <= 90 ){
			if (DEBUG_MODE){
				cout << "Taking the tangent" << endl;
			}
			directionX = arroundDirectionY;
			directionY =  arroundDirectionX;
		} 
		// The difference between the angle to the goal and obstacle is more than 90
		// so lets try going to the obstacle, but first take a step away from the obstacle
		else {
			if (DEBUG_MODE){
				cout << "Take a step away from the obstacle" << endl;
			}
			// We want to take a step backwards so caclulate the direction as such
			arroundDirectionX = myX - sensor.m_xmin;
			arroundDirectionY = myY - sensor.m_ymin;
			arroundDistance = sqrt(pow(arroundDirectionY, 2) + pow(arroundDirectionX, 2));
			// Normalize direction
			arroundDirectionX = arroundDirectionX / arroundDistance;
			arroundDirectionY = arroundDirectionY / arroundDistance;

			// Debug calculations
			obstacleSlope = arroundDirectionY / arroundDirectionX;
			obstacleAngle = (atan2(arroundDirectionY, arroundDirectionX) * 180 / 3.14159265);
			arroundDirectionY = arroundDirectionY;
			angleDiff = (int)(goalAngle - obstacleAngle) % 360;

			directionX = arroundDirectionX;
			directionY = arroundDirectionY;
			if (DEBUG_MODE){
				cout << "New slope: " << obstacleSlope << " angle " << obstacleAngle
					<< " angle diff " << angleDiff << endl;
			}
		}
	}
	else {
		if (DEBUG_MODE){
			cout << "Going to goal.  Angle: " << goalAngle << " slope: " << goalSlope << endl;
		}
	}

	move.m_dx = directionX * m_simulator->GetStep();
	move.m_dy = directionY * m_simulator->GetStep();
    
    return move;
}

Move BugAlgorithms::Bug1(Sensor sensor)
{
	//cout << "Running Bug 1" << endl;
    bool DEBUG_MODE = true;
	Move move ={0,0};
	

	// Determine my location and goal location
	double goalX = m_simulator->GetGoalCenterX();
	double goalY = m_simulator->GetGoalCenterY();
	double myX = m_simulator->GetRobotCenterX();
	double myY = m_simulator->GetRobotCenterY();

	// Get the direction to the goal
	double directionX = goalX - myX;
	double directionY = goalY - myY;
	double distance = sqrt(directionY * directionY + directionX * directionX);
	double arroundDirectionX = sensor.m_xmin - myX;
	double arroundDirectionY = sensor.m_ymin - myY;
	double arroundDistance = sqrt(pow(arroundDirectionY, 2) + pow(arroundDirectionX, 2));
	double obstacleAngle = (atan2(arroundDirectionY, arroundDirectionX) * 180 / 3.14159265);
	arroundDirectionX = arroundDirectionX / arroundDistance;
	arroundDirectionY = arroundDirectionY / arroundDistance;
	arroundDirectionY = -1 * arroundDirectionY;

	// Normalize direction
	directionX = directionX / distance;
	directionY = directionY / distance;
	switch (m_mode) {
		case AROUND:
			if (DEBUG_MODE){
				cout << "Changing from AROUND to Around and Away from Hit Point" << endl;
			}
			directionX = arroundDirectionY;
			directionY = arroundDirectionX; 
			m_mode = AROUND_AND_AWAY_FROM_HIT_POINT;
			directionX = arroundDirectionY;
		
		break;

		case AROUND_AND_AWAY_FROM_HIT_POINT:
			// Make sure we are near the hit point and our angle is far enough different
			if (m_simulator->ArePointsNear(myX, myY, m_hit[0], m_hit[1]) && m_steps > 20){ //abs(obstacleAngle - m_hitAngle) ){
				m_mode = AROUND_AND_TOWARD_LEAVE_POINT;
				directionX = arroundDirectionY;
				directionY = arroundDirectionX; 
				if (DEBUG_MODE){
					cout << "Changing to Around and toward Leave Point" << endl;
					cout << "Point distance " << (pow(myX - m_hit[0], 2) + pow(myY - m_hit[1],2)) << endl;
				}
			}
		break;

		case AROUND_AND_TOWARD_LEAVE_POINT:
			if (m_simulator->ArePointsNear(myX, myY, m_leave[0], m_leave[1])){
				m_mode = STRAIGHT_AND_AWAY_FROM_LEAVE_POINT;
				directionX = arroundDirectionY;
				directionY = arroundDirectionX; 
				if (DEBUG_MODE){
					cout << "Changing to Straight and Away From Leave Point" << endl;
					cout << "Point distance " << (pow(myX - m_leave[0], 2) + pow(myY - m_leave[1],2)) << endl;
				}
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
			m_hitAngle = obstacleAngle;
			m_mode = AROUND;
			m_steps = 0;
			if (DEBUG_MODE){
				cout << "Changing from Straight to Around, have leave point with distance " << m_distLeaveToGoal << endl;
			}
		}
		if (m_mode != AROUND_AND_TOWARD_LEAVE_POINT
			&& m_distLeaveToGoal > distance){
			cout << "Found a leave point with distance " << distance << endl;
			m_distLeaveToGoal = distance;
			m_leave[0] = myX;
			m_leave[1] = myY;
		} else {
			cout << "My distance is " << distance << " the saved is " << m_distLeaveToGoal << endl;
		}

		directionX = arroundDirectionY;
		directionY = arroundDirectionX; 
		
	} else {
		if (m_mode == STRAIGHT_AND_AWAY_FROM_LEAVE_POINT){
			m_mode = STRAIGHT;
			if (DEBUG_MODE){
				cout << "Changing to Straight" << endl;
			}
		}
	}
    
	move.m_dx = directionX * m_simulator->GetStep();
	move.m_dy = directionY * m_simulator->GetStep();

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

       



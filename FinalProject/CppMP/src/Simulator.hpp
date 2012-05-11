/**
 *@file Simulator.hpp
 */

#ifndef SIMULATOR_HPP_
#define SIMULATOR_HPP_

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio> 
#include <cstdlib>
#include <vector>
#include "PseudoRandom.hpp"

class Simulator
{
public:    
    Simulator(void);
    
    ~Simulator(void);

    enum
	{
	    STATE_X = 0,
	    STATE_Y = 1,
		STATE_ORIENTATION_IN_RADS =2,
		STATE_TRANS_VELOCITY = 3,
		STATE_STEERING_VELOCITY =4,
	    STATE_NR_DIMS = 5
	};

	static double MIN_VELOCITY;
	static double MAX_VELOCITY;
	static double MIN_ANGLE_VELOCITY;
	static double MAX_ANGLE_VELOCITY;
	static double MIN_ACCELERATION;
	static double MAX_ACCELERATION;
	static double MIN_ANGLE_ACCELERATION;
	static double MAX_ANGLE_ACCELERATION;

    void SetupFromFile(const char fname[]);

    double GetRobotCenterX(void) const
    {
	return m_circles[0];	
    }

    double GetRobotCenterY(void) const
    {
	return m_circles[1];	
    }

    double GetRobotRadius(void) const
    {
	return m_circles[2];
    }
    
    double GetGoalCenterX(void) const
    {
	return m_circles[3];	
    }

    double GetGoalCenterY(void) const
    {
	return m_circles[4];	
    }

    double GetGoalRadius(void) const
    {
	return m_circles[5];
    }

    int GetNrObstacles(void) const
    {
	return m_circles.size() / 3 - 2;
    }
    
    double GetObstacleCenterX(const int i) const
    {
	return m_circles[6 + 3 * i];
    }
    
    double GetObstacleCenterY(const int i) const
    {
	return m_circles[7 + 3 * i];
    }
    
    double GetObstacleRadius(const int i) const
    {
	return m_circles[8 + 3 * i];
    }
    
    double GetDistanceFromRobotCenterToGoal(void) const
    {
	const double rx = GetRobotCenterX();	
	const double ry = GetRobotCenterY();	
	const double gx = GetGoalCenterX();
	const double gy = GetGoalCenterY();
	
	return sqrt((rx - gx) * (rx - gx) + (ry - gy) * (ry - gy));
    }

    bool HasRobotReachedGoal(void) const
    {
	return GetDistanceFromRobotCenterToGoal() <= GetGoalRadius();
    }
	 
	double GetRobotTheta()
	{
		return m_state[STATE_ORIENTATION_IN_RADS];
	}

    void SetRobotState(const double s[])
    {
		SetRobotCenter(s[Simulator::STATE_X], s[Simulator::STATE_Y]);
		SetRobotTheta(s[Simulator::STATE_ORIENTATION_IN_RADS]);
	}
    
	void SetRobotTheta(double theta)
	{
		m_state[STATE_ORIENTATION_IN_RADS] = theta;
	}

    void SetRobotCenter(const double x, const double y)
    {
	m_circles[0] = x;
	m_circles[1] = y;
    }

	double getLastSampleX()
	{
		return 	m_samplePoint[0];
	}
	double getLastSampleY()
	{
		return m_samplePoint[1];
	
	}

	void setLastSample(double x, double y)
	{
		m_samplePoint[0] = x;
		m_samplePoint[1] = y;
	}
    
    bool IsValidState(void) const;
  
    double GetDistOneStep(void) const
    {
	return m_distOneStep;
    }

    void SampleState(double s[]) const
    {
		s[STATE_X] = PseudoRandomUniformReal(m_bbox[0], m_bbox[2]);
		s[STATE_Y] = PseudoRandomUniformReal(m_bbox[1], m_bbox[3]);
		s[STATE_ORIENTATION_IN_RADS] = PseudoRandomRadian();
		s[STATE_TRANS_VELOCITY] = PseudoRandomUniformReal(MIN_VELOCITY, MAX_VELOCITY);
		s[STATE_STEERING_VELOCITY] = PseudoRandomUniformReal(MIN_ANGLE_VELOCITY, MAX_ANGLE_VELOCITY);
    }
    

    const double* GetBoundingBox(void) const
    {
	return m_bbox;
    }
    

protected:    
    std::vector<double> m_circles;
	double  m_samplePoint[2];

    double              m_bbox[4];
	double              m_state[STATE_NR_DIMS];
    friend class Graphics;


private:
	    double  m_distOneStep;
};

#endif

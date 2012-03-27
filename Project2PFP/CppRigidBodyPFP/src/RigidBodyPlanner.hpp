#ifndef RIGID_BODY_PLANNER_HPP_
#define RIGID_BODY_PLANNER_HPP_

#include "RigidBodySimulator.hpp"
#define PI 3.14159
struct RigidBodyMove
{
    double m_dx;
    double m_dy;
    double m_dtheta;

	RigidBodyMove()
	{
		m_dx = 0;
		m_dy = 0;
		m_dtheta = 0;
	}
};

class RigidBodyPlanner
{
public:
    RigidBodyPlanner(RigidBodySimulator * const simulator);
            
    ~RigidBodyPlanner(void);

    /*
     * This is the function that you should implement.
     * This function needs to compute by how much the position (dx, dy) 
     * and orientation (dtheta) should change so that the robot makes a small 
     * move toward the goal while avoiding obstacles, 
     * as guided by the potential field.
     *
     * You have access to the simulator.
     * You can use the methods available in simulator to get all the information
     * you need to correctly implement this function
     *
     */
    RigidBodyMove ConfigurationMove(void);
    
protected:
    RigidBodySimulator *m_simulator;
	double normalizeAngle(double a);
	double modulus(double a, double b);
};

#endif

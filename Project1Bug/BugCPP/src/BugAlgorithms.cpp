#include "BugAlgorithms.hpp"

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
	//add your implementation
    
    return move;
}

       



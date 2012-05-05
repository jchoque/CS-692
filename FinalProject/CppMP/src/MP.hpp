#ifndef MOTION_PLANNER_HPP_
#define MOTION_PLANNER_HPP_

#include "Simulator.hpp"


struct Vertex
{
    enum
	{
	    TYPE_NONE = 0,
	    TYPE_INIT = 1,
	    TYPE_GOAL = 2
	};
	
    int    m_parent;
    double m_state[Simulator::STATE_NR_DIMS];
    int    m_type;
    int    m_nchildren;

	double * getXY()
	{
		double *xy = new double[2];
		xy[0] =m_state[0];
		xy[1] =m_state[1];

		return xy;
	}
    
};

    

class MotionPlanner
{
public:
    MotionPlanner(Simulator * const simulator);
            
    ~MotionPlanner(void);

    void ExtendRRT(void);
        
protected:

	double calculateWeight(int pVertex)
	{
		return 1.0/(1.0*pow((double)m_vertices[pVertex]->m_nchildren,2));
	}

	double calculateDistance(double sto[], double vertexState [])
	{
		return sqrt( pow(sto[Simulator::STATE_X]-vertexState[Simulator::STATE_X],2) + pow(sto[Simulator::STATE_Y]-vertexState[Simulator::STATE_Y],2)) +
			abs(sto[Simulator::STATE_ORIENTATION_IN_RADS]-vertexState[Simulator::STATE_ORIENTATION_IN_RADS]) +
			abs(sto[Simulator::STATE_STEERING_VELOCITY]-vertexState[Simulator::STATE_STEERING_VELOCITY]) +
			abs(sto[Simulator::STATE_TRANS_VELOCITY] -vertexState[Simulator::STATE_TRANS_VELOCITY]);

	}

	int getClosestVid(double sto [])
	{
		int currMinIdx = 0;
		double currMinDistance = calculateDistance(sto, m_vertices[0]->m_state);
		
		for(int i=0;i<m_vertices.size();i++)
		{
		
			double tempDistance = calculateDistance(sto, m_vertices[i]->m_state);

			if(tempDistance < currMinDistance)
			{
				currMinIdx = i;
				currMinDistance = tempDistance;

			}
		}
		return currMinIdx;
	}

	bool shouldPickRand;
	int failCount;

	int pickWeightedRandomIdx();
	
    bool IsProblemSolved(void)
    {
	return m_vidAtGoal >= 0;
    }

    void GetPathFromInitToGoal(std::vector<int> *path) const;

    void AddVertex(Vertex * const v);

    bool ExtendTree(const int    vid,
		    const double sto[]);
    
    Simulator            *m_simulator;
    std::vector<Vertex *> m_vertices;
    int                   m_vidAtGoal;
    double                m_totalSolveTime;

    
    friend class Graphics;    
};

#endif

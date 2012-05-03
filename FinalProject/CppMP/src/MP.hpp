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

	int getClosestVid(double sto [])
	{
		int currMinIdx = 0;
		double currMinDistance = sqrt( pow(sto[0]-m_vertices[0]->m_state[0],2) + pow(sto[1]-m_vertices[0]->m_state[1],2));
		for(int i=0;i<m_vertices.size();i++)
		{
			double vertexX = m_vertices[i]->m_state[0];
			double vertexY = m_vertices[i]->m_state[1];

			double tempDistance = sqrt( pow(sto[0]-vertexX,2) + pow(sto[1]-vertexY,2));

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

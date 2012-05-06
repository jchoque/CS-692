#include "MP.hpp"
#include "PseudoRandom.hpp"
#include "MyTimer.hpp"
#include <cstring>
#include <iostream>
#include <math.h>

using namespace std;


MotionPlanner::MotionPlanner(Simulator * const simulator)
{
	shouldPickRand = false;
	failCount = 0;
    m_simulator = simulator;   

    Vertex *vinit = new Vertex();

    vinit->m_parent   = -1;   
    vinit->m_nchildren= 0;    
    vinit->m_state[0] = m_simulator->GetRobotCenterX();
    vinit->m_state[1] = m_simulator->GetRobotCenterY();
	m_simulator->SetRobotTheta(0);

    AddVertex(vinit);
    m_vidAtGoal = -1;
    m_totalSolveTime = 0;
}

MotionPlanner::~MotionPlanner(void)
{
    //do not delete m_simulator  

    const int n = m_vertices.size();
    for(int i = 0; i < n; ++i)
	delete m_vertices[i];
}


bool MotionPlanner::ExtendTree(const int vid,double u, double v, double pSubGoal[])
{

	double currentLocX = m_simulator->GetRobotCenterX();
	double currentLocY = m_simulator->GetRobotCenterY();

	//This is our start position
	double stepSize = m_simulator->GetDistOneStep();

	bool inObstacle = false;

	double distance = 10000;
	double stepVertex[Simulator::STATE_NR_DIMS];
	stepVertex[Simulator::STATE_X] = m_vertices[vid]->m_state[Simulator::STATE_X];
	stepVertex[Simulator::STATE_Y] = m_vertices[vid]->m_state[Simulator::STATE_Y];
	stepVertex[Simulator::STATE_ORIENTATION_IN_RADS] = m_vertices[vid]->m_state[Simulator::STATE_ORIENTATION_IN_RADS];
	stepVertex[Simulator::STATE_TRANS_VELOCITY] = m_vertices[vid]->m_state[Simulator::STATE_TRANS_VELOCITY];
	stepVertex[Simulator::STATE_STEERING_VELOCITY] = m_vertices[vid]->m_state[Simulator::STATE_STEERING_VELOCITY];
	int parent = vid;
	int iters =0;
	while (!inObstacle && distance >=stepSize && iters<100)
	{
		iters++;
		double deltaX = stepSize* stepVertex[Simulator::STATE_TRANS_VELOCITY]*cos(stepVertex[Simulator::STATE_ORIENTATION_IN_RADS]);
		double deltaY = stepSize*stepVertex[Simulator::STATE_TRANS_VELOCITY]*sin(stepVertex[Simulator::STATE_ORIENTATION_IN_RADS]);
		double deltatheta = stepSize*(stepVertex[Simulator::STATE_TRANS_VELOCITY]/m_simulator->GetRobotRadius()) * tan(stepVertex[Simulator::STATE_STEERING_VELOCITY]);
		double deltaSpeed = stepSize * u;
		double deltaAngleVel = stepSize*v;
		double testState[Simulator::STATE_NR_DIMS];
		stepVertex[Simulator::STATE_X]+=deltaX;
		stepVertex[Simulator::STATE_Y]+=deltaY;
		stepVertex[Simulator::STATE_ORIENTATION_IN_RADS]+=deltatheta;
		stepVertex[Simulator::STATE_TRANS_VELOCITY]+=deltaSpeed;
		stepVertex[Simulator::STATE_STEERING_VELOCITY]+=deltaAngleVel;

		// Set the robot location so we can determine if this is a valid state
		m_simulator->SetRobotState(stepVertex);
		
		if (m_simulator->IsValidState()){
			
			distance = calculateDistance(stepVertex, pSubGoal);

			if (m_simulator->HasRobotReachedGoal())
			{
				m_vidAtGoal = vid;
				break;
			}
			Vertex *vertex = new Vertex();
			for(int i=0;i<Simulator::STATE_NR_DIMS;i++)
			{
				vertex->m_state[i] = stepVertex[i];
			}
			vertex->m_parent = parent;
			AddVertex(vertex);
			parent = m_vertices.size()-1;
			currentLocX = stepVertex[Simulator::STATE_X];
			currentLocY = stepVertex[Simulator::STATE_Y];
		}
		else {
			inObstacle = true;
			// Reset the robot to be at the previous vertex
			m_simulator->SetRobotCenter(currentLocX, currentLocY);
		}
	}
	//Did we hit an obstacle? If not then we added it sucessfully. 
	return !inObstacle;
}

void MotionPlanner::ExtendRRT(void)
{
    Clock clk;
    StartTime(&clk);

	//1. Sample state
	double * sampleState = new double[Simulator::STATE_NR_DIMS];
	m_simulator->SampleState(sampleState);
	
	//2. Check to see if the state is valid
	m_simulator->SetRobotCenter(sampleState[Simulator::STATE_X], sampleState[Simulator::STATE_Y]);

	if(m_simulator->IsValidState())
	{
		//3. Find the nearest configuration based on distance.
		int vid = getClosestVid(sampleState);

		double tempObj[Simulator::STATE_NR_DIMS];
		double deltat = .1;

		//Find the best out of many controls
		double bestDistance = DBL_MAX;
		double bestControlU = -1;
		double bestControlV = -1;
		for(int i=0;i<10;i++)
		{
			double u = PseudoRandomUniformReal(Simulator::MIN_VELOCITY,Simulator::MAX_VELOCITY);
			double v = PseudoRandomUniformReal(Simulator::MIN_ANGLE_VELOCITY, Simulator::MAX_ANGLE_VELOCITY);

			for(int i=0;i<Simulator::STATE_NR_DIMS; i++)
			{
				tempObj[i] = m_vertices[vid]->m_state[i];
			}

			for(double i=0;i<5;i++)	
			{

				double deltaX = deltat* tempObj[Simulator::STATE_TRANS_VELOCITY]*cos(tempObj[Simulator::STATE_ORIENTATION_IN_RADS]);
				double deltaY = deltat*tempObj[Simulator::STATE_TRANS_VELOCITY]*sin(tempObj[Simulator::STATE_ORIENTATION_IN_RADS]);
				double deltatheta = deltat*(tempObj[Simulator::STATE_TRANS_VELOCITY]/m_simulator->GetRobotRadius()) * tan(tempObj[Simulator::STATE_STEERING_VELOCITY]);
				double deltaSpeed = deltat * u;
				double deltaAngleVel = deltat*v;
				double testState[Simulator::STATE_NR_DIMS];
				testState[Simulator::STATE_X] = tempObj[Simulator::STATE_X]+deltaX;
				testState[Simulator::STATE_Y] = tempObj[Simulator::STATE_Y]+deltaY;
				testState[Simulator::STATE_ORIENTATION_IN_RADS] = tempObj[Simulator::STATE_ORIENTATION_IN_RADS]+deltatheta;
				testState[Simulator::STATE_TRANS_VELOCITY] = tempObj[Simulator::STATE_TRANS_VELOCITY]+deltaSpeed;
				testState[Simulator::STATE_STEERING_VELOCITY] = tempObj[Simulator::STATE_STEERING_VELOCITY]+deltaAngleVel;

				m_simulator->SetRobotState(testState);
				if(!m_simulator->IsValidState())
				{
					break;
				}
				else
				{
					for(int j=0;j<Simulator::STATE_NR_DIMS; j++)
					{
						tempObj[j] = testState[j];
					}
				}
			}

			double tempDistance = calculateDistance(tempObj,sampleState);
			if(tempDistance<bestDistance)
			{
				bestControlU = u;
				bestControlV = v;
				bestDistance = tempDistance;
			}
		}

		ExtendTree(vid, bestControlU, bestControlV, sampleState);
	
		//4. Create a local trajectoy based on the closest configuration and the sampled configuration. 

		//5. If the sub trajectory from [0, step] is valid, add the trajectory from [0, step] 
			//Finally check to see if we have found an answer
	}
	m_totalSolveTime += ElapsedTime(&clk);    
}


void MotionPlanner::AddVertex(Vertex * const v)
{
    if(v->m_type == Vertex::TYPE_GOAL)
	m_vidAtGoal = m_vertices.size();
    m_vertices.push_back(v); 
    if(v->m_parent >= 0)
	(++m_vertices[v->m_parent]->m_nchildren);
}

void MotionPlanner::GetPathFromInitToGoal(std::vector<int> *path) const
{

	
    std::vector<int> rpath;
    
    rpath.clear();
    
    int i = m_vidAtGoal;
    do
    {
	rpath.push_back(i);
	i = m_vertices[i]->m_parent;	
    } 
    while(i >= 0);
   
    path->clear();
    for(int i = rpath.size() - 1; i >= 0; --i)
	path->push_back(rpath[i]);
}


int MotionPlanner::pickWeightedRandomIdx()
{
	// Find the maximum children a vertex could have
	double totalWeight = 0;
	
	for (unsigned int i = 0; i < m_vertices.size(); i++){
		//relative weight
		totalWeight += calculateWeight(i);
	}

	// Generate a random number based on the totalWeight and then
	// pick the vertex that matches that weight
	double weightPicked = PseudoRandomUniformReal(0,totalWeight); 

	int vid = 0;  // The vector index selected
	
	//Reinit the total weight
	totalWeight = 0;
	for (unsigned int i = 0; i < m_vertices.size(); i++){ 
		totalWeight += calculateWeight(i);  // Add it to the previous vector weights
		if (totalWeight >= weightPicked){
			vid = i;
			break;
		}
	}

	return vid;
}
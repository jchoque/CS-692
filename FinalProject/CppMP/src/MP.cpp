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


bool MotionPlanner::ExtendTree(const int    vid, 
			       const double sto[])
{

	//This is our start position
	double vertexX = m_vertices[vid]->m_state[0];
	double vertexY = m_vertices[vid]->m_state[1];	
	double stepSize = m_simulator->GetDistOneStep();
	double distance = sqrt(pow(vertexX - sto[0], 2) + pow(vertexY - sto[1], 2));
	bool inObstacle = false;
	
	double deltaX = (sto[0]-vertexX)/distance;
	double deltaY = (sto[1]-vertexY)/distance;
	// While we don't hit an obstacle and our distance is greater than 0
	// walk down this vertex path to see if there are any obstacles or the 
	// goal
	double nextX=vertexX;
	double nextY=vertexY;
	while (!inObstacle && distance >=stepSize){
		
		//This will first "normalize" the vector and will increase it by a single step
		nextX += (deltaX * stepSize);
		nextY += (deltaY * stepSize);
		
		// Set the robot location so we can determine if this is a valid state
		m_simulator->SetRobotCenter(nextX, nextY);
		
		if (m_simulator->IsValidState()){
			distance = sqrt(pow(nextX - sto[0], 2) + pow(nextY - sto[1], 2));
			if (m_simulator->HasRobotReachedGoal()){
				m_vidAtGoal = vid;
				break;
			}
		}
		else {
			inObstacle = true;
			// Reset the robot to be at the previous vertex
			m_simulator->SetRobotCenter(vertexX, vertexY);
		}
	}

	// If we are not in an obstacle then we have found a valid vertex
	// so add it to our list
	if (!inObstacle) {
		Vertex *vertex = new Vertex();
		vertex->m_state[0] = nextX;
		vertex->m_state[1] = nextY;
		vertex->m_parent = vid;
		AddVertex(vertex);
		m_simulator->SetRobotCenter(nextX, nextY);
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

		int vid = getClosestVid(sampleState);
		//3. Find the nearest configuration based on distance.
			//TODO: Need to find out what a good distance metric is. Do we use Euclidean, or is there some other way to check?

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
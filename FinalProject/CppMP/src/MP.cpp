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

	//For some reason this is getting set to 1, even though I set it to something else..weird
	//double stepSize = m_simulator->GetDistOneStep();
	double stepSize = 0.1;
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
	while (!inObstacle && distance >=stepSize && iters<1000)
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
		stepVertex[Simulator::STATE_ORIENTATION_IN_RADS] = clampAngle(stepVertex[Simulator::STATE_ORIENTATION_IN_RADS]+deltatheta);
		stepVertex[Simulator::STATE_TRANS_VELOCITY] = clampValue(stepVertex[Simulator::STATE_TRANS_VELOCITY]+deltaSpeed, Simulator::MIN_VELOCITY, Simulator::MAX_VELOCITY);
		stepVertex[Simulator::STATE_STEERING_VELOCITY] = clampValue(stepVertex[Simulator::STATE_STEERING_VELOCITY]+deltaAngleVel,Simulator::MIN_ANGLE_VELOCITY, Simulator::MAX_ANGLE_VELOCITY);

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
	double tooClose =  m_simulator->GetRobotRadius() * 2;
	double tooFar =  tooClose * 10;
	//1. Sample state
	// Since sampling takes very little time we want to only sample vertices that are:
	// a)  Unique points that are not very close to existing vertices because they
	//     most likely won't provide any new paths
	// b)  Points that are not very far from all other vertices because the 
	//     probability of having to maneuver around multiple obstacles increases
	//     and causes the computational time to increase
	double * sampleState = new double[Simulator::STATE_NR_DIMS];
	bool newPoint;
	do {
		newPoint = true;
		m_simulator->SampleState(sampleState);

		// Check all known vertices to see if this point is relatively close to an existing vertex
		// or if the vertex is way too far away don't even try to add it
		double minDist = DBL_MAX;
		for (int idx = 0; idx < m_vertices.size() && newPoint; idx++){
			double dist = sqrt(pow(sampleState[Simulator::STATE_X] - m_vertices[idx]->m_state[Simulator::STATE_X], 2) + 
				pow(sampleState[Simulator::STATE_Y] - m_vertices[idx]->m_state[Simulator::STATE_Y], 2));
			if (dist < tooClose){
				newPoint = false;
				break;
			}
			// Store the minimum distance to make sure this point isn't too far
			// from all of the other points
			else if (minDist > dist){
				minDist = dist;
			}
			
		}
		// If this is a new point then make sure it isn't too far from all other points
		if (newPoint && minDist > tooFar){
			newPoint = false;
		}
	} while (!newPoint);  // While we haven't found a new point
	
	//2. Check to see if the state is valid
	m_simulator->SetRobotCenter(sampleState[Simulator::STATE_X], sampleState[Simulator::STATE_Y]);

	if(m_simulator->IsValidState())
	{
		//3. Find the nearest configuration based on distance.
		int vid = getClosestVid(sampleState);

		double tempObj[Simulator::STATE_NR_DIMS];
		//double deltat = m_simulator->GetDistOneStep();
		double deltat = 0.1;
		//Find the best out of many controls
		double bestDistance = DBL_MAX;
		double bestControlU = -1;
		double bestControlV = -1;
		for(int i=0;i<10;i++)
		{
			double u = PseudoRandomUniformReal(Simulator::MIN_ACCELERATION,Simulator::MAX_ACCELERATION);
			double v = PseudoRandomUniformReal(Simulator::MIN_ANGLE_ACCELERATION, Simulator::MAX_ANGLE_ACCELERATION);

			for(int i=0;i<Simulator::STATE_NR_DIMS; i++)
			{
				tempObj[i] = m_vertices[vid]->m_state[i];
			}

			for(int j=0;j<5;j++)	
			{

				double deltaX = deltat* tempObj[Simulator::STATE_TRANS_VELOCITY]*cos(tempObj[Simulator::STATE_ORIENTATION_IN_RADS]);
				double deltaY = deltat*tempObj[Simulator::STATE_TRANS_VELOCITY]*sin(tempObj[Simulator::STATE_ORIENTATION_IN_RADS]);
				double deltatheta = deltat*(tempObj[Simulator::STATE_TRANS_VELOCITY]/m_simulator->GetRobotRadius()) * tan(tempObj[Simulator::STATE_STEERING_VELOCITY]);
				double deltaSpeed = deltat * u;
				double deltaAngleVel = deltat*v;

				double lastGoodState[Simulator::STATE_NR_DIMS];
				//Save last good state, in case it fails
				memcpy(lastGoodState,tempObj,Simulator::STATE_NR_DIMS);
				for(int k=0;k<Simulator::STATE_NR_DIMS;k++)
				{
					lastGoodState[k] = tempObj[k];
				}
				
				tempObj[Simulator::STATE_X]+=deltaX;
				tempObj[Simulator::STATE_Y]+=deltaY;
				tempObj[Simulator::STATE_ORIENTATION_IN_RADS] = clampAngle(tempObj[Simulator::STATE_ORIENTATION_IN_RADS]+deltatheta);
				tempObj[Simulator::STATE_TRANS_VELOCITY] = clampValue(tempObj[Simulator::STATE_TRANS_VELOCITY]+deltaSpeed,Simulator::MIN_VELOCITY, Simulator::MAX_VELOCITY);
				tempObj[Simulator::STATE_STEERING_VELOCITY] = clampValue(tempObj[Simulator::STATE_STEERING_VELOCITY]+deltaAngleVel, Simulator::MIN_ANGLE_VELOCITY, Simulator::MAX_ANGLE_VELOCITY);

				m_simulator->SetRobotState(tempObj);
				if(!m_simulator->IsValidState())
				{
					memcpy(tempObj,lastGoodState,Simulator::STATE_NR_DIMS);
					break;
				}
			}

			if(m_simulator->IsValidState())
			{
				double tempDistance = calculateDistance(tempObj,sampleState);
				if(tempDistance<bestDistance)
				{
					bestControlU = u;
					bestControlV = v;
					bestDistance = tempDistance;
				}
			}

			ExtendTree(vid, bestControlU, bestControlV, sampleState);
		}
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
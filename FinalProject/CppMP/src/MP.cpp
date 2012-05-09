#include "MP.hpp"
#include "PseudoRandom.hpp"
#include "MyTimer.hpp"
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <math.h>
#include <stdio.h>
#include <unordered_map>

using namespace std;


MotionPlanner::MotionPlanner(Simulator * const simulator)
{
	shouldPickRand = false;
	failCount = 0;
    m_simulator = simulator;   
	
	double oldX = m_simulator->GetRobotCenterX();
	double oldY = m_simulator->GetRobotCenterY();
    Vertex *vinit = new Vertex();

    vinit->m_parent   = -1;   
    vinit->m_nchildren= 0;    
    vinit->m_state[0] = m_simulator->GetRobotCenterX();
    vinit->m_state[1] = m_simulator->GetRobotCenterY();
	for(int i=2;i<Simulator::STATE_NR_DIMS;i++)
	{
		vinit->m_state[i] =0;
	}
	m_simulator->SetRobotTheta(0);
	generateReachableState(0,vinit);

    AddVertex(vinit);
    m_vidAtGoal = -1;
    m_totalSolveTime = 0;

	m_simulator->SetRobotCenter(oldX, oldY);
}

MotionPlanner::~MotionPlanner(void)
{
    //do not delete m_simulator  

    const int n = m_vertices.size();
    for(int i = 0; i < n; ++i)
	delete m_vertices[i];
}


void MotionPlanner::ExtendTree(const int vid,double u, double v, double pSubGoal[],int pMode)
{

	double currentLocX = m_simulator->GetRobotCenterX();
	double currentLocY = m_simulator->GetRobotCenterY();

	//This is our start position

	//For some reason this is getting set to 1, even though I set it to something else..weird
	//double stepSize = m_simulator->GetDistOneStep();
	double stepSize = 0.1;
	bool inObstacle = false;
	Vertex *vertex;
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
			vertex = new Vertex();
			for(int i=0;i<Simulator::STATE_NR_DIMS;i++)
			{
				vertex->m_state[i] = stepVertex[i];
			}

			vertex->m_parent = parent;
			AddVertex(vertex);
			parent = m_vertices.size()-1;

			if(pMode == REACH_RRT)
			{
				generateReachableState(parent,vertex);
				
			}
			currentLocX = stepVertex[Simulator::STATE_X];
			currentLocY = stepVertex[Simulator::STATE_Y];
			
			// Store this point in memory so we don't pick another point close 
			// to it to build off of
			int pointX = (int)vertex->m_state[Simulator::STATE_X];
			int pointY = (int)vertex->m_state[Simulator::STATE_Y];
			map <int,int> points;
			unordered_map<int, map<int,int>>::const_iterator iter = m_endPoints.find(pointX);
			if (iter != m_endPoints.end()){
				points = iter->second;
			}
			std::pair<int, int> point (pointY, 0);
			points.insert(point);
			std::pair<int, map<int, int>> data (pointX, points);
			m_endPoints.insert(data);		
		}
		else 
		{
			inObstacle = true;
			// Reset the robot to be at the previous vertex
			m_simulator->SetRobotCenter(currentLocX, currentLocY);
		}
	}
	
}

void MotionPlanner::ExtendRRT(void)
{
    Clock clk;
    StartTime(&clk);
	//1. Sample state
	// Since sampling takes very little time we want to only sample vertices that are
	// unique points that are not very close to existing vertices because they
	// most likely won't provide any new paths
	double * sampleState = new double[Simulator::STATE_NR_DIMS];
	bool validState;
//	cout << "Generating a new point" << endl;
	do {
		validState = true;
		m_simulator->SampleState(sampleState);
	
		int pointX = (int)sampleState[Simulator::STATE_X];
		int pointY = (int)sampleState[Simulator::STATE_Y];
	
		unordered_map<int, map<int,int>>::const_iterator iter = m_endPoints.find(pointX);//pointX.str());
		if (iter != m_endPoints.end()){
			if (iter->second.find(pointY) != iter->second.end()){
				cout << "Skipping point " << pointX << "," << pointY << endl;
				validState = false;
				break;
			}
		}
		
	} while (!validState);
//	cout << "Found a valid point!" << endl;

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

void MotionPlanner::ExtendRG_RRT(void)
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

		//Now see if vid's children is closer
		
		int childrenIdx = -1;
		double minDistance = calculateDistance(sampleState,m_vertices[vid]->m_state);
		std::vector<ReachableObj *>children = m_vertices.at(vid)->mReachableObj;

		for(int i=0; i<children.size();i++)
		{
			double tempDistance =calculateDistance(sampleState, children[i]->m_state); 
			if(tempDistance< minDistance)
			{
				minDistance = tempDistance;
				childrenIdx = i;

			}

		}
		
		if(childrenIdx != -1)
		{

			double u = children[childrenIdx]->u;
			double v = children[childrenIdx]->v;
			ExtendTree(vid,u,v,sampleState,REACH_RRT);
		}
	}
	
	m_totalSolveTime += ElapsedTime(&clk);
}
void MotionPlanner::generateReachableState(int pParentIdx, Vertex *pParentVertex)
{
	double tempObj[Simulator::STATE_NR_DIMS];
	double oldValues[Simulator::STATE_NR_DIMS];
	double deltat = .1;

	bool addIt = false;
	for(int k=0;k<5;k++)
	{

		for(int i=0;i<Simulator::STATE_NR_DIMS;i++)
		{
			tempObj[i] = pParentVertex->m_state[i];
		}
		double u = PseudoRandomUniformReal(Simulator::MIN_ACCELERATION, Simulator::MAX_ANGLE_ACCELERATION);
		double v = PseudoRandomUniformReal(Simulator::MIN_ANGLE_ACCELERATION, Simulator::MAX_ANGLE_ACCELERATION);
	
		for(int iter=0;iter<1000;iter++)
		{
			double deltaX = deltat* tempObj[Simulator::STATE_TRANS_VELOCITY]*cos(tempObj[Simulator::STATE_ORIENTATION_IN_RADS]);
			double deltaY = deltat*tempObj[Simulator::STATE_TRANS_VELOCITY]*sin(tempObj[Simulator::STATE_ORIENTATION_IN_RADS]);
			double deltatheta = deltat*(tempObj[Simulator::STATE_TRANS_VELOCITY]/m_simulator->GetRobotRadius()) * tan(tempObj[Simulator::STATE_STEERING_VELOCITY]);
			double deltaSpeed = deltat * u;
			double deltaAngleVel = deltat*v;

			for(int i=0;i<Simulator::STATE_NR_DIMS;i++)
			{
				oldValues[i] = tempObj[i];
			}

			tempObj[Simulator::STATE_X]+=deltaX;
			tempObj[Simulator::STATE_Y]+=deltaY;
			tempObj[Simulator::STATE_ORIENTATION_IN_RADS]=clampAngle(tempObj[Simulator::STATE_ORIENTATION_IN_RADS]+deltatheta);
			tempObj[Simulator::STATE_TRANS_VELOCITY] = clampValue(tempObj[Simulator::STATE_TRANS_VELOCITY]+deltaSpeed,Simulator::MIN_VELOCITY, Simulator::MAX_VELOCITY);
			tempObj[Simulator::STATE_STEERING_VELOCITY] = clampValue(tempObj[Simulator::STATE_STEERING_VELOCITY]+deltaAngleVel,Simulator::MIN_ANGLE_VELOCITY, Simulator::MAX_ANGLE_ACCELERATION);
		
			m_simulator->SetRobotState(tempObj);
			if(m_simulator->IsValidState())
			{
				addIt = true;
			}
			else
			{
				for(int i=0;i<Simulator::STATE_NR_DIMS;i++)
				{
					tempObj[i] = oldValues[i];
				}

				break;
			}
		}
	
		if(addIt)
		{
			ReachableObj *anObj = new ReachableObj();
			for(int i=0;i<Simulator::STATE_NR_DIMS;i++)
			{
				anObj->m_state[i] = tempObj[i];
			}
			anObj->u = u;
			anObj->v = v;
			anObj->m_parent = pParentIdx;
			pParentVertex->mReachableObj.push_back(anObj);
		}
	}
}
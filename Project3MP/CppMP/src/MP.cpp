#include "MP.hpp"
#include "PseudoRandom.hpp"
#include "MyTimer.hpp"
#include <cstring>
#include <iostream>
#include <math.h>

using namespace std;


MotionPlanner::MotionPlanner(Simulator * const simulator)
{
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


void MotionPlanner::ExtendTree(const int    vid, 
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
}

void MotionPlanner::ExtendRandom(void)
{
    Clock clk;
    StartTime(&clk);
	double sto[2];

	//Uniformly pick an index
	int vid = (int)PseudoRandomUniformReal(0,m_vertices.size()-1);
	
	// Extend the tree out to this point if there are no obstacles
	ExtendTree(vid, sto);
    
    m_totalSolveTime += ElapsedTime(&clk);
}

void MotionPlanner::ExtendRRT(void)
{
    Clock clk;
    StartTime(&clk);

	//The random sample of a new state
	double sto[2];

	//Uniformly sample a state
	m_simulator->SampleState(sto);

	//Now find the vertex that is closest to the goal
	int currMinIdx = 0;
	double currMinDistance = sqrt( pow(sto[0]-m_vertices[0]->m_state[0],2) + pow(sto[1]-m_vertices[0]->m_state[1],2));
	for(int i=1;i<m_vertices.size();i++)
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

	//Now that we've found the min distance, extend the vertex
	ExtendTree(currMinIdx,sto);
    
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendEST(void)
{
    Clock clk;
    StartTime(&clk);
	
	// Get our next state to check
	double sto[2];
	m_simulator->SampleState(sto);

	// Find the maximum children a vertex could have
	int maxWeight = 0;
	for (unsigned int i = 1; i < m_vertices.size(); i++){
		if (m_vertices[i]->m_nchildren > maxWeight){
			maxWeight = m_vertices[i]->m_nchildren;
		}
	}
	// Increase the chances of picking a vertex with the most children
	maxWeight *= 2;

	// If we have multiple vertices with the same number of children(weight)
	// then store their index and choose one of them randomly later
	vector<int> matchingVertices;

	// Generate a random number based on the maxWeight and then
	// pick the vertex that matches that weight
	int weightPicked = ceil(PseudoRandomUniformReal(0,maxWeight) / 2);

	int vid = 0;
	for (unsigned int i = 1; i < m_vertices.size(); i++){
		if (m_vertices[i]->m_nchildren == weightPicked){
			matchingVertices.push_back(i);
			vid = i;
		}
		else if (m_vertices[i]->m_nchildren < weightPicked) {
			if (matchingVertices.size() == 0 ){
				vid = i;
				matchingVertices.push_back(i);
			} else if (m_vertices[matchingVertices[0]]->m_nchildren < 
				m_vertices[i]->m_nchildren) {
				matchingVertices.clear();
				vid = i;
				matchingVertices.push_back(i);
			} else if (m_vertices[matchingVertices[0]]->m_nchildren < 
				m_vertices[i]->m_nchildren) {
				matchingVertices.push_back(i);
			}
		}
	}

	// If we found multiple vertices that had the same matching weight then pick 
	// one of them at random
	if (matchingVertices.size() > 1){
		int randomPick = (int)PseudoRandomUniformReal(0,matchingVertices.size() - 1);
		vid = matchingVertices[randomPick];
	}
	ExtendTree(vid, sto);
	//Sleep (200);
    m_totalSolveTime += ElapsedTime(&clk);
}



//Chris's Approach: Idea is to do the following:
//1. Uniformly pick an obstacle
//2. Uniformly pick an angle around the chosen obstacle
//3. Uniformly pick a distance from the obstacle, in the bounds [radius+robotRadius, radius+2*robotRadius]
//4. Get point that's the distance and angle from the center of the chosen obstacle
//5. Uniformly pick a vertex to try. Keep trying all vertexes (randomly) until all have been tried or a connection 
//can be made. 
//6. At the end, sample the goal. The idea is that if you can see the goal, go for it. 
void MotionPlanner::ExtendMyApproach_Chris(void)
{
    Clock clk;
    StartTime(&clk);
 
//your code

	//Idea: 
	//1. Unformly pick an obstacle. 
	int obsIdx = PseudoRandomUniformReal(0,m_simulator->GetNrObstacles());
	//2. Next uniformly pick an angle around the obstacle. 
	double angleInDegs = PseudoRandomUniformReal(0,359); //This will get an angle in degrees, be sure to convert to radians
	//before calculating 
	double angleInRads = convertDegsToRads(angleInDegs);
	//3. Finally pick a random distance in the bounds: (radius+robotRadius, radius+robotRadius*2)
	double distFromObs = PseudoRandomUniformReal(m_simulator->GetObstacleRadius(obsIdx)+m_simulator->GetRobotRadius(),m_simulator->GetObstacleRadius(obsIdx)+2*m_simulator->GetRobotRadius());
    //4. Get a pont at an angle around the circle at the random distance. 
	double sto[2];
	sto[0] = m_simulator->GetObstacleCenterX(obsIdx) + distFromObs*cos(angleInRads);
	sto[1] = m_simulator->GetObstacleCenterY(obsIdx) + distFromObs*sin(angleInRads);

	//5. Uniformly pick a vertex until either all vertexs are chosen or one has been added 
	bool isAdded = false;
	int currSize = m_vertices.size();

	vector<Vertex *>tempVec(m_vertices);
	
	//Instead of randomly, we sort by distnace:
	
	//So randomly pick a vertex and keep going through until we are able to add one
	for(int i=0;i<currSize && !isAdded;i++)
	{
	
		int idx = PseudoRandomUniformReal(0,tempVec.size()-1);

		cout<<"Trying idx["<<idx<<" with sto=("<<sto[0]<<", "<<sto[1]<<")"<<endl;
		ExtendTree(idx,sto);
		
		//Keep trying until we find a vertex that works
		if(currSize != m_vertices.size())
		{
			cout<<"ADDED sto=("<<sto[0]<<", "<<sto[1]<<") to vertex: "<<idx<<endl;
			isAdded = true;
		}
		tempVec.erase(tempVec.begin() + idx);

	}
	
	//6. If we added a vertex, we should try connecting the vertex to the goal
	if(isAdded)
	{
		sto[0] = m_simulator->GetGoalCenterX();
		sto[1] = m_simulator->GetGoalCenterY();
		ExtendTree(m_vertices.size()-1,sto);

	}

	cout<<endl<<endl<<"CHRIS: End of my approach, total num vertexs"<<m_vertices.size()<<endl;
	//6. Sample the goal. If there's a clear path, might as well try
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendMyApproach_Brian(void)
{

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
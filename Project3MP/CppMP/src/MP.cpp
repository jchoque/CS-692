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

void MotionPlanner::ExtendRandom(void)
{
    Clock clk;
    StartTime(&clk);
	double sto[2];

	//Uniformly sample a state
	m_simulator->SampleState(sto);

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

	//Now that we've found the min distance, extend the vertex
	ExtendTree(getClosestVid(sto),sto);
    
    m_totalSolveTime += ElapsedTime(&clk);
}


void MotionPlanner::ExtendEST(void)
{
    Clock clk;
    StartTime(&clk);

	// Get our next state to check
	double sto[2];
	m_simulator->SampleState(sto);

	int vid = pickWeightedRandomIdx();
	ExtendTree(vid, sto);

    m_totalSolveTime += ElapsedTime(&clk);
}



//Chris's Approach: Idea is to do the following:
//1. Pick an obstacle. This will use a weighted algorithm, with obstacles closer to the goal
// getting a higher weight. However if this algorithm fails to add a vertex in 50 tries, then it switches to a
// uniform random approach. If the uniform random approach fails 50 times, then it switches back to the weighted. 
// The reason for this is to try and solve getting out of local minimum. 
//
//2. Uniformly pick an angle around the chosen obstacle
//
//3. Uniformly pick a distance from the obstacle, in the bounds [radius+robotRadius, radius+2*robotRadius]
//
//4. Get point that's the distance and angle from the center of the chosen obstacle
//
//5. To get the vertex, this will use an EST based approach where it will attempt to connect to the closest vertex. 
//
//6. At the end, sample the goal. The idea is that if you can see the goal, go for it. 
void MotionPlanner::ExtendMyApproach_Chris(void)
{
    Clock clk;
    StartTime(&clk);
 
//your code

	//1. Pick an obstacle. This should try a weighted approach or a random.
	int obsIdx = 0;
	if(shouldPickRand)
	{
		obsIdx = PseudoRandomUniformReal(0,m_simulator->GetNrObstacles()-1);
	}
	else
	{
		obsIdx = pickWeightedObstacle();

	}	
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
	
	//6. Get the closest vid and try to connect
	if(ExtendTree(getClosestVid(sto),sto))
	{
		failCount = 0;
		//If we were able to add the vertex, see if we can get to the goal
		sto[0] = m_simulator->GetGoalCenterX();
		sto[1] = m_simulator->GetGoalCenterY();
		ExtendTree(m_vertices.size()-1,sto);
	} 
	else
	{
		//We failed to add, so retry
		failCount++;

	}
	
	if(failCount == 50)
	{
		shouldPickRand = !shouldPickRand;
		failCount = 0;
	}

    m_totalSolveTime += ElapsedTime(&clk);
}


// Brian's Approach:  Idea is to do the following:
//1. Pick a random point that is not "close" to another valid vertex
// Closeness in this case is within the width of the robot
//2. Select the vertex closest to this point and try to add the point using that vertex
//3. If that succeeds then continue on
//4. Otherwise find the vertex closest to the goal ignoring the vertex already tried
// and try to add the point from that vertex
void MotionPlanner::ExtendMyApproach_Brian(void)
{
	Clock clk;
    StartTime(&clk);
	// Get our next state to check
	double sto[2];
	bool uniquePoint = false;
	int robotSize = (int)m_simulator->GetRobotRadius() * 2;
	unsigned int vertexCount = m_vertices.size();
	int vid = 0 ; 
	double minDistance;

	// Loop through all verteces to make sure we aren't selecting a point too close to 
	// other verteces
	do{
		m_simulator->SampleState(sto);
		minDistance = sqrt(pow(sto[0] - m_vertices[0]->m_state[0], 2) +
				pow(sto[1] - m_vertices[0]->m_state[1], 2));
		
		bool tooClose = true;
		for (unsigned int i = 0; i < vertexCount; i++){
			double closeness = sqrt(pow(sto[0] - m_vertices[i]->m_state[0], 2) + pow(sto[1] - m_vertices[i]->m_state[1], 2));
			if (closeness <= robotSize){ // We are too close so break out and select a new one
				tooClose = false;
				break;
			} 
			// Remember the vertex closest to this sto
			else if (closeness < minDistance){
				vid = i; 
				minDistance = closeness;
			}
		}
		uniquePoint = tooClose;
	} while (!uniquePoint);

	// Try to add the point from the vertex closest to it
	ExtendTree(vid, sto);

	// If the vertex failed to be added then try finding the closest vertice
	// to the goal besides the one already tried and branch to the point from it
	if (vertexCount == m_vertices.size()){
		minDistance = sqrt(pow(m_simulator->GetGoalCenterX() - m_vertices[0]->m_state[0], 2) +
				pow(m_simulator->GetGoalCenterY() - m_vertices[0]->m_state[1], 2));

		for (unsigned int i = 1; i < vertexCount; i++){
			double closeness = sqrt(pow(m_simulator->GetGoalCenterX() - m_vertices[i]->m_state[0], 2) +
				pow(m_simulator->GetGoalCenterY() - m_vertices[i]->m_state[1], 2));

			// Pick the closest vertex to the goal and not a vertex that was selected the first time
			if (closeness < minDistance && vid != i){ 
				minDistance = closeness;
				vid = i;
			}
		}
		// Try to add the point from the vertex closest to the goal
		ExtendTree(vid, sto);
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

int MotionPlanner:: pickWeightedObstacle()
{
		// Find the maximum children a vertex could have
	double totalWeight = 0;



	double currentWeight = 0.0;
	for (unsigned int i = 0; i < m_simulator->GetNrObstacles(); i++){
		//relative weight
		currentWeight = 
			1.0/(1.0+pow(sqrt( pow(m_simulator->GetGoalCenterX()-m_simulator->GetObstacleCenterX(i),2) +pow(m_simulator->GetGoalCenterY()-m_simulator->GetObstacleCenterY(i),2)),2));
		

		totalWeight += currentWeight;

	}

	// Generate a random number based on the totalWeight and then
	// pick the vertex that matches that weight
	double weightPicked = PseudoRandomUniformReal(0,totalWeight); 

	int idx = 0;  // The vector index selected
	
	//Reinit the total weight
	totalWeight = 0;
	for (unsigned int i = 0; i < m_simulator->GetNrObstacles(); i++){ 
		totalWeight += 1.0/(1.0+pow(sqrt( pow(m_simulator->GetGoalCenterX()-m_simulator->GetObstacleCenterX(i),2) +pow(m_simulator->GetGoalCenterY()-m_simulator->GetObstacleCenterY(i),2)),2)); // Add it to the previous vector weights
		if (totalWeight >= weightPicked){
			idx = i;
			break;
		}
	}

	return idx;

}
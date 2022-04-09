// Astar.h

#pragma once

#include <vector>
#include <map>
#include "GLUT/glut.h"
#include "dependencies.h"
#include "Grid.h"
//#include "robot.h"

using namespace std;

//int CalcID(int x, int y, int theta);
class CGrid;
//class robot;

class CNode
{
public:
	int x;
	int y;
	int theta;
	//vector<int> mNeighbours;  // Keeps the id of the neighbours
	//vector<double> mDistToNeighbours;
	
	int mParent;

	double g_score;
	double h_score;
	double f_score;

	bool operator==(const CNode& rhs)
	{
		return (x == rhs.x && y == rhs.y && theta == rhs.theta);
	}
	bool operator!=(const CNode& rhs)
	{
		return !(*this == rhs);
	}

	CNode() : x(0), y(0), theta(0), g_score(-1), h_score(-1), f_score(-1), mParent(-1) {}
	CNode(int _x, int _y, int _theta): x(_x), y(_y), theta(_theta), g_score(-1), h_score(-1), f_score(-1), mParent(-1)
	{
	}

	void Set(int _x, int _y, int _theta) { x=_x; y=_y; theta=_theta; }
};

//typedef map<int, CNode>						NodeMap;
//typedef map<int, CNode>::iterator			MapIter;
typedef vector<CNode>::iterator			NodeIter;
class AStar
{
public:
	//NodeMap Nodes;
	//MapIter GetNode(int mID);
	int Run(CGrid& grid);
	
	void SortInsertToNode(vector<CNode>& Vec, CNode& Node);
	double heuristic(CNode& node, int goalX, int goalY, int goalTheta);
	int DiffTheta(int Section, int Goal);
	int PrintPath(vector<CNode>& Nodes, int idx);
	void AddNeighbours(CGrid& grid, CNode& node, int posInClosed/*parent ID*/, vector<CNode>& vecOpen, CNode& TargetTmp);
	void AddNeighbour(int x, int y, int theta, CGrid& grid, CNode& node,
							int posInClosed/*parent ID*/, vector<CNode>& vecOpen, CNode& TargetTmp);

	void SetDispFunc(void (*Disp)(void)) { DispFunc=Disp; }
	//void SetRobot(robot* _pRobot) { pRobot=_pRobot; }
	vector<CNode> Path;
	CNode Start;
	CNode Target;

	void (*DispFunc)(void);
	//robot* pRobot;

	~AStar();
};
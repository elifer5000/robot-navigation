// Astar.cpp 

#include <stdio.h>
#include <vector>
#include <algorithm>
#include "Astar.h"

using namespace std;

AStar::~AStar()
{
}

void AStar::SortInsertToNode(vector<CNode>& Vec, CNode& Node)
{
	if (Vec.empty())
	{
		Vec.push_back(Node);
		return;
	}

	int Min=0;
	int Max=Vec.size()-1;
	
	if (Node.f_score <= Vec.front().f_score)
	{
		Vec.insert(Vec.begin(), Node);
		return;
	}

	if (Node.f_score >= Vec.back().f_score)
	{
		Vec.push_back(Node);
		return;
	}
	
	int Middle;

	while (1)
	{
		Middle=(Max+Min)/2;
		if (Node.f_score == Vec[Middle].f_score)
		{
			Vec.insert(Vec.begin()+Middle, Node);
			return;
		}
		if (Node.f_score > Vec[Middle].f_score)
			Min=Middle;
		if (Node.f_score < Vec[Middle].f_score)
			Max=Middle;

		if (Max-Min<=1)
		{
			Vec.insert(Vec.begin()+Min+1, Node);
			return;
		}
	}
}

double AStar::heuristic(CNode& node, int goalX, int goalY, int goalTheta)
{
	//return 0; // dijkstra

	float DX=(float)(node.x - goalX);
	float DY=(float)(node.y - goalY);
	float DTheta=(float)DiffTheta(node.theta, goalTheta);
	return sqrt(DX*DX + DY*DY + DTheta*DTheta);
}

int AStar::DiffTheta(int Section, int Goal)
{
	int Diff=abs(Goal-Section);
	if (Diff>(ThetaSections-1)/2)
		Diff=(ThetaSections-1)-Diff;
	return Diff;
}

int AStar::Run(CGrid& grid)
{
	grid.RestoreGrid();
	// Assume Start Target are "real coordinates"
	CNode StartTmp = Start;
	CNode TargetTmp = Target;
	Path.clear();
	vector<CNode> Closed, Open;

	StartTmp.g_score = 0;
	StartTmp.h_score = heuristic(StartTmp, TargetTmp.x, TargetTmp.y, TargetTmp.theta);
	StartTmp.f_score = StartTmp.h_score;
	Open.push_back(StartTmp);

	int Count=0;
	while (Open.size() > 0)
	{
		CNode XBest = Open.front();	// Should be x_best (having the lowest f_score) (this now happens automagically from SortInsertToNode)
		Open.erase(Open.begin());
		Closed.push_back(XBest);
		grid.DiscreteGrid[XBest.x][XBest.y][XBest.theta] = 4; // It's now in closed
	
		if (++Count>100)
		{
			//pRobot->Slice=XBest.theta;
			DispFunc();
			Count=0;
		}

		if (XBest.x == TargetTmp.x && XBest.y == TargetTmp.y && XBest.theta == TargetTmp.theta)
		{
			PrintPath(Closed, Closed.size()-1);
			printf("\n");
			return 1;
		}

		int Count = 0;
		AddNeighbours(grid, XBest, Closed.size()-1, Open, TargetTmp);
	}

	printf(":( - Sorry, no path found!\n");
	return 0;
}

int AStar::PrintPath(vector<CNode>& Nodes, int idx)
{
	if (idx > 0)
	{
		PrintPath(Nodes, Nodes[idx].mParent);
		printf("-> (%d,%d,%d) ", Nodes[idx].x,Nodes[idx].y,Nodes[idx].theta);
	}
	else
	{
		printf("(%d,%d,%d) ", Nodes[idx].x,Nodes[idx].y,Nodes[idx].theta);
	}
	
	Path.push_back(Nodes[idx]);
	
	return 1;
}

void AStar::AddNeighbour(int x, int y, int theta, CGrid& grid, CNode& node, int posInClosed/*parent ID*/, vector<CNode>& vecOpen, CNode& TargetTmp)
{
	if (grid.DiscreteGrid[x][y][theta] == 0) // This neighbour is also free
	{
		CNode NewNode(x, y, theta);
		NewNode.mParent = posInClosed;
		double Tentative_G_Score = node.g_score + 1.0;
		NewNode.g_score = Tentative_G_Score;
		NewNode.h_score = heuristic(NewNode, TargetTmp.x, TargetTmp.y, TargetTmp.theta);
		NewNode.f_score = NewNode.g_score + NewNode.h_score;
		grid.DiscreteGrid[x][y][theta] = 3; // It's now in open
		SortInsertToNode(vecOpen, NewNode);
	}
	else if (grid.DiscreteGrid[x][y][theta] == 3) // This neighbour is in open
	{
		NodeIter iter = find(vecOpen.begin(), vecOpen.end(), CNode(x, y, theta));
		double Tentative_G_Score = node.g_score + 1.0;
		if (Tentative_G_Score < (*iter).g_score)
		{
			CNode NewNode = *iter;
			NewNode.g_score = Tentative_G_Score;
			NewNode.mParent = posInClosed;
			NewNode.f_score = NewNode.g_score + NewNode.h_score;
			vecOpen.erase(iter);
			SortInsertToNode(vecOpen, NewNode);
		}
	}
}

void AStar::AddNeighbours(CGrid& grid, CNode& node, int posInClosed/*parent ID*/, vector<CNode>& vecOpen, CNode& TargetTmp)
{
	for (int i = node.x-1; i <= node.x+1; i+=2)
	{
		if (i >= 0 && i < GridSize)
		{
			AddNeighbour(i, node.y, node.theta, grid, node, posInClosed, vecOpen, TargetTmp);
		}
	}
	for (int j = node.y-1; j <= node.y+1; j+=2)
	{
		if (j >= 0 && j < GridSize)
		{
			AddNeighbour(node.x, j, node.theta, grid, node, posInClosed, vecOpen, TargetTmp);
		}
	}
					
	// Check neighbours (top-bottom)
	for (int k = node.theta-1; k <= node.theta+1; k+=2)
	{
		int NeighbourSection = k;
		if (NeighbourSection == ThetaSections)
			NeighbourSection = 0;
		else if (NeighbourSection == -1)
			NeighbourSection = ThetaSections-1;

		AddNeighbour(node.x, node.y, NeighbourSection, grid, node, posInClosed, vecOpen, TargetTmp);
	}
}
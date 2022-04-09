#pragma once
#include "dependencies.h"
#include <vector>
#include "point.h"
#include "Astar.h"

using namespace std;

class CGrid
{
public:
	int DiscreteGrid[GridSize][GridSize][ThetaSections];
	//CNode* GridNodes[GridSize][GridSize][ThetaSections];
	
	void AddPolygon(vector<point> Vertices, int slice);
	void AddLine(point p1, point p2, vector<point>& target);

	void Draw(int section);
	void DrawSquare(float x, float y, float Step, float Offset, int Type);
	void RemoveDuplicates(int slice);
	void InitGrid();
	void RestoreGrid();
	CGrid(void);
	~CGrid(void);
};


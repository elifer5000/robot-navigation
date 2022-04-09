#include "Grid.h"
#include "GLUT/glut.h"
#include <algorithm>

void RemoveDuplicatesInVector(vector<point>& vec)
{
	sort(vec.begin(), vec.end());
	vec.erase(unique(vec.begin(), vec.end()), vec.end());
}

CGrid::CGrid(void)
{
	InitGrid();
}

void CGrid::InitGrid()
{
	// Init grid to 0
	for (int i = 0; i < GridSize; i++)
	{
		for (int j = 0; j < GridSize; j++)
		{
			for (int k = 0; k < ThetaSections; k++)
			{
				DiscreteGrid[i][j][k] = 0;
			}
		}
	}
}

void CGrid::RestoreGrid()
{
	for (int i = 0; i < GridSize; i++)
	{
		for (int j = 0; j < GridSize; j++)
		{
			for (int k = 0; k < ThetaSections; k++)
			{
				if (DiscreteGrid[i][j][k] == 3 || DiscreteGrid[i][j][k] == 4) 
					DiscreteGrid[i][j][k] = 0;
			}
		}
	}
}

CGrid::~CGrid(void)
{
}

void CGrid::AddPolygon(vector<point> Vertices, int slice)
{
	if (Vertices.size() < 2)
		return;

	vector<point> ThisPolygonPoints;
	for (unsigned int i = 0; i < Vertices.size()-1; i++)
	{
		AddLine(Vertices[i], Vertices[i+1], ThisPolygonPoints);
	}
	AddLine(Vertices.back(), Vertices[0], ThisPolygonPoints); // last line

	if (ThisPolygonPoints.empty())
		return;
	
	RemoveDuplicatesInVector(ThisPolygonPoints); // Sorts and removes duplicates
	
	// Calculate fill points
	vector<point> FillPoints;
	float curY = 0.0, curX = 0.0;
	float nextCurY = 0.0, nextCurX = 0.0;
	float Step = 1.0f;
	for (unsigned int i = 0; i < ThisPolygonPoints.size()-1; i++)
	{
		curY = ThisPolygonPoints[i].y;
		curX = ThisPolygonPoints[i].x;
		nextCurY = ThisPolygonPoints[i+1].y;
		
		if (compare(nextCurY, curY) == 0) // next one still on the same line
		{
			nextCurX = ThisPolygonPoints[i+1].x;
			curX += Step;
			while (compare(curX, nextCurX) < 0) // fill with points up to the next point
			{
				FillPoints.push_back(point(curX, curY));
				curX += Step;
			}
		}
	}

	for (unsigned int i=0; i<ThisPolygonPoints.size(); ++i)
	{
		if (ThisPolygonPoints[i].x >= 0 && ThisPolygonPoints[i].x < GridSize &&
			ThisPolygonPoints[i].y >= 0 && ThisPolygonPoints[i].y < GridSize)
		{
			int idx = (int)ThisPolygonPoints[i].x;
			int idy = (int)ThisPolygonPoints[i].y;
			if (DiscreteGrid[idx][idy][slice] == 0) // If there was a fill already, leave it like that
				DiscreteGrid[idx][idy][slice] = 2; // 2 is border
		}
	}
		
	for (unsigned int i=0; i<FillPoints.size(); ++i)
	{
		if (FillPoints[i].x >= 0 && FillPoints[i].x < GridSize &&
			FillPoints[i].y >= 0 && FillPoints[i].y < GridSize)
		{
			int idx = (int)FillPoints[i].x;
			int idy = (int)FillPoints[i].y;
			DiscreteGrid[idx][idy][slice] = 1; // 1 is fill
		}
	}
}

void CGrid::AddLine(point p1, point p2, vector<point>& target)
{
	p1.factor(Resolution); 
	p2.factor(Resolution); 
	p1.Round();
	p2.Round();
	float x1 = p1.x, x2 = p2.x;
	float y1 = p1.y, y2 = p2.y;
	
	float dx = abs(x2 - x1);
	float dy = abs(y2 - y1);

	int Stepx = 0, Stepy = 0;
	if (x1 < x2)
		Stepx = 1;
	else
		Stepx = -1;

	if (y1 < y2)
		Stepy = 1;
	else
		Stepy = -1;

	float Error = dx - dy;
	while (true)
	{
		target.push_back(point(x1, y1));
		if (compare(x1, x2) == 0 && compare(y1, y2) == 0)
			break;
		if (2*Error > -dy)
		{
			Error -= dy;
			x1 += Stepx;
			if (compare(x1, x2) == 0 && compare(y1, y2) == 0)
			{
				target.push_back(point(x1, y1));
				break;
			}
		}
		if (2*Error < dx)
		{
			Error += dx;
			y1 += Stepy;
			if (compare(x1, x2) == 0 && compare(y1, y2) == 0)
			{
				target.push_back(point(x1, y1));
				break;
			}
		}
	}
}

void CGrid::Draw(int section)
{
	glPushMatrix();

	glLineWidth(1);
		
	float Step = 1.0/Resolution;
	float Offset = 0.0*Step;
	
	for (int idx = 0; idx < GridSize; idx++)
	{
		for (int idy = 0; idy < GridSize; idy++)
		{
			float x = (float)idx/(float)Resolution;
			float y = (float)idy/(float)Resolution;
			if (DiscreteGrid[idx][idy][section] == 2) // Border
			{
				glColor3f(0.2,0.8,0.2);
				DrawSquare(x, y, Step, Offset, GL_POLYGON);
			}
			else if (DiscreteGrid[idx][idy][section] == 1) // Fill
			{
				glColor3f(0.1,0.4,0.0);
				DrawSquare(x, y, Step, Offset, GL_POLYGON);
			}
			else if (DiscreteGrid[idx][idy][section] == 4) // closed
			{
				glColor3f(0.2,0.3,0.4);
				DrawSquare(x, y, Step, Offset, GL_POLYGON);
			}
			else if (DiscreteGrid[idx][idy][section] == 3) // open
			{
				glColor3f(1.0, 0.8,0.0);
				DrawSquare(x, y, Step, Offset, GL_POLYGON);
			}
		}
	}


	glBegin(GL_LINES);
	glColor3f(0.0,0.0,0.0);
	// Draw horizontal grid lines
	for (int i = 0; i <= GridSize; ++i)
	{
		glVertex2f(0-Step/2, i*Step-Step/2);
		glVertex2f(GridSize*Step-Step/2, i*Step-Step/2);
	}
	// Draw vertical grid lines
	for (int j = 0; j <= GridSize; ++j)
	{
		glVertex2f(j*Step-Step/2, 0-Step/2);
		glVertex2f(j*Step-Step/2, GridSize*Step-Step/2);
	}
	glEnd();

	glPopMatrix();
}

void CGrid::DrawSquare(float x, float y, float Step, float Offset, int Type)
{
	glBegin(Type);
	glVertex2f(x-Step/2.+Offset, y-Step/2.+Offset);//, 0.5);
	glVertex2f(x+Step/2.-Offset, y-Step/2.+Offset);//, 0.5);
	glVertex2f(x+Step/2.-Offset, y+Step/2.-Offset);//, 0.5);
	glVertex2f(x-Step/2.+Offset, y+Step/2.-Offset);//, 0.5);
	glEnd();
}
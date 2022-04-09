// Class automatically generated by Dev-C++ New Class wizard

#ifndef ROBOT_H
#define ROBOT_H

#pragma once

#include <vector>
#include "GLUT/glut.h"
#include "point.h"
#include "dependencies.h"
#include "Grid.h"

using namespace std;
class CGrid;

/*
 * No description
 */
class robot
{
      public:
			/* Class elements */
			vector<point> Vertices; // original robot geometry
			point Center;
           
			vector<point> Normals; // original robot geometry
			vector<point> RotNormals; // Rotated robot normals
			vector<float> RotNormAngles;
			int MinNormAngInd;
           
			vector<point> Vectors; // Vectors from center to verts
			vector<point> RotVectors; // Rotated vectors from center to verts
           
			// Coordinates
			float dx, dy, theta;
			int GridX, GridY, Slice;

			CGrid* pGrid;
			bool Collision;
           
			/* Constructor */
			robot(float _dx=0, float _dy=0, float _theta=0);
                 
			/* Destructor */
			~robot();
                          
			/* Functions */  
			void AddVertex(float x, float y);
			void CalculateNormals();
			void CalculateVectors();
           
			void RotateNormals(float Orient);
			void RotateVectors(float Orient);
           
			void Draw();
			void DrawDiscrete();

		    void MoveForward();
			void MoveBack();
			void TurnLeft();
			void TurnRight();
			void MoveLeft();
			void MoveRight();
			bool isFreeSpace(float x, float y, int theta);

			void ToggleCollision();

			void SetDiscretePos();
			void SetPos();
			void SetGrid(CGrid* _pGrid);
};

#endif // ROBOT_H

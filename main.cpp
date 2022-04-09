#include <stdlib.h>
#include <vector>
#include "GLUT/glut.h"
#include "robot.h"
#include "obstacle.h"
#include "Grid.h"
#include "point.h"
#include "dependencies.h"
#include "math.h"
#include "Astar.h"

using namespace std;

//////////////////// Function Decalrations ////////////////////
/* GLUT callback Handlers */
static void resize(int width, int height);
static void display(void);
static void key(unsigned char key, int x, int y);
static void mouse(int button, int state, int x, int y);
static void motion(int x, int y);
static void specialkey(int key, int x, int y);
static void idle(void);

void Reset();
void DrawObstacles();
void CalculateObstNormals();
void CalculateCObst();
void SetOrto();
void RunAStar();
void SimulatePath(int Dir);

void PrintStatus(int i);
void SetRightClickMenu();
void PathOptions(int value);

////////////////////// Global Variables ///////////////////////
robot Robot;
vector<obstacle> Obstacles;
CGrid Grid;
AStar astar;
vector<CNode> Path;
int PathStep=-1;

/* World parameters */
float WorldWidth=36+2;
float WorldHeight=32+6;
float OLeft=0;
float ORight=WorldWidth;
float OTop=WorldHeight;
float OBottom=0;

float TranslateX = 0;
float TranslateY = 0;
float TotalZoomLevel = 1.0f;
float ZoomLevel = 1.0f;
float ZoomFactor = 1.05f;
int downX = 0, downY = 0;
float ScreenWidth=0, ScreenHeight=0;

double objx=0, objy=0, objz=0;
bool isZooming = false, isPanning = false;
double prevmodelview[16], modelview[16], projection[16];
int viewport[4];

bool ModeMode = false;
bool ShowGrid = true;
bool ShowCObstacles = false;
bool ShowTrail = true;

void InitPrevmodelview()
{
	int i;
	for(i=0; i<16; i++)
		prevmodelview[i]=0;
	for(i=0; i<16; i+=5)
		prevmodelview[i]=1;
}

//////////////////// Function Definitions ////////////////////
/* GLUT callback Handlers */

static void 
resize(int width, int height)
{
	const float ar = (float) width / (float) height;
    
	OLeft=-WorldWidth/2;
	ORight=WorldWidth/2;
	OTop=WorldHeight/2;
	OBottom=-WorldHeight/2;
	
	if (ar>=1) {
		OLeft*=ar;
		ORight*=ar;
	}
	else {
		OTop/=ar;
		OBottom/=ar;
	}

	ScreenWidth = width;
	ScreenHeight = height;
	glViewport(0, 0, width, height);
	SetOrto();
}

static void 
display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glColor3d(1,0,0);

	glPushMatrix();
		if (isZooming)
		{
			glTranslatef(objx, objy, 0);
			glScalef(ZoomLevel, ZoomLevel, 1);
			glTranslatef(-objx, -objy, 0);
		}
		
		glMultMatrixd(prevmodelview); // Multiply accumulated zoom
		
		// 16 is half the bounding box of the total area
		glTranslatef(TranslateX-16, TranslateY-16, 0.0);
		if (ShowCObstacles)
		{
			DrawObstacles();
		}
		
		if (ShowGrid)
		{
			Grid.Draw(Robot.Slice);
		}
		if (ShowTrail && PathStep>0)
		{
			int Temp=PathStep;
			PathStep=-1;
			for (int i=0; i<=Temp; ++i)
			{
				SimulatePath(1);
				if (ShowCObstacles)
					Robot.Draw();
				else if (ShowGrid)
					Robot.DrawDiscrete();
			}
		}
		else
		{
			if (ShowCObstacles)
				Robot.Draw();
			else if (ShowGrid)
				Robot.DrawDiscrete();
		}
		

		glPointSize(8);
		glBegin(GL_POINTS);
		glColor3f(0,0.8,0);
		glVertex2f((float)astar.Start.x/(float)Resolution, (float)astar.Start.y/(float)Resolution);
		glColor3f(0,0,0);
		glVertex2f((float)astar.Target.x/(float)Resolution, (float)astar.Target.y/(float)Resolution);
		glEnd();

	glPopMatrix();
    glutSwapBuffers();
}


static void 
key(unsigned char key, int x, int y)
{
    switch (key) 
    {
		case 27 :
			exit(0);
			break;
		case 'w':
		case 'W':
			Robot.MoveForward();
			break;
		case 's':
		case 'S':
			Robot.MoveBack();
			break;
		case 'a':
		case 'A':
			if (ModeMode)
				Robot.MoveLeft();
			else
				Robot.TurnLeft();
			break;
		case 'd':
		case 'D':
			if (ModeMode)
				Robot.MoveRight();
			else
				Robot.TurnRight();
			break;
		case 'q':
		case 'Q':
			if (ModeMode)
				Robot.TurnLeft();
			else
				Robot.MoveLeft();
			break;
		case 'e':
		case 'E':
			if (ModeMode)
				Robot.TurnRight();
			else
				Robot.MoveRight();
			break;
		case 'r':
		case 'R':
			Reset();
			break;
		case 'm':
		case 'M':
			ModeMode = !ModeMode;
			break;
		case 'c':
		case 'C':
			Robot.ToggleCollision();
			break;
		case 'p':
		case 'P':
			ShowTrail = !ShowTrail;
			break;
		case 'o':
		case 'O':
			 ShowCObstacles = !ShowCObstacles;
			 break;
		case 'g':
		case 'G':
			 ShowGrid = !ShowGrid;
			 break;
		case 't':
		case 'T':
			RunAStar();
			break;
		case '+':
			SimulatePath(1);
			break;
		case '-':
			SimulatePath(-1);
			break;
    }

    glutPostRedisplay();
}

void RunAStar()
{
	printf("Running A* with Start=[%d, %d, %d] and Target=[%d, %d, %d]...\n", astar.Start.x, astar.Start.y, astar.Start.theta, astar.Target.x, astar.Target.y, astar.Target.theta);
	//astar.Run(4, 24, 0, 4, 8, 0);
	astar.Run(Grid);
	Path = astar.Path;
	PathStep=-1;
	SimulatePath(1);
	glutPostRedisplay();
}

void Reset()
{
	isZooming = false;
	isPanning = false;
	TranslateX = 0;
	TranslateY = 0;
	TotalZoomLevel = 1.0;
	ZoomLevel = 1.0;
	objx=0; objy=0; objz=0;
	InitPrevmodelview();
}

void SetOrto()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(OLeft, ORight, OBottom, OTop, -1, 1);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//glTranslatef(-16, -16, 0.0); // to center the room
}


static void 
mouse(int button, int state, int x, int y)
{
	switch (button)
	{
	case GLUT_MIDDLE_BUTTON:
		if (state == GLUT_DOWN)
		{
			glGetDoublev(GL_PROJECTION_MATRIX, projection);
			glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
			glGetIntegerv(GL_VIEWPORT, viewport);
			float z;
			//Read the window z co-ordinate (the z value on that point in unit cube)		
			glReadPixels(x, viewport[3]-y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z);
			//Unproject the window co-ordinates to find the world co-ordinates.
			gluUnProject(x, viewport[3]-y, z, modelview,  projection, viewport, &objx, &objy, &objz);
			//printf("dx: %.3lf, dy:%.3lf\n", objx, objy);
			downY = y;
			isZooming = true;
		}
		else
		{
			// Save current zoom matrix
			glPushMatrix();
				glLoadIdentity();
				glTranslatef(objx, objy, 0);
				glScalef(ZoomLevel, ZoomLevel, 1);
				glTranslatef(-objx, -objy, 0);
				glMultMatrixd(prevmodelview);
				glGetDoublev(GL_MODELVIEW_MATRIX, prevmodelview);
				ZoomLevel = 1.0f; // This is the current view, so reset the zoom level to start again in the next zoom
			glPopMatrix();
			isZooming = false;
		}
		break;
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN)
		{
			downX = x;
			downY = y;
			isPanning = true;
		}
		else 
			isPanning = false;

		break;
	}
	glutPostRedisplay();
}

static void
motion(int x, int y)
{
	if (isZooming)
	{
			if (y > downY && TotalZoomLevel >= 0.01)
			{
				ZoomLevel /= ZoomFactor;
				TotalZoomLevel /= ZoomFactor;
				downY = y;
			}
			else if (y < downY && TotalZoomLevel <= 10.0)
			{
				ZoomLevel *= ZoomFactor;
				TotalZoomLevel *= ZoomFactor;
				downY = y;
			}
			//TotalZoomLevel = MIN(MAX(TotalZoomLevel, 0.01), 10);
			//ZoomLevel = MIN(MAX(ZoomLevel, 0.01), 10);
	}
	else if (isPanning)
	{
		int dx = x - downX;
		int dy = y - downY;
		//printf("dx: %d, dy:%d\n", dx, dy);
		TranslateY -= (float)dy*(WorldHeight/ScreenHeight)/(1.2*TotalZoomLevel);
		TranslateX += (float)dx*(WorldWidth/ScreenWidth)/(1.2*TotalZoomLevel);
		downY = y;
		downX = x;
	}
	glutPostRedisplay();
}

static void 
specialkey(int key, int x, int y)
{
	switch (key) 
	{
	case GLUT_KEY_UP :
		Robot.MoveForward();
		break;
	case GLUT_KEY_DOWN:
		Robot.MoveBack();
		break;
	case GLUT_KEY_LEFT:
		if (ModeMode)
			Robot.MoveLeft();
		else
			Robot.TurnLeft();
		break;
	case GLUT_KEY_RIGHT:
		if (ModeMode)
			Robot.MoveRight();
		else
			Robot.TurnRight();
		break;
	}

	glutPostRedisplay();
}

static void 
idle(void)
{
    glutPostRedisplay();
}

/* Program entry point */
int 
main(int argc, char *argv[])
{
	InitPrevmodelview();
	glutInit(&argc, argv);
	glutInitWindowSize(640,640);
	glutInitWindowPosition(80,50);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

	glutCreateWindow("Robotic Navigation Project");

	glutReshapeFunc(resize);
	glutDisplayFunc(display);
	glutKeyboardFunc(key);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutSpecialFunc(specialkey);
	//glutIdleFunc(idle);

	SetRightClickMenu();

	glClearColor(1,1,1,1);

	//////// Define Robot //////
	PrintStatus(1);
	Robot.dx=4;
	Robot.dy=24;
	Robot.theta=0;
    
	Robot.Center.Set(0,0);
    
	Robot.Vertices.push_back(point(0,0));
	Robot.Vertices.push_back(point(8,0));
	Robot.Vertices.push_back(point(8,1));
	Robot.Vertices.push_back(point(0,1));

	Robot.CalculateNormals();
	Robot.CalculateVectors();
    
	//////// Define Obstacles //////
	PrintStatus(2);
	Obstacles.push_back(obstacle());
	Obstacles.at(Obstacles.size()-1).AddVertex(0,18);
	Obstacles.at(Obstacles.size()-1).AddVertex(10,18);
	Obstacles.at(Obstacles.size()-1).AddVertex(10,19);
	Obstacles.at(Obstacles.size()-1).AddVertex(0,19);
	Obstacles.push_back(obstacle());
	Obstacles.at(Obstacles.size()-1).AddVertex(17,18);
	Obstacles.at(Obstacles.size()-1).AddVertex(18,18);
	Obstacles.at(Obstacles.size()-1).AddVertex(18,29);
	Obstacles.at(Obstacles.size()-1).AddVertex(17,29);
	Obstacles.push_back(obstacle());
	Obstacles.at(Obstacles.size()-1).AddVertex(25,18);
	Obstacles.at(Obstacles.size()-1).AddVertex(32,18);
	Obstacles.at(Obstacles.size()-1).AddVertex(32,19);
	Obstacles.at(Obstacles.size()-1).AddVertex(25,19);
	Obstacles.push_back(obstacle());
	Obstacles.at(Obstacles.size()-1).AddVertex(0,14);
	Obstacles.at(Obstacles.size()-1).AddVertex(19,14);
	Obstacles.at(Obstacles.size()-1).AddVertex(19,15);
	Obstacles.at(Obstacles.size()-1).AddVertex(0,15);
	Obstacles.push_back(obstacle());
	Obstacles.at(Obstacles.size()-1).AddVertex(24,14);
	Obstacles.at(Obstacles.size()-1).AddVertex(31,14);
	Obstacles.at(Obstacles.size()-1).AddVertex(31,15);
	Obstacles.at(Obstacles.size()-1).AddVertex(24,15);
	Obstacles.push_back(obstacle());
	Obstacles.at(Obstacles.size()-1).AddVertex(10,19);
	Obstacles.at(Obstacles.size()-1).AddVertex(11,19);
	Obstacles.at(Obstacles.size()-1).AddVertex(11,20);
	Obstacles.at(Obstacles.size()-1).AddVertex(10,20);
	Obstacles.push_back(obstacle());
	Obstacles.at(Obstacles.size()-1).AddVertex(24,19);
	Obstacles.at(Obstacles.size()-1).AddVertex(25,19);
	Obstacles.at(Obstacles.size()-1).AddVertex(25,20);
	Obstacles.at(Obstacles.size()-1).AddVertex(24,20);
	Obstacles.push_back(obstacle());
	Obstacles.at(Obstacles.size()-1).AddVertex(0,29);
	Obstacles.at(Obstacles.size()-1).AddVertex(32,29);
	Obstacles.at(Obstacles.size()-1).AddVertex(32,30);
	Obstacles.at(Obstacles.size()-1).AddVertex(0,30);
	Obstacles.push_back(obstacle());
	Obstacles.at(Obstacles.size()-1).AddVertex(0,1);
	Obstacles.at(Obstacles.size()-1).AddVertex(1,1);
	Obstacles.at(Obstacles.size()-1).AddVertex(1,29);
	Obstacles.at(Obstacles.size()-1).AddVertex(0,29);
	Obstacles.push_back(obstacle());
	Obstacles.at(Obstacles.size()-1).AddVertex(0,0);
	Obstacles.at(Obstacles.size()-1).AddVertex(32,0);
	Obstacles.at(Obstacles.size()-1).AddVertex(32,1);
	Obstacles.at(Obstacles.size()-1).AddVertex(0,1);
	Obstacles.push_back(obstacle());
	Obstacles.at(Obstacles.size()-1).AddVertex(31,1);
	Obstacles.at(Obstacles.size()-1).AddVertex(32,1);
	Obstacles.at(Obstacles.size()-1).AddVertex(32,29);
	Obstacles.at(Obstacles.size()-1).AddVertex(31,29);

	CalculateObstNormals();
	PrintStatus(3);
	// Calculate CObstacles
	CalculateCObst();
    
	Robot.SetGrid(&Grid);
	astar.SetDispFunc(display);
	//astar.SetRobot(&Robot);

	astar.Start.Set(4*Resolution,24*Resolution,0);
	astar.Target.Set(4*Resolution,8*Resolution,0); // 4,8,0

	glutMainLoop();

	return EXIT_SUCCESS;
}

void DrawObstacles()
{
  int NumObst=Obstacles.size();
     
	for (int i=0; i<NumObst; ++i) {
		Obstacles.at(i).DrawCObst(Robot.Slice);
	}
	for (int i=0; i<NumObst; ++i) {
		Obstacles.at(i).DrawObst();
	}
}

void CalculateObstNormals()
{
	int NumObst=Obstacles.size();
     
	for (int i=0; i<NumObst; ++i) {
		Obstacles.at(i).CalculateNormals();
	}
}

void CalculateCObst()
{
	int NumObst = Obstacles.size();
     
	for (int i=0; i<NumObst; ++i)
	{
		PrintStatus(i+4);
		Obstacles[i].CalculateCObst3D(Robot, Grid);
	}
}

void SimulatePath(int Dir)
{
	if (Path.empty())
		return;

	PathStep+=Dir;
	if (PathStep >= (int)Path.size())
		PathStep = 0;
	if (PathStep < 0)
	{
		int val=Path.size();
		PathStep = val-1;
	}
	
	Robot.dx = (float)Path[PathStep].x/(float)Resolution;    
	Robot.dy = (float)Path[PathStep].y/(float)Resolution;    
	Robot.SetDiscretePos();
	Robot.Slice = Path[PathStep].theta;
	Robot.theta = (float)Robot.Slice/(float)ThetaSections*360.0;
}

void PrintStatus(int i)
{
	switch (i)
	{
	case 1:
		printf("Initializing Robot AI...\n");
		break;
	case 2:
		printf("Creating stimulation environment...\n");
		break;
	case 3:
		printf("Reticulating splines...\n");
		break;
	case 4:
		printf("Studying terrain maps...\n");
		break;
	case 5:
		printf("Squeezing through small corridors...\n");
		break;
	case 6:
		printf("Bumping into obstacles...\n");
		break;
	case 7:
		printf("Searching the iRobot forum...\n");
		break;
	case 8:
		printf("Starting up SkyNet...\n");
		break;
	case 9:
		printf("Writing silly jokes...\n");
		break;
	case 10:
		printf("Repeating the same line as before...\n");
		break;
	case 11:
		printf("Repeating the same line as before...\n");
		break;
	case 12:
		printf("Repeating the same line as before...\n");
		break;
	case 13:
		printf("Running out of ideas...\n");
		break;
	case 14:
		printf("Searching for a search algorithm...\n");
		break;
	}
}

void SetRightClickMenu ()
{
	glutCreateMenu(PathOptions);
	glutAddMenuEntry("Start", 0);
	glutAddMenuEntry("Target", 1);
	glutAddMenuEntry("Run A*", 2);
	glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void PathOptions(int value)
{
	float Step = 1.0f/Resolution;

	switch (value)
	{
	case 0: // Set Start
		if (Robot.GridX >= 0 && Robot.GridX < GridSize &&
			Robot.GridY >= 0 && Robot.GridY < GridSize)
		{
			if (Grid.DiscreteGrid[Robot.GridX][Robot.GridY][Robot.Slice]==1 || Grid.DiscreteGrid[Robot.GridX][Robot.GridY][Robot.Slice]==2)
				return;
			astar.Start.x=Robot.GridX;
			astar.Start.y=Robot.GridY;
			astar.Start.theta=Robot.Slice;
			glutPostRedisplay();
		}
		break;
	case 1: // Set Target
		if (Robot.GridX >= 0 && Robot.GridX < GridSize &&
			Robot.GridY >= 0 && Robot.GridY < GridSize)
		{
			if (Grid.DiscreteGrid[Robot.GridX][Robot.GridY][Robot.Slice]==1 || Grid.DiscreteGrid[Robot.GridX][Robot.GridY][Robot.Slice]==2)
				return;
			astar.Target.x=Robot.GridX;
			astar.Target.y=Robot.GridY;
			astar.Target.theta=Robot.Slice;
		}
		glutPostRedisplay();
		break;
	case 2: // Run A*
		RunAStar();
		break;
	}
}

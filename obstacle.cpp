// Class automatically generated by Dev-C++ New Class wizard

#include "obstacle.h" // class's header file

// class constructor
obstacle::obstacle()
{
}

// class destructor
obstacle::~obstacle()
{
	// insert your code here
}

void obstacle::AddVertex(float x, float y)
{
    point temp(x,y);
    Vertices.push_back(temp);
}

void obstacle::CalculateNormals()
{
    int NumVerts=Vertices.size();
    float Min=10;
    int Ind=0;
    
    for (int i=0; i<NumVerts-1; ++i)
    {
        point temp;
        temp=Vertices[i+1]-Vertices[i];
        temp.Normalize();
        float xtemp=temp.x;
        temp.x=temp.y;
        temp.y=-xtemp;
        Normals.push_back(temp);
        NormAngles.push_back(temp.GetAngle());
        if (NormAngles.back()<Min)
        {
           Min=NormAngles.back();
           Ind=NormAngles.size()-1;
        }
    }
    point temp;
    temp=Vertices[0]-Vertices[NumVerts-1];
    temp.Normalize();
    float xtemp=temp.x;
    temp.x=temp.y;
    temp.y=-xtemp;
    Normals.push_back(temp);
    NormAngles.push_back(temp.GetAngle());
    if (NormAngles.back()<Min)
    {
       Min=NormAngles.back();
       Ind=NormAngles.size()-1;
    }
    MinNormAngInd=Ind;
}

void obstacle::CalculateCObst3D(robot& Robot, CGrid& _grid) {
     for (int i=0; i<ThetaSections; i++)
     {
         vector<point> Slice;
         CalculateCObstSlice(Robot, (float)i/(float)ThetaSections*360.0, Slice);
         Slices.push_back(Slice);
		 _grid.AddPolygon(Slices[i], i);
     }
}

void obstacle::CalculateCObstSlice(robot& Robot, float theta, vector<point>& Slice) {
	Robot.RotateNormals(theta);
	Robot.RotateVectors(theta);

	int Switch;
	vector<float> Angles1;
	vector<float> Angles2;
	vector<point> Vertices1;
	vector<point> Vertices2;
	int MinInd1, MinInd2;

	vector<float> TempAngles=NormAngles;
	unsigned int TempMinNormAngInd=MinNormAngInd;
	
	if ((theta>0 && theta<=90) || (theta>180 && theta<=270))
	{
		if (compare(TempAngles.at(MinNormAngInd), -M_PI) == 0)
			TempAngles.at(MinNormAngInd)=M_PI;
		++TempMinNormAngInd;
		if (TempMinNormAngInd>=TempAngles.size())
			TempMinNormAngInd=0;
	}

	if (Robot.RotNormAngles.at(Robot.MinNormAngInd)<TempAngles.at(TempMinNormAngInd))
	{ // First normal in robot is > than first normal in obstacle
		Switch=-1;
		Angles1=TempAngles;
		Angles2=Robot.RotNormAngles;
		Vertices1=Vertices;
		Vertices2=Robot.RotVectors;
		MinInd1=TempMinNormAngInd;
		MinInd2=Robot.MinNormAngInd;
	}  
	else
	{
		Switch=1;
		Angles1=Robot.RotNormAngles;
		Angles2=TempAngles;
		Vertices1=Robot.RotVectors;
		Vertices2=Vertices;
		MinInd1=Robot.MinNormAngInd;
		MinInd2=TempMinNormAngInd;
	}

	int n1=Angles1.size();
	int n2=Angles2.size();

	// Sort normals from min to max
	SortAngles(Angles1,MinInd1);
	SortAngles(Angles2,MinInd2);

	Angles1.push_back(Angles1.front()+2.0*M_PI);
	Angles2.push_back(Angles2.front()+2.0*M_PI);

	// Find pairs of coordinates using given algorithm
	int i=0,j=0;
	vector<mypair> Pairs;
	while (i<n2)
	{
		Pairs.push_back(mypair(i,j));
		while ((Angles1[j]>=Angles2[i]) && (Angles1[j]<Angles2[i+1]))
		{
			if (Angles1[j]==Angles2[i])
			{
				Pairs.pop_back();
			}

			++j;
			if (j<n1)
			{
				Pairs.push_back(mypair(i,j));
			}
			else
			{
				j=j-n1;
				while (i<n2)
				{
					Pairs.push_back(mypair(i,j));
					++i;
				}
				break;
			}                     
		}
		++i;
	}   

	for (unsigned int i=0; i<Pairs.size(); ++i)
	{
		// Update pairs to match vertices
		Pairs[i].i+=MinInd1;
		Pairs[i].j+=MinInd2+1;

		if (Pairs[i].i>=n1) Pairs[i].i-=n1;
		if (Pairs[i].j>=n2) Pairs[i].j-=n2;

		// Calculate CObst vertices
		point COVert;
		COVert.x=Switch*(Vertices2[(int)Pairs[i].i].x-Vertices1[(int)Pairs[i].j].x);
		COVert.y=Switch*(Vertices2[(int)Pairs[i].i].y-Vertices1[(int)Pairs[i].j].y);
		Slice.push_back(COVert);
	}
}

void obstacle::SortAngles(vector<float>& Angles, int MinInd)
{
     for (int i=0; i<MinInd; ++i) {
         Angles.push_back(Angles.front());
         Angles.erase(Angles.begin());
     }
}
         
void obstacle::DrawObst()
{
     int NumVerts=Vertices.size();
     
     glColor3f(0,0,1);
     glBegin(GL_POLYGON);
        for (int i=0; i<NumVerts; ++i)
        {
            glVertex2f(Vertices[i].x,Vertices[i].y);
        }
     glEnd();

     glColor3f(0,0,0);
     glLineWidth(2);
     glBegin(GL_LINE_LOOP);
        for (int i=0; i<NumVerts; ++i)
        {
            glVertex2f(Vertices[i].x,Vertices[i].y);
        }
     glEnd();
}

void obstacle::DrawCObst(int Slice)
{
     int NumVerts=Slices[Slice].size();
     
     glColor3f(0.4,0.4,0.6);
     glBegin(GL_POLYGON);
        for (int i=0; i<NumVerts; ++i)
        {
            glVertex2f(Slices[Slice].at(i).x,Slices[Slice].at(i).y);
        }
     glEnd();

	 glColor3f(0.5,0.5,0);
	 glLineWidth(2);
     glBegin(GL_LINE_LOOP);
        for (int i=0; i<NumVerts; ++i)
        {
            glVertex2f(Slices[Slice].at(i).x,Slices[Slice].at(i).y);
        }
     glEnd();
}

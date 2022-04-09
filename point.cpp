// Class automatically generated by Dev-C++ New Class wizard

#include "point.h" // class's header file


float RoundFloat(float Number, int nPrecision)
{
	float Multiplier = 1;
	if (nPrecision > 0)
		Multiplier = pow(10.0f, nPrecision);
	float Temp = Number * Multiplier;
	Temp = floor(Temp + 0.5f);
	Number = Temp/Multiplier;

	return Number;
}

// class constructor
point::point(float _x, float _y):
                   x(_x), y(_y)
{
	// insert your code here
}

// class destructor
point::~point()
{
	// insert your code here
}


void point::Rotate(float Orient, bool ConvertDeg2Rad)
{
     if (ConvertDeg2Rad)
     {
        Orient*=M_PI/180.0f;
     }
     
     // uses radians
     point temp;
     temp.x=cos(Orient)*x-sin(Orient)*y;
     temp.y=sin(Orient)*x+cos(Orient)*y;
     *this=temp;
}
 
void point::Round(int precision)
{
	x = RoundFloat(x, precision);
	y = RoundFloat(y, precision);
}

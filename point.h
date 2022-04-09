// Class automatically generated by Dev-C++ New Class wizard

#ifndef POINT_H
#define POINT_H

#pragma once

#include <math.h>
#include "dependencies.h"

#ifndef M_PI
#define M_PI 3.141592f
#endif

/*
 * No description
 */
class point
{
	public:
		float x,y;
        
		// class constructor
		point(float _x=0, float _y=0);
		// class destructor
		~point();
		
		bool operator==(const point& rhs)
		{
			return (compare(rhs.x, this->x) == 0 && compare(rhs.y, this->y) == 0);
		}

		bool operator!=(const point& rhs)
		{
			return !(*this == rhs);
		}
		// Compare so we can sort first according to y, then according to x
		bool operator<(const point& rhs)
		{
			int compY = compare(this->y, rhs.y);
			if (compY < 0)
				return true;
			if (compY == 0)
			{
				int compX = compare(this->x, rhs.x);
				if (compX < 0)
					return true;
			}

			return false;
		}
		bool operator>(const point& rhs)
		{
			int compY = compare(this->y, rhs.y);
			if (compY > 0)
				return true;
			if (compY == 0)
			{
				int compX = compare(this->x, rhs.x);
				if (compX > 0)
					return true;
			}

			return false;
		}


		point operator+(const point& rhs)
		{
				point temp;
				temp.x=(*this).x+rhs.x;
				temp.y=(*this).y+rhs.y;
				return temp;
		}
        
    point operator-(const point& rhs)
    {
        point temp;
        temp.x=(*this).x-rhs.x;
        temp.y=(*this).y-rhs.y;
        return temp;
    }

		void factor(const float& konst)
    {
        x *= konst;
				y *= konst;
    }
        
    void Set(float _x, float _y)
    {
          x=_x;
          y=_y;
    }
             
    void Normalize()
    {
          float Length;
          Length=sqrt(x*x+y*y);
          if (Length>0)
          {
            x/=Length;
            y/=Length;
          }
    }
        
    float GetAngle()
    {
			float Angle=atan2(y,x);
			
			//if (Angle < 0)
			//	Angle += 2*M_PI;
					
			return Angle;
    }
        
    void Rotate(float Orient, bool ConvertDeg2Rad=true);  // uses radians     

		void Round(int precision=0);
};

#endif // POINT_H
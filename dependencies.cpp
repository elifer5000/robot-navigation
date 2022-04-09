#include "dependencies.h"

int compare(float x, float y)
{
  float t = x - y ;
  if (abs(t) < EPS)
		return 0;
  if (t > EPS)
		return 1;
  
	return -1;
}
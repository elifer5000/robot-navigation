#ifndef DEPENDENCIES_H
#define DEPENDENCIES_H

#pragma once

#include <math.h>
// Global variables
#define EPS 0.000001

#define Resolution 3  // resolution of pixels per unit
#define GridSize (32*Resolution) 	// discrete grid size
#define ThetaSections (52*Resolution)	// number of theta layers 52

#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))

#define DEG2RAD(x) ((x)*((M_PI)/(180.0)))
#define RAD2DEG(x) ((x)*((180.0)/(M_PI)))

//        Returns : 0 - if x == y
//                  1 - if x > y
//                 -1 - if x < y
int compare(float x, float y);
#endif // DEPENDENCIES_H
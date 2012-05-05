#ifndef PSEUDO_RANDOM_HPP_
#define PSEUDO_RANDOM_HPP_

#include <cstdlib>
#include <cmath>
#include <math.h>

//Not sure why we need this, but without it, we don't know what pi is.s
#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#if defined (WINDOWS) || defined (_WIN32) || defined (__WIN32__)
  #define OS_WINDOWS
  #define srandom srand
  #define random rand
  #define RANDOM_MAX RAND_MAX
#else
  #define RANDOM_MAX 2147483647
#endif

unsigned int PseudoRandomSeed(void);

static inline double PseudoRandomUniformReal(void)
{
    return ((double) random()) / ((double) RANDOM_MAX);
}

static inline double PseudoRandomUniformReal(const double min, const double max)
{
    return min + (max - min) * PseudoRandomUniformReal();
}



static inline double convertDegsToRads(double pAngleInDegs)
{
	return pAngleInDegs * (M_PI/180);
}

static inline double PseudoRandomRadian(void)
{
	return convertDegsToRads(PseudoRandomUniformReal(0,359));
}

#endif












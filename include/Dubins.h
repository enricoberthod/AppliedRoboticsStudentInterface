//Dubins.h

#ifndef Dubins_H
#define Dubins_H
#include "DataStructure.h"

void Dubins(double,double,double,double,double,double,D*,int *);

//Auxiliary functions declaration
double sinc(double);
double mod2pi(double);
void circline(double,double,double,double, double, double*, double*, double*);
void dubinsarc(C*, double,double,double,double,double);


#endif

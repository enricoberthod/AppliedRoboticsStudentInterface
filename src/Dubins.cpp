#include <iostream>
#include <cmath>
#include <vector>
#include <opencv2/core.hpp>
#include "DataStructure.h"

void circline(double ,double ,double ,double , double, double *, double *, double *);
bool check(double , double ,double ,double ,double ,double ,double ,double);

// Implementation of function sinc(t), returning 1 for t==0, and sin(t)/t otherwise
double sinc(double t)
{
	double s;

	if(abs(t) < 0.002)
 	//For small values of t use Taylor series approximation
		s = 1 - pow(t,2)/6 * (1 - pow(t,2)/20);
 	else
		s = sin(t)/t;

 	return s; 
}

//Normalize an angle (in range [0,2*M_PI))
double mod2pi(double ang)
{
	double out = ang;

	while (out < 0)
		out = out + 2 * M_PI;
  
	while (out >= 2 * M_PI)
		out = out - 2 * M_PI;
	
	return out;
}

//Evaluate an arc (circular or straight) composing a Dubins curve, at a given arc-length s
void circline(double s,double x0,double y0,double th0, double k, double *x, double *y, double *th)
{
	*x = (x0 + s * sinc(k * s / 2.0) * cos(th0 + k * s / 2));
	*y = (y0 + s * sinc(k * s / 2.0) * sin(th0 + k * s / 2));
	*th = mod2pi(th0 + (k * s)); 
}

//Fill the Dubins curve structure
void dubinsarc(C *c, double x0,double y0,double th0,double k,double L)
{
	c->x0 = x0;
	c->y0 = y0;
	c->th0 = th0;
	c->k = k;
	c->L = L;
 	circline(L, x0, y0, th0, k, &c->xf, &c->yf, &c->thf);
}

void dubinscurve(D *curve, double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2)
{
	dubinsarc(&(curve->a1),x0, y0, th0, k0, s1);
	dubinsarc(&(curve->a2),curve->a1.xf, curve->a1.yf, curve->a1.thf, k1, s2);
	dubinsarc(&(curve->a3),curve->a2.xf, curve->a2.yf, curve->a2.thf, k2, s3);
	curve->L = curve->a1.L + curve->a2.L + curve->a3.L;
}

//Normalize an angular difference (range (-M_PI, M_PI])
double rangeSymm(double ang)
{
	double out = ang;
	
	while (out <= -M_PI)
		out = out + 2 * M_PI;
    
	while(out > M_PI)
		out = out - 2 * M_PI;
    
	return out;
}

//Check validity of a solution by evaluating explicitly the 3 equations defining a Dubins problem (in standard form)
bool check(double s1, double k0,double s2,double k1,double s3,double k2,double th0,double thf)
{
	double x0 = -1;
	double y0 = 0;
	double xf = 1;
	double yf = 0;
	
	double eq1 = x0 + s1 * sinc((1/2.) * k0 * s1) * cos(th0 + (1/2.) * k0 * s1)+ s2 * sinc((1/2.) * k1 * s2) * cos(th0 + k0 * s1 + (1/2.) * k1 * s2)+ s3 * sinc((1/2.) * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + (1/2.) * k2 * s3) - xf;
	double eq2 = y0 + s1 * sinc((1/2.) * k0 * s1) * sin(th0 + (1/2.) * k0 * s1)+ s2 * sinc((1/2.) * k1 * s2) * sin(th0 + k0 * s1 + (1/2.) * k1 * s2)+ s3 * sinc((1/2.) * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + (1/2.) * k2 * s3) - yf;
	double eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);
	
	bool Lpos = (s1 > 0) || (s2 > 0) || (s3 > 0);
	bool result = (sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1.e-10) && Lpos;
	
	return result;
}

//Scale the input problem to standard form (x0: -1, y0: 0, xf: 1, yf: 0)
void scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax, double *sc_th0, double *sc_thf, double *sc_Kmax, double *lambda)
{
	//Find transform parameters
	double dx = xf - x0;
	double dy = yf - y0;
	double phi = atan2(dy, dx);
	*lambda = hypot(dx, dy);
	*lambda = *lambda / 2;
	
	//Scale and normalize angles and curvature
	*sc_th0 = mod2pi(th0 - phi);
	*sc_thf = mod2pi(thf - phi);
  	*sc_Kmax = Kmax * (*lambda);
}

//Scale the solution to the standard problem back to the original problem
void scaleFromStandard(double lambda, double sc_s1, double sc_s2, double sc_s3, double *s1, double *s2, double *s3)
{
	*s1 = sc_s1 * lambda;
	*s2 = sc_s2 * lambda;
	*s3 = sc_s3 * lambda;
}

//LSL
void LSL(double sc_th0, double sc_thf, double sc_Kmax, bool *ok, double  *sc_s1, double *sc_s2, double *sc_s3)
{
	double invK = 1 / sc_Kmax;
	double C = cos(sc_thf) - cos(sc_th0);
	double S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
	double temp1 = atan2(C, S);
	*sc_s1 = invK * mod2pi(temp1 - sc_th0);
	double temp2 = 2 + 4 * pow(sc_Kmax,2) - 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
	
	if(temp2 < 0)
	{
		*ok = false; *sc_s1 = 0; *sc_s2 = 0; *sc_s3 = 0;
		return;
	}
	
	*sc_s2 = invK * sqrt(temp2);
	*sc_s3 = invK * mod2pi(sc_thf - temp1);
  	*ok = true;
}

//RSR
void RSR(double sc_th0, double sc_thf, double sc_Kmax, bool *ok, double *sc_s1, double *sc_s2, double *sc_s3)
{
	double invK = 1 / sc_Kmax;
	double C = cos(sc_th0) - cos(sc_thf);
	double S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
	double temp1 = atan2(C, S);
	*sc_s1 = invK * mod2pi(sc_th0 - temp1);
	double temp2 = 2 + 4 * pow(sc_Kmax,2) - 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));
	
	if(temp2 < 0)
	{
		*ok = false; *sc_s1 = 0; *sc_s2 = 0; *sc_s3 = 0;
		return;
	}
	
	*sc_s2 = invK * sqrt(temp2);
	*sc_s3 = invK * mod2pi(temp1 - sc_thf);
  	*ok = true;
}

//LSR
void LSR(double sc_th0, double sc_thf, double sc_Kmax, bool *ok, double *sc_s1, double *sc_s2, double *sc_s3)
{
	double invK = 1 / sc_Kmax;
	double C = cos(sc_th0) + cos(sc_thf);
	double S = 2 * sc_Kmax + sin(sc_th0) + sin(sc_thf);
	double temp1 = atan2(-C, S);
	double temp3 = 4 * pow(sc_Kmax,2) - 2 + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
	
	if (temp3 < 0)
	{
	*ok = false; *sc_s1 = 0; *sc_s2 = 0; *sc_s3 = 0;
	return;
	}
	
	*sc_s2 = invK * sqrt(temp3);
	double temp2 = -atan2(-2, *sc_s2 * sc_Kmax);
	*sc_s1 = invK * mod2pi(temp1 + temp2 - sc_th0);
	*sc_s3 = invK * mod2pi(temp1 + temp2 - sc_thf);
	*ok = true;
}

//RSL
void RSL(double sc_th0, double sc_thf, double sc_Kmax, bool *ok, double *sc_s1, double *sc_s2, double *sc_s3)
{
	double invK = 1 / sc_Kmax;
	double C = cos(sc_th0) + cos(sc_thf);
	double S = 2 * sc_Kmax - sin(sc_th0) - sin(sc_thf);
	double temp1 = atan2(C, S);
	double temp3 = 4 * pow(sc_Kmax,2) - 2 + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));
	
	if(temp3 < 0)
	{
	*ok = false; *sc_s1 = 0; *sc_s2 = 0; *sc_s3 = 0;
	return;
	}
	
	*sc_s2 = invK * sqrt(temp3);
	double temp2 = atan2(2, *sc_s2 * sc_Kmax);
	*sc_s1 = invK * mod2pi(sc_th0 - temp1 + temp2);
	*sc_s3 = invK * mod2pi(sc_thf - temp1 + temp2);
	*ok = true;
}

//RLR
void RLR(double sc_th0, double sc_thf, double sc_Kmax, bool *ok, double *sc_s1, double *sc_s2, double *sc_s3)
{
	double invK = 1 / sc_Kmax;
	double C = cos(sc_th0) - cos(sc_thf);
	double S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
	double temp1 = atan2(C, S);
	double temp2 = 0.125 * (6 - 4 * pow(sc_Kmax,2) + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));
	
	if(abs(temp2) > 1)
	{
	*ok = false; *sc_s1 = 0; *sc_s2 = 0; *sc_s3 = 0;
	return;
	}
	
	*sc_s2 = invK * mod2pi(2 * M_PI - acos(temp2));
	*sc_s1 = invK * mod2pi(sc_th0 - temp1 + 0.5 * (*sc_s2) * sc_Kmax);
	*sc_s3 = invK * mod2pi(sc_th0 - sc_thf + sc_Kmax * (*sc_s2 - *sc_s1));
  	*ok = true;
}

//LRL
void LRL(double sc_th0, double sc_thf, double sc_Kmax, bool *ok, double *sc_s1, double *sc_s2, double *sc_s3)
{
	double invK = 1 / sc_Kmax;
	double C = cos(sc_thf) - cos(sc_th0);
	double S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
	double temp1 = atan2(C, S);
	double temp2 = 0.125 * (6 - 4 * pow(sc_Kmax,2) + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));
	
	if(abs(temp2) > 1)
	{
	*ok = false; *sc_s1 = 0; *sc_s2 = 0; *sc_s3 = 0;
	return;
	}
	
	*sc_s2 = invK * mod2pi(2 * M_PI - acos(temp2));
	*sc_s1 = invK * mod2pi(temp1 - sc_th0 + 0.5 * (*sc_s2) * sc_Kmax);
	*sc_s3 = invK * mod2pi(sc_thf - sc_th0 + sc_Kmax * (*sc_s2 - *sc_s1));
	*ok = true;
}

//Solve the Dubins problem for the given input parameters.
//Return the type and the parameters of the optimal curve
void dubins_shortest_path(double x0, double y0, double th0, double xf, double yf, double thf, double Kmax, int *pidx, D *curve)
{
	//Variables
	double sc_th0,sc_thf,sc_Kmax,lambda;
	bool ok;
	double sc_s1_c,sc_s2_c,sc_s3_c;
	
	//Compute params of standard scaled problem
	scaleToStandard(x0, y0, th0, xf, yf, thf, Kmax, &sc_th0, &sc_thf, &sc_Kmax, &lambda);
	
	//Define the functions corresponding to the different primitives, and the corresponding curvatue signs 
	std::vector<void (*)(double, double, double, bool*, double*, double*, double*)> primitives;
	
	primitives = {*LSL, *RSR, *LSR, *RSL, *RLR, *LRL}; // assign function to a vector  //% LSL  //% RSR //% LSR  //% RSL //% RLR  //% LRL
	
	int ksigns [6][3] = {{1,0,1},{-1,0,-1},{1,0,-1},{-1,0,1},{-1,1,-1},{1,-1,1}};
	
	//Try all the possible primitives, to find the optimal solution
	*pidx = -1;
	double sc_s1, sc_s2, sc_s3, Lcur;
	int L = 1000000000;
	
	for(int i=0;i<primitives.size();i++)
	{
		primitives[i](sc_th0, sc_thf, sc_Kmax, &ok, &sc_s1_c, &sc_s2_c, &sc_s3_c);
		Lcur = sc_s1_c + sc_s2_c + sc_s3_c;
	
		if(ok && Lcur<L)
		{
		      L = Lcur;
		      sc_s1 = sc_s1_c;
		      sc_s2 = sc_s2_c;
		      sc_s3 = sc_s3_c;
		      *pidx = i;
    		}
	}

	if(*pidx>=0) //add an = cause cpp start from 0 
	{
		//Transform the solution to the problem in standard form to the solution of the original problem (scale the lengths)  
		double s1,s2,s3;
		scaleFromStandard(lambda, sc_s1, sc_s2, sc_s3, &s1, &s2, &s3);
	
		//Construct the Dubins curve object with the computed optimal parameters
		dubinscurve(curve, x0, y0, th0, s1, s2, s3, ksigns[*pidx][0]*Kmax, ksigns[*pidx][1]*Kmax, ksigns[*pidx][2]*Kmax);

		//Check the correctness of the algorithm
		if(!check(sc_s1, ksigns[*pidx][0]*sc_Kmax, sc_s2, ksigns[*pidx][1]*sc_Kmax, sc_s3, ksigns[*pidx][2]*sc_Kmax, sc_th0, sc_thf))
	        	std::cout << "Algorithm INcorrect" << std::endl;
		else
                	std::cout << "Algorithm correct" << std::endl;
	}
}

//Main fuction called to compute optimal Dubins solution
void Dubins(double x0,double y0,double xf,double yf, double th0, double thf, D *curve, int *pidx)
{	
        double Kmax = 0.01;

	//Find optimal Dubins solution
        dubins_shortest_path(x0, y0, th0, xf, yf, thf, Kmax, pidx, curve);
}

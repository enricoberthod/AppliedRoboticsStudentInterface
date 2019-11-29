//DataStructure.h

#ifndef DataStructure_H
#define DataStructure_H

#include <opencv2/core.hpp>

/////////////////////////////////////////////////////////////////__________DATA STRUCTURE__________/////////////////////////////////////////////////////////////////

enum colorElem : unsigned short int {RED=1,BLUE=2,BLACK=3};

//Create a structure representing border,gate,obstacles in this order {1,1,n}
struct element{
	colorElem color_element;
 	std::vector<cv::Point> shapeContours;
};

//Create a structure representing circular green elements
struct greenElement{
//vector of 3 points per circle (x,y,radius);
	cv::Vec3f shapeContours;
//numeber of the object
    	std::string number;
};

//Create a structure representing an arc of a Dubins curve (straight or circular)
struct C{
       double x0,y0,th0,k,L,xf,yf,thf;
};

// Create a structure representing a Dubins curve (composed by three arcs)
struct D{
       C a1,a2,a3;
	double L = a1.L+a2.L+a3.L;
};

#endif
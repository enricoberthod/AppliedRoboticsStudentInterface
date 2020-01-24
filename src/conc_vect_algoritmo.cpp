using namespace std;

#include <vector>
#include <iostream>

#define PI 3.14159265

float theta[4] = {0, PI/2, PI, 3*PI/2}



//---DA 0 a n
// Alla posizione iniziale conosco l'angolo, poi calcolo l'angolo migliore per
// la posizione successiva.
// La posizione finale del tratto ROBOT - VITTIMA N°1 diventa la posizione
// iniziale del tratto VITTIMA N°1 - VITTIMA N°2



IDP(){
	angoli[0] = angolo_start_robot;
	function_L(0, angoli[0]);
}


void function_L(int j, float theta_j){
	float min_length = 999999999.0;
	for(int i=0; i<4; i++) {
		Dubins(rightPath[j].a,rightPath[j].b,rightPath[j+1].a,rightPath[j+1].b, theta_j, theta[i], &curve, &pidx);
		if(curve.L < min_length) {
			min_length = curve.L;
			angoli[j+1] = theta[i];
		}
	}
	if(j < (n-1)){			//controllare estremi j +/- 1
		function_L(j+1, angoli[j+1]);
	}
}

//------------------







int main(){

	Dubins(rightPath[i].a,rightPath[i].b,rightPath[i+1].a,rightPath[i+1].b, angles[j], angles[j+1], &curve, &pidx); //add initial theta in and theta out.
	


}


void IDP() {
	D curve;
	float min_length = 999999999.0;
	angoli[];
	L = 0;
	
	for(int j = n-1; j>=0; j--) {   //contrare estremi j +/- 1
		min_length = 999999999.0;
		if(j = n-1) {
			for(int i=0; i<4; i++) {
				Dubins(rightPath[j].a,rightPath[j].b,rightPath[j+1].a,rightPath[j+1].b, theta[0], theta[i], &curve, &pidx);
				if(curve.L < min_length) {
					min_length = curve.L;
					angoli[j+1] = theta[i];
				}
			}
			L = min_length;
		}
		else {
			for(int i=0; i<4; i++) {
				Dubins(rightPath[j].a,rightPath[j].b,rightPath[j+1].a,rightPath[j+1].b, theta[0], theta[i], &curve, &pidx);
				if((curve.L + L) < min_length) {
					min_length = curve.L + L;
					angoli[j+1] = theta[i];
				}
			}
			L = min_length;
		}
		
		
	}
	
}



IDP(){
	for(int j = n-1; j>=0; j--){
		function_L(j, theta[0])
	}
}


function_L(int j, float theta_j){
	float min_length = 999999999.0;
	float min_index = 0;
	for(int i=0; i<4; i++) {
		Dubins(rightPath[j].a,rightPath[j].b,rightPath[j+1].a,rightPath[j+1].b, theta_j, theta[i], &curve, &pidx);
		if(j < (n-1)){
			len = curve.L + function_L(j+1, theta[i])
		}
		if(j = n-1){
			len = curve.L;
		}
		if(len < min_length) {
			min_length = len;
			min_index = i;
			angoli[j+1] = theta[i];
		}
	}
	if(j < (n-1)) {
			function_L(j+1, theta[min_index])
	}
	return min_length;
}

/*
function_L(int j, float theta_j){
	if(j = n-1) {
		L = function_D(j, theta_j)
	}
	else {
		for(int i=0; i<4; i++) {
			L = function_D(j, theta_j) + function_L(j+1, angoli[j+1])
		}
	}
}
*/





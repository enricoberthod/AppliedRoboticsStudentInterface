#include "find_robot.hpp"

/* function find_Robot: given the image of the map, recognize objects on the map
   -parameters:
   	img_in: image of the map
	scale: TODO
	gate: struct with the points of the robot
	x: x coordinate of the center of gravity of the robot
	y: y coordinate of the center of gravity of the robot
	theta: angle of the robot at the beginning
	config_folder: path to reach the param.xml config file
    -return: true if all is ok
*/
bool find_Robot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
	#define FIND_ROBOT_DEBUG_PLOT 

	const double MIN_AREA_SIZE = 50; 
	
	// Convert color space from BGR to HSV
	cv::Mat hsv_img;
	cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

	// Extract blue color region
	cv::Mat blue_mask;    
	cv::inRange(hsv_img, cv::Scalar(100, 120, 150), cv::Scalar(135, 255, 255), blue_mask);
	
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
	cv::erode(blue_mask, blue_mask, kernel);
	cv::dilate(blue_mask, blue_mask, kernel);
	
	// BLUE MASK on real world imgs contains noise to get rid of the noise a possible solution is to use:
  
	// Find blue mask contours
	std::vector<std::vector<cv::Point>> contours;    
	cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

#ifdef FIND_ROBOT_DEBUG_PLOT // do this only if FIND_DEBUG_PLOT is defined
	cv::Mat contours_img;
	contours_img = img_in.clone();
	cv::drawContours(contours_img, contours, -1, cv::Scalar(0,0,0), 4, cv::LINE_AA);
#endif

	std::vector<cv::Point> approx_curve;
	std::vector<std::vector<cv::Point>> contours_approx;    
	bool found = false;
	for (int i=0; i<contours.size(); ++i) { 

		double area = cv::contourArea(contours[i]);
		if (area < MIN_AREA_SIZE) continue; 				// filter too small contours to remove false positives
		approxPolyDP(contours[i], approx_curve, 20, true);
		contours_approx = {approx_curve};
		
		area = cv::contourArea(approx_curve);
	

#ifdef FIND_ROBOT_DEBUG_PLOT   // do this only if FIND_DEBUG_PLOT is defined
	contours_approx = {approx_curve};
	cv::drawContours(contours_img, contours_approx, -1, cv::Scalar(0,0,255), 3, cv::LINE_AA);
#endif

		found = true;  // we found the blue triangle exit!
		break;
	}	

	// If we found the robot triangle set robot position and create the triangle poligon
	if (found) {      
		// emplace back every vertex on triangle (output of this function)
		for (const auto& pt: approx_curve) {        
			triangle.emplace_back(pt.x/scale, pt.y/scale);
			// remember to use the scale to convert the position on the image (pixels) to the position in the arena (meters)
		}

		// Find the position of the robot
		// NB: the position of the robot coincide with the baricenter of the triangle
		double cx = 0, cy = 0;

		// Compute the triangle baricenter
		for (auto vertex: triangle) {
			// NB: triangle point are expressed in meters
			cx += vertex.x; 
			cy += vertex.y;
		}
		cx /= static_cast<double>(triangle.size());
		cy /=  static_cast<double>(triangle.size());

		// Find the robot orientation (i.e the angle of height relative to the base with the x axis)
		double dst = 0;
		Point top_vertex; // 
		for (auto& vertex: triangle) {
			const double dx = vertex.x-cx;      
			const double dy = vertex.y-cy;
			const double curr_d = dx*dx + dy*dy;
			if (curr_d > dst) { 
				dst = curr_d;
				top_vertex = vertex;
			}
		}

		// Store the position of the robot in the output
		x = cx;
		y = cy;

		// Compute the robot orientation
		const double dx = cx - top_vertex.x;
		const double dy = cy - top_vertex.y;
		theta = std::atan2(dy, dx);

#ifdef FIND_ROBOT_DEBUG_PLOT   // do this only if FIND_DEBUG_PLOT is defined   
		// Draw over the imag ethe ba
		cv::Point cv_baricenter(x*scale, y*scale); 									// convert back m to px
		cv::Point cv_vertex(top_vertex.x*scale, top_vertex.y*scale); 				// convert back m to px
		cv::line(contours_img, cv_baricenter, cv_vertex, cv::Scalar(0,255,0), 3);
		cv::circle(contours_img, cv_baricenter, 5, cv::Scalar(0,0,255), -1);
		cv::circle(contours_img, cv_vertex, 5, cv::Scalar(0,255,0), -1);      
#endif  
	}
	return found;    
}

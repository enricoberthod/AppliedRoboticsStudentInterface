#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "undistort_img.hpp"
#include "process_Map.hpp"
#include "PlanPath_Interface.h"
#include "DataStructure.h" 
#include "find_robot.hpp"

#include <stdexcept>
#include <sstream>
#include <cstring>
#include <experimental/filesystem>


using namespace cv;
using namespace std;

const double MIN_AREA_SIZE_ST = 1000;
/////BLACK_MASK
const int Nlr = 0, Nlg = 0, Nlb = 0;
const int Nhr = 180, Nhg = 255, Nhb = 30; 

namespace student {

 void loadImage(cv::Mat& img_out, const std::string& config_folder){  
   throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );
 }

 void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
	static int id = 0;
	cv::imshow("test",img_in);
	char key = (char)waitKey(30);
	if (key=='s') {
		imwrite(config_folder+"/img-saved"+to_string(id)+".jpg", img_in);
		std::cout << config_folder+"/img-saved"+to_string(id)+".jpg"<< std::endl;
		id++;
	}
	//throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );
  }


//--Autodetection border's points
  vector<cv::Point2f> detectBorder(const cv::Mat &img_in){
	cv::namedWindow("BORDER_filter", cv::WINDOW_NORMAL);
	cv::resizeWindow("BORDER_filter", 467, 350);
	
	cv::Mat hsv_img;
	cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
	vector<cv::Point> bordo;
	
	cv::Mat contours_img;
	std::vector<std::vector<cv::Point>> contours, contours_approx;
	std::vector<cv::Point> approx_curve;
	cv::Mat border_mask;
	cv::inRange(hsv_img, cv::Scalar(Nlr, Nlg, Nlb), cv::Scalar(Nhr, Nhg, Nhb), border_mask);
	
	// Filter (applying erosion and dilation) GATE
	kernel = cv::getStructuringElement(cv::MORPH_RECT , cv::Size((2*2) + 1, (2*2)+1));
	cv::erode(border_mask, border_mask, kernel);
	cv::dilate(border_mask, border_mask, kernel);
	
	// Process gate mask
	contours_img = img_in.clone();
	cv::findContours(border_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
	cv::imshow("BORDER_filter", contours_img);
	cv::waitKey(0);
	std::cout << "N. contours: " << contours.size() << std::endl;
	cv::Rect boundRect;

	for (int i=0; i<contours.size(); ++i) {
		std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
		double area = cv::contourArea(contours[i]);
		std::cout << "---- area = " << area << std::endl;
		if (area < MIN_AREA_SIZE_ST) continue; // filter too small contours to remove false positives
		approxPolyDP(contours[i], approx_curve, 20, true);
		if(approx_curve.size() == 4) {
			contours_approx = {approx_curve};
			for (const auto& pt: approx_curve) {        
				bordo.emplace_back(pt.x, pt.y);
			}
			cv::drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
			std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
			boundRect = cv::boundingRect(cv::Mat(approx_curve)); 
		}
	}
	
	cv::Point centro = cv::Point(boundRect.x+(boundRect.width/2),boundRect.y+(boundRect.height/2));
	cv::circle( contours_img, centro, 8, cv::Scalar(0,170,220), -1, 8, 0 );
	cout << "circle -> " << centro << endl;
	cv::imshow("BORDER_filter", contours_img);
	cv::waitKey(0);

	vector<cv::Point2f> b;
	
	int x_min = centro.x;
	int	y_min = centro.y;
	for(int i =0; i<bordo.size(); i++) {
		if(bordo[i].x <= x_min &&  bordo[i].y >= x_min) {
			b.emplace_back(bordo[i].x+15, bordo[i].y-15);
			//bordo.erase(i);
		}
	}
	for(int i =0; i<bordo.size(); i++) {
		if(bordo[i].x >= x_min &&  bordo[i].y >= x_min) {
			b.emplace_back(bordo[i].x-15, bordo[i].y-15);
			//bordo.erase(i);
		}
	}
	for(int i =0; i<bordo.size(); i++) {
		if(bordo[i].x >= x_min &&  bordo[i].y <= x_min) {
			b.emplace_back(bordo[i].x-15, bordo[i].y+15);
			//bordo.erase(i);
		}
	}
	for(int i =0; i<bordo.size(); i++) {
		if(bordo[i].x <= x_min &&  bordo[i].y <= x_min) {
			b.emplace_back(bordo[i].x+15, bordo[i].y+15);
			//bordo.erase(i);
		}
	}
	
	for(int i = 0; i < b.size(); i++) {
		cout << b[i] << endl; 
		cv::circle( contours_img, b[i], 15, cv::Scalar(0,170,220), -1, 8, 0 );
		cv::imshow("BORDER_filter", contours_img);
		cv::waitKey(0);
	}
	
	//bordo.size();
	//std::cout << "   Elements: " << gate.size() << std::endl;
  
	//cv::imshow("BORDER_filter", contours_img);
	//cv::waitKey(0);
	return b; 
  }


  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" ); 

	std::string file_path = config_folder + "/extrinsicCalib.csv";

    std::vector<cv::Point2f> image_points;

    if (!std::experimental::filesystem::exists(file_path)){
          
      std::experimental::filesystem::create_directories(config_folder);
      
      image_points = detectBorder(img_in);
      // SAVE POINT TO FILE
      // std::cout << "IMAGE POINTS: " << std::endl;
      // for (const auto pt: image_points) {
      //   std::cout << pt << std::endl;
      // }
      std::ofstream output(file_path);
      if (!output.is_open()){
        throw std::runtime_error("Cannot write file: " + file_path);
      }
      for (const auto pt: image_points) {
        output << pt.x << " " << pt.y << std::endl;
      }
      output.close();
    }else{
      // LOAD POINT FROM FILE
      std::ifstream input(file_path);
      if (!input.is_open()){
        throw std::runtime_error("Cannot read file: " + file_path);
      }
      while (!input.eof()){
        double x, y;
        if (!(input >> x >> y)) {
          if (input.eof()) break;
          else {
            throw std::runtime_error("Malformed file: " + file_path);
          }
        }
        image_points.emplace_back(x, y);
      }
      input.close();
    }
    
	
    cv::Mat dist_coeffs;
	
    dist_coeffs   = (cv::Mat1d(1,4) << 0, 0, 0, 0, 0);
	
    bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);
	
    // cv::Mat Rt;
    // cv::Rodrigues(rvec_, Rt);
    // auto R = Rt.t();
    // auto pos = -R * tvec_;
	
    if (!ok) {
      std::cerr << "FAILED SOLVE_PNP" << std::endl;
    }

    return ok;
  }

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
	undistort_img(img_in, img_out, cam_matrix, dist_coeffs, config_folder);
  }

  //-------------------------------------------------------------------------
  //          FIND PLANE TRANSFORM
  //-------------------------------------------------------------------------


   void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, 
                          const std::vector<cv::Point2f>& dest_image_points_plane, cv::Mat& plane_transf, const std::string& config_folder){
    
    cv::Mat image_points;

    // project points
    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);

    plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
  }

  
  //-------------------------------------------------------------------------
  //          UNWARP TRANSFORM
  //-------------------------------------------------------------------------
  void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, 
            const std::string& config_folder){
      cv::warpPerspective(img_in, img_out, transf, img_in.size());
  }

  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );   
	process_Map(img_in, scale, obstacle_list, victim_list, gate, config_folder);
	return true;
  }

  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
    	std::cout << "siamo qui 1" << std::endl;
	find_Robot(img_in, scale, triangle, x, y, theta, config_folder);
	//throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );    
	std::cout << "siamo qui" << std::endl;
	return true;
  }

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );     
    plan_Path123(borders, obstacle_list, victim_list, gate, x, y, theta, path, config_folder);
	return true;
  }


}


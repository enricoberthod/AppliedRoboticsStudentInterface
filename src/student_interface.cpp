#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "undistort_img.hpp"
#include "pippo.hpp"

#include <stdexcept>
#include <sstream>
#include <cstring>


using namespace cv;
using namespace std;

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

  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );   
  }

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
	

	pippo_function(img_in, img_out, cam_matrix, dist_coeffs, config_folder);


	//undistort_img(img_in, img_out, cam_matrix, dist_coeffs, config_folder);

/*
	 //cv::undistort(img_in, img_out, cam_matrix, dist_coeffs);
	 // OPTIMIZED VERSION
	    static bool maps_initialized = false;
	    static cv::Mat full_map1, full_map2;

	    if(!maps_initialized){
	      // Note: m1type=CV_16SC2 to use fast fixed-point maps (see cv::remap)
	      cv::Mat R;
	      cv::initUndistortRectifyMap(cam_matrix, dist_coeffs, R, cam_matrix, 
			                img_in.size(), CV_16SC2, full_map1, full_map2);

	      maps_initialized = true;
	    }

	    // Initialize output image    
	    cv::remap(img_in, img_out, full_map1, full_map2, cv::INTER_LINEAR); 
    //throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );  
*/
  }

  //-------------------------------------------------------------------------
  //          FIND PLANE TRANSFORM
  //-------------------------------------------------------------------------
  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, cv::Mat& plane_transf, const std::string& config_folder){
    
    cv::Mat image_points;

    // project points
    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);

    // find homography
    std::vector<cv::Point2f> plane_points;
    for (const auto pt: object_points_plane) {
      plane_points.emplace_back(pt.x, pt.y);
    }

    plane_transf = cv::getPerspectiveTransform(image_points, plane_points);
  }

  //-------------------------------------------------------------------------
  //          UNWARP TRANSFORM
  //-------------------------------------------------------------------------
  void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, const double scale, const std::string& config_folder){
    cv::warpPerspective(img_in, img_out, transf, img_in.size());
  }

  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );   
	process_Map(img_in, scale, obstacle_list, victim_list, gate, config_folder);
  }

  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );    
  }

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );     
  }


}


#include "undistort_img.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>


void undistort_img(const cv::Mat& img_in, cv::Mat& img_out, 
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder)
{
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
}


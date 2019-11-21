#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <utility>

#include <opencv2/opencv.hpp>
#include "process_Map.hpp"

#define PI 3.14159265

using namespace std;
using namespace cv;

bool process_Map(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){
	
	cv::Mat hsv_img;
	cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);
	
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));

	redRegions(hsv_img, img_in, kernel, scale, obstacle_list);
	
	findGate(hsv_img,img_in, kernel, scale, gate);
	findVictim(hsv_img,img_in, kernel, scale, victim_list);	
	
	return true;
}



void redRegions(cv::Mat hsv_img, cv::Mat img_in, cv::Mat kernel, const double scale, std::vector<Polygon>& obstacle_list) {
		
	namedWindow("Original 4", WINDOW_NORMAL);
	resizeWindow("Original 4", 678, 498);
	
	std::vector<std::vector<cv::Point>> contours, contours_approx;
	std::vector<cv::Point> approx_curve;
	
	// Find red regions: h values around 0 (positive and negative angle: [0,15] U [160,179])
	cv::Mat contours_img, red_mask_low, red_mask_high, red_mask;
	cv::inRange(hsv_img, cv::Scalar(RLlr, RLlg, RLlb), cv::Scalar(RLhr, RLhg, RLhb), red_mask_low);
	cv::inRange(hsv_img, cv::Scalar(RHlr, RHlg, RHlb), cv::Scalar(RHhr, RHhg, RHhb), red_mask_high);
	cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask); // combine together the two binary masks

	// Filter (applying an erosion and dilation) the image
	cv::erode(red_mask, red_mask, kernel);
	cv::dilate(red_mask, red_mask, kernel);
	
	contours_img = img_in.clone();
	cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
	std::cout << "" << std::endl;
	std::cout << "RED MASK"<< std::endl;
	std::cout << "N. contours (Elements): " << contours.size() << std::endl;
	for (int i=0; i<contours.size(); ++i) {
		std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
		double area = cv::contourArea(contours[i]);
		//cout << "area : " << area << endl;
		if (area < MIN_AREA_SIZE) continue;
		approxPolyDP(contours[i], approx_curve, 7, true);
		contours_approx={approx_curve};
		
		Polygon p;
		for (const auto& pt: approx_curve) {
			p.emplace_back(pt.x/scale, pt.y/scale);
		}
		
		obstacle_list.emplace_back(p);
		drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
		std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
		imshow("Original 4", contours_img);
		waitKey(0);
	}
	
	std::cout << "   Elements: " << obstacle_list.size() << std::endl;
	cv::imshow("Original 4", contours_img);
	cv::waitKey(0);
}



void findGate(cv::Mat hsv_img, Mat img_in, cv::Mat kernel, const double scale, Polygon& gate) {
	cv::namedWindow("GATE_filter", WINDOW_NORMAL);
	cv::resizeWindow("GATE_filter", 678, 498);
	
	cv::Mat contours_img;
	std::vector<std::vector<cv::Point>> contours, contours_approx;
	std::vector<cv::Point> approx_curve;
	cv::Mat gate_mask;
	Mat victim_mask;
	cv::inRange(hsv_img, cv::Scalar(Glr, Glg, Glb), cv::Scalar(Ghr, Ghg, Ghb), gate_mask);
	victim_mask = gate_mask.clone();
	
	// Filter (applying erosion and dilation) GATE
	kernel = cv::getStructuringElement(cv::MORPH_RECT , cv::Size((2*2) + 1, (2*2)+1));
	cv::erode(gate_mask, gate_mask, kernel);
	cv::dilate(gate_mask, gate_mask, kernel);
	
	// Process gate mask
	contours_img = img_in.clone();
	cv::findContours(gate_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
	std::cout << "N. contours: " << contours.size() << std::endl;
	std::vector<cv::Rect> boundRect(contours.size());

	for (int i=0; i<contours.size(); ++i) {
		std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
		double area = cv::contourArea(contours[i]);
		//std::cout << "---- area = " << area << std::endl;
		if (area < MIN_AREA_SIZE) continue; // filter too small contours to remove false positives
		approxPolyDP(contours[i], approx_curve, 7, true);
		if(approx_curve.size() == 4) {
			contours_approx = {approx_curve};
			for (const auto& pt: approx_curve) {        
				gate.emplace_back(pt.x/scale, pt.y/scale);
			}
			drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
			std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
		}
	}
	
	std::cout << "   Elements: " << gate.size() << std::endl;
  
	cv::imshow("GATE_filter", contours_img);
	cv::waitKey(0);
}

void findVictim(cv::Mat hsv_img, Mat img_in, cv::Mat kernel, const double scale, std::vector<std::pair<int,Polygon>>& victim_list) {
	//namedWindow("VICTIM_filter", WINDOW_NORMAL);
	//resizeWindow("VICTIM_filter", 678, 498);
	
	// Find green regions
	cv::Mat green_mask;
	// store a binary image in green_mask where the white pixel are those contained in HSV rage (45,40,40) --> (75,255,255)  
	cv::inRange(hsv_img, cv::Scalar(45, 40, 40), cv::Scalar(75, 255, 255), green_mask);
  
	// Apply some filtering
	// Create the kernel of the filter i.e. a rectanble with dimension 3x3
	kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
	// Dilate using the generated kernel
	cv::dilate(green_mask, green_mask, kernel);
    cv::dilate(green_mask, green_mask, kernel);
	cv::dilate(green_mask, green_mask, kernel);
	cv::dilate(green_mask, green_mask, kernel);

	// Find contours
	std::vector<std::vector<cv::Point>> contours, contours_approx;  
  
	// Create an image which we can modify not changing the original image!
	cv::Mat contours_img;
	contours_img = img_in.clone();

	// Finds contours in a binary image.
	cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);  
    
	// create an array of rectangle (i.e. bounding box containing the green area contour)  
	std::vector<cv::Rect> boundRect(contours.size());
	for (int i=0; i<contours.size(); ++i)
	{
		double area = cv::contourArea(contours[i]);
		if (area < MIN_AREA_SIZE) continue; // filter too small contours to remove false positives

		std::vector<cv::Point> approx_curve;
		approxPolyDP(contours[i], approx_curve, 10, true);
		if(approx_curve.size() < 6) continue; //fitler out the gate 
		contours_approx = {approx_curve};

		// Draw the contours on image with a line color of BGR=(0,170,220) and a width of 3
		drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);

		// find the bounding box of the green blob approx curve
		boundRect[i] = boundingRect(cv::Mat(approx_curve)); 
	}
     
	cv::Mat green_mask_inv;

	// Init a matrix specify its dimension (img.rows, img.cols), default color(255,255,255) 
	// and elemet type (CV_8UC3).
	cv::Mat filtered(img_in.rows, img_in.cols, CV_8UC3, cv::Scalar(255,255,255));

	// generate binary mask with inverted pixels w.r.t. green mask -> black numbers are part of this mask
	cv::bitwise_not(green_mask, green_mask_inv); 
  
	// Load digits template images
	std::vector<cv::Mat> templROIs;
	for (int i=1; i<=10; ++i) {
		auto num_template = cv::imread(template_folder + std::to_string(i) + ".png");
		// Store the template in templROIs (vector of mat)
		templROIs.emplace_back(num_template);
	}  
  
	img_in.copyTo(filtered, green_mask_inv);   // create copy of image without green shapes
  
	// create a 3x3 recttangular kernel for img filtering
	kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2)+1));
  
	// For each green blob in the original image containing a digit
	for (int i=0; i<boundRect.size(); ++i)
	{
		// Constructor of mat, we pass the original image and the coordinate to copy and we obtain
		// an image pointing to that subimage
		cv::Mat processROI(filtered, boundRect[i]); // extract the ROI containing the digit
    
		if (processROI.empty()) continue;
    
		// The size of the number in the Template image should be similar to the dimension
		// of the number in the ROI
		cv::resize(processROI, processROI, cv::Size(200, 200)); // resize the ROI 
		cv::threshold( processROI, processROI, 100, 255, 0 );   // threshold and binarize the image, to suppress some noise
    
		// Apply some additional smoothing and filtering
		cv::erode(processROI, processROI, kernel);
		cv::GaussianBlur(processROI, processROI, cv::Size(5, 5), 2, 2);
		cv::erode(processROI, processROI, kernel);
		cv::dilate(processROI, processROI, kernel);
		cv::dilate(processROI, processROI, kernel);
		cv::erode(processROI, processROI, kernel);
		cv::erode(processROI, processROI, kernel);

    
		// Show the actual image used for the template matching
		cv::imshow("ROI", processROI);
    
		// Find the template digit with the best matching
		double maxScore = 0;
		int maxIdx = -1;
		for (int j=0; j<templROIs.size(); ++j) {
			cv::Mat result1;
			cv::Mat result2;
			Mat dst1;
			Mat dst2;
			dst1 = Mat::zeros(templROIs[j].rows, templROIs[j].cols, CV_8UC3);
			dst2 = Mat::zeros(processROI.rows, processROI.cols, CV_8UC3);
			cv::Point centro;
	
			float m1 = f_linea(templROIs[j], dst1, centro);
			float m2 = f_linea(processROI, dst2, centro);
		
			float n = fabs((m1-m2)/(1+m1*m2));
			float angolo = atan(n) *180/PI;
		
			Mat dst3;
			Mat rot1;
			Mat dst4;
			Mat rot2;
			double score1;
			double score2;
			double score;
			
			rot1 = getRotationMatrix2D(centro, angolo, 1);
			warpAffine(dst2, dst3, rot1, Size(processROI.cols,processROI.rows), 1, 0, Scalar(0, 0, 0));
			rot2 = getRotationMatrix2D(centro, -angolo, 1);
			warpAffine(dst2, dst4, rot2, Size(processROI.cols,processROI.rows), 1, 0, Scalar(0, 0, 0));
		
			// Match the ROI with the templROIs j-th
			cv::matchTemplate(dst3, dst1, result1, cv::TM_CCOEFF);
			cv::matchTemplate(dst4, dst1, result2, cv::TM_CCOEFF);
			cv::minMaxLoc(result1, nullptr, &score1); 
			cv::minMaxLoc(result2, nullptr, &score2);
			score = max(score1,score2);

			// Compare the score with the others, if it is higher save this as the best match!
			if (score > maxScore) {
				maxScore = score;
				maxIdx = j+1;
			}
	  
			angolo = 8;
			for(int k=0; k <2; k++) {
				rot1 = getRotationMatrix2D(centro, angolo, 1);
				warpAffine(dst3, dst3, rot1, Size(processROI.cols,processROI.rows), 1, 0, Scalar(0, 0, 0));
				rot2 = getRotationMatrix2D(centro, -angolo, 1);
				warpAffine(dst4, dst4, rot2, Size(processROI.cols,processROI.rows), 1, 0, Scalar(0, 0, 0));
		
				cv::matchTemplate(dst3, dst1, result1, cv::TM_CCOEFF);
				cv::matchTemplate(dst4, dst1, result2, cv::TM_CCOEFF);
		
				cv::minMaxLoc(result1, nullptr, &score1); 
				cv::minMaxLoc(result2, nullptr, &score2);
				score = max(score1,score2);

				// Compare the score with the others, if it is higher save this as the best match!
				if (score > maxScore) {
					maxScore = score;
					maxIdx = j+1;
				}
			}
	
			angolo = 24;
			rot1 = getRotationMatrix2D(centro, -angolo, 1);
			warpAffine(dst3, dst3, rot1, Size(processROI.cols,processROI.rows), 1, 0, Scalar(0, 0, 0));
			rot2 = getRotationMatrix2D(centro, angolo, 1);
			warpAffine(dst4, dst4, rot2, Size(processROI.cols,processROI.rows), 1, 0, Scalar(0, 0, 0));
	
			cv::matchTemplate(dst3, dst1, result1, cv::TM_CCOEFF);
			cv::matchTemplate(dst4, dst1, result2, cv::TM_CCOEFF);

			cv::minMaxLoc(result1, nullptr, &score1); 
			cv::minMaxLoc(result2, nullptr, &score2);
			score = max(score1,score2);

			// Compare the score with the others, if it is higher save this as the best match!
			if (score > maxScore) {
				maxScore = score;
				maxIdx = j+1;
			}
		
			angolo = 8;
			rot1 = getRotationMatrix2D(centro, -angolo, 1);
			warpAffine(dst3, dst3, rot1, Size(processROI.cols,processROI.rows), 1, 0, Scalar(0, 0, 0));
			rot2 = getRotationMatrix2D(centro, angolo, 1);
			warpAffine(dst4, dst4, rot2, Size(processROI.cols,processROI.rows), 1, 0, Scalar(0, 0, 0));

			cv::matchTemplate(dst3, dst1, result1, cv::TM_CCOEFF);
			cv::matchTemplate(dst4, dst1, result2, cv::TM_CCOEFF);
		
			cv::minMaxLoc(result1, nullptr, &score1); 
			cv::minMaxLoc(result2, nullptr, &score2);
			score = max(score1,score2);

			// Compare the score with the others, if it is higher save this as the best match!
			if (score > maxScore) {
				maxScore = score;
				maxIdx = j+1;
			}
	
		}
    
		if(maxIdx>5) {
			maxIdx = maxIdx - 5;
		}
	
		// Display the best fitting number
		std::cout << "Best fitting template: " << maxIdx << std::endl;    
		cv::waitKey(0);
	}
}

/*

int main() {
	//inizializzazione
	string filename = "04.jpg";
	Mat img = imread(filename.c_str());
	double scale = 800/1.56;
	vector<Polygon> obstacle_list;
	vector<std::pair<int,Polygon>> victim_list;
	Polygon gate;
	string config_folder = "/cygdrive/d/Enrico/SCUOLA/Università/LABORATORY OF APPLIED ROBOTICS/fork";
	
	processMap(img, scale, obstacle_list, victim_list, gate, config_folder);
	
	return 0;
} */





float f_linea(Mat src, Mat dst, cv::Point& C) {
	cvtColor(src, src, COLOR_BGR2GRAY);
	threshold(src, src, 220, 255, THRESH_BINARY);
	vector<Mat> contours;
	Mat hierarchy;
	Mat linea;
	findContours(src, contours, hierarchy, CV_RETR_CCOMP, CHAIN_APPROX_SIMPLE);
	Mat cnt = contours[contours.size()-1];
	
	fitLine(cnt, linea, CV_DIST_HUBER, 0, 0.01, 0.01);
	drawContours(dst, contours, contours.size()-1, Scalar(0,255,0), 1, 8, hierarchy, 100);
	
	float vx = linea.at<float>(0,0);
	float vy = linea.at<float>(1,0);
	float x = linea.at<float>(2,0);
	float y = linea.at<float>(3,0);
	
	float lefty = ((-x*vy/vx)+y);
	float righty = (((src.cols-x)*vy/vx)+y);
	cv::Point point1 = cv::Point(src.cols-1,righty);
	cv::Point point2 = cv::Point(0, lefty);
	
	float m = (float)(point1.y - point2.y)/(point1.x - point2.x);
	C.x = (point1.x + point2.x)/2;
	C.y = (point1.y + point2.y)/2;

	return m;
}

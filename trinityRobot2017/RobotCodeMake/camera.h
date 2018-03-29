#pragma once
#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>
#include <cmath>
using namespace cv;
using namespace std;



class Camera {
public:
	Camera(std::string targetFile);
	double getOffsetX();
	double getRotation();
	bool poll();
private:
	double offset;
	double rotation;
	VideoCapture stream1;
	Mat imgo,img,imgg, imgg2, ing2o, img2, desc1, desc2;
	std::vector<KeyPoint> kpts1, kpts2;
	Ptr<Feature2D> akaze;
};






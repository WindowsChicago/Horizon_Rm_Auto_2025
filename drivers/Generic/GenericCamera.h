#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

class GNCamera
{
public:
	GNCamera()
	{}

	~GNCamera()
	{}

	bool GetMat(cv::Mat &src)
	{
			cv::Mat matImage;
			//capture >> matImage;
			src = matImage;
			return true;
	};

	
private:


};

//
//  motion_structure.hpp
//  Motion Tracking
//
//  Created by Henry Chopp on 3/6/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#ifndef motion_structure_hpp
#define motion_structure_hpp

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

class MotionStruct {
	Mat prevStruct;
	Mat currStruct;
	SimpleBlobDetector detector;
	
public:
	MotionStruct(void);
	void update(Mat);
	Mat compareStructs(void);
};

#endif /* motion_structure_hpp */

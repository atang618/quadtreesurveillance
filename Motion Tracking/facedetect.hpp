//
//  facedetect.hpp
//  Motion Tracking
//
//  Created by Allen Tang on 3/3/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#ifndef facedetect_hpp
#define facedetect_hpp


#include <stdlib.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/highgui.hpp"

using namespace cv;
using namespace std;

class HaarDetector {
public:
    HaarDetector(string type);
    void detectFace(Mat frame);
    vector<Rect> faceBoxes;
    
private:
    CascadeClassifier classifier;
};

#endif /* facedetect_hpp */

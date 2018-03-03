//
//  qt_decompose.hpp
//  QT_Track
//
//  Created by Henry Chopp on 3/1/18.
//  Copyright © 2018 Henry Chopp. All rights reserved.
//

#ifndef qt_decompose_hpp
#define qt_decompose_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

void qt_decomp(Mat* im, Mat* structure, Rect portion, int* byteCount, double stdThresh);


#endif /* qt_decompose_hpp */


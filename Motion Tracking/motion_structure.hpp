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
    static Mat prevStruct;
    static Mat currStruct;
    static Mat background;
    
public:
    MotionStruct(Mat, Mat);
    void update(Mat);
    Mat generateStructs(Mat);
};


#endif /* motion_structure_hpp */

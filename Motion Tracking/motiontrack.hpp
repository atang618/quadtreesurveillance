//
//  motiontrack.hpp
//  Motion Tracking
//
//  Created by Allen Tang on 2/24/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#ifndef motiontrack_hpp
#define motiontrack_hpp

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

class MotionTracker {
    static Mat prevFrame;
    static Mat currentFrame;
    static Mat deltaFrame;
    static void backgroundSubtract(void);

public:
    vector<Rect> boundingBoxes;
    Rect largestBox;
    double largestArea;
    Mat thresh;
    MotionTracker(Mat);
    void setPrev(Mat);
    void setCurrent(Mat);
    void findBoundingBox(int,double);
    void update(Mat);
    bool compare();


};


#endif /* motiontrack_hpp */

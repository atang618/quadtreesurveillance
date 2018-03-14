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
#include <opencv2/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "kcftracker.hpp"

using namespace cv;
using namespace std;


enum State {Tracked, Occluded, Lost};

struct BBOX {
    Rect2d rect;
    Point2d center;
};

struct COR {
    Rect2d BBox;
    vector<int> matchedTO;
};

struct TO {
    KCFTracker tracker;
    Rect2d BBox;
    State state;
    vector<int> occludeTO;
    vector<int> overlapTO;
    int matchedCOR;
    int counterOCC;
    int counterOL;
    int counterLOST;
};

class MotionTracker {
    static Mat prevFrame;
    static Mat currentFrame;
    static Mat deltaFrame;
    static void backgroundSubtract(void);
    static bool mergeBoxes(vector<BBOX> input, vector<BBOX>* output, double minDist);

public:
    vector<Rect2d> FSBoxes;
    Rect largestBox;
    double largestArea;
    Mat thresh;
    MotionTracker(Mat);
    void setPrev(Mat);
    void setCurrent(Mat);
    void findBoundingBox(int threshValue, int winSize, double minArea, double maxArea, double minDist);
    void update(Mat);
    bool compare();


};

Point2d findCenter(Rect2d rect);
double findOverlap(Rect2d A, Rect2d B);
double findPercentIntersect(Rect2d A, Rect2d B);

#endif /* motiontrack_hpp */

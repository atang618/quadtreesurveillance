//
//  motiontrack.cpp
//  Motion Tracking
//
//  Created by Allen Tang on 2/24/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#include "motiontrack.hpp"

Mat MotionTracker::prevFrame;
Mat MotionTracker::deltaFrame;
Mat MotionTracker::currentFrame;


MotionTracker::MotionTracker(Mat init){
    if (init.channels()!=1){
        cvtColor(init, currentFrame, CV_BGR2GRAY);
    } else {
        currentFrame = init;
    }
}

void MotionTracker::backgroundSubtract(void) {
    GaussianBlur(currentFrame, currentFrame, Size(9,9), 0);
    absdiff(prevFrame, currentFrame, deltaFrame);
    //imshow("Delta Frame", deltaFrame);
    
}

void MotionTracker::setPrev(Mat prev){
    if (prev.channels()!=1){
        cvtColor(prev, prevFrame, CV_BGR2GRAY);
    } else {
        prev.copyTo(prevFrame);
    }
}

void MotionTracker::setCurrent(Mat curr){
    if (curr.channels()!=1){
        cvtColor(curr, currentFrame, CV_BGR2GRAY);
    } else {
        curr.copyTo(currentFrame);
    }
    
}


void MotionTracker::update(Mat input) {
    currentFrame.copyTo(prevFrame);
    if (input.channels()!=1){
        cvtColor(input, currentFrame, CV_BGR2GRAY);
    } else {
        currentFrame = input;
    }
}

void MotionTracker::findBoundingBox(int threshValue, double minArea) {
    backgroundSubtract();
    vector<vector<Point>> contours;
    vector<Vec4i> heirarchy;
    threshold(deltaFrame, thresh, threshValue, 255, CV_THRESH_BINARY);
    Mat element = getStructuringElement(MORPH_RECT, Size(35,35));
    dilate(thresh, thresh, element,Point(-1,-1),1);
    imshow("Thresh",thresh);
    findContours(thresh, contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    boundingBoxes.clear();
    largestArea = minArea;
    double area;
    for (int i = 0; i < contours.size(); i++) {
        area = contourArea(contours[i],false);
        if (area > minArea && (heirarchy[i][3] < 0)) {
			boundingBoxes.push_back(boundingRect(contours[i]));
		}
        if (i == 0 || area > largestArea) {
            largestBox = boundingRect(contours[i]);
            largestArea = area;
        }
    }
}


bool MotionTracker::compare(){
    if (equal(prevFrame.begin<int>(), prevFrame.end<int>(), currentFrame.begin<int>())){
        return true;
    } else {
        return false;
    }
}


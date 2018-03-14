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


Point2d findCenter(Rect2d rect) {
    Point2d result;
    result.x = rect.x + rect.width/2;
    result.y = rect.y + rect.height/2;
    return result;
}

double findOverlap(Rect2d A, Rect2d B) {
    // Returns the percent overlap
    return ((A & B).area())/((A | B).area());
}

double findPercentIntersect(Rect2d A, Rect2d B) {
    // Returns the overlap as fraction of A
    return ((A & B).area())/(A.area());
}
    
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

void MotionTracker::findBoundingBox(int threshValue, int winSize, double minArea, double maxArea, double minDist) {
    backgroundSubtract();
    vector<vector<Point>> contours;
    vector<Vec4i> heirarchy;
    threshold(deltaFrame, thresh, threshValue, 255, CV_THRESH_BINARY);
    Mat element = getStructuringElement(MORPH_RECT, Size(winSize,winSize));
    dilate(thresh, thresh, element);
//    dilate(thresh, thresh, element);
//    erode(thresh, thresh, element);
    
    imshow("Thresh",thresh);
    findContours(thresh, contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    FSBoxes.clear();
    vector<BBOX> potentialCandidates;
    double area;
    for (int i = 0; i < contours.size(); i++) {
        area = contourArea(contours[i],false);
        if (area > minArea && (heirarchy[i][3] < 0)) {
            BBOX tempBox;
            tempBox.rect = boundingRect(contours[i]);
            tempBox.center = findCenter(tempBox.rect);
            potentialCandidates.push_back(tempBox);
		}
    }
    // Iteratively merge boxes with overlap or close to each other
    bool fullymerged = false;
    vector<BBOX> finalCandidates;
    while (!fullymerged) {
        fullymerged = mergeBoxes(potentialCandidates, &finalCandidates, minDist);
        if (!fullymerged) {
            potentialCandidates = finalCandidates;
            finalCandidates.clear();
        }
    }
    for (int i = 0; i < finalCandidates.size(); i++) {
        // Avoid entire frame bounding boxes
        if (finalCandidates[i].rect.area() < maxArea) {
            Rect2d newCandidate;
            newCandidate = finalCandidates[i].rect;
            FSBoxes.push_back(newCandidate);
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

bool MotionTracker::mergeBoxes(vector<BBOX> input, vector<BBOX>* output, double minDist) {
    bool* coveredBoxes = (bool*) calloc(input.size(),sizeof(bool));
    bool noOverlap = true;
    for (int i = 0; i < input.size(); i++){
        for (int j = 0; j < input.size(); j++){
            if (j > i && !coveredBoxes[j]){
                if ((input[i].rect & input[j].rect).area() > 0 || (norm(input[i].center - input[j].center) < minDist)) {
                    BBOX newBox;
                    newBox.rect = input[i].rect | input[j].rect;
                    newBox.center = findCenter(newBox.rect);
                    output->push_back(newBox);
                    coveredBoxes[i] = true;
                    coveredBoxes[j] = true;
                    break;
                    }
                }
        }
        if (coveredBoxes[i]) {
            noOverlap = false;
        } else { // did not overlap with any other boxes
            output->push_back(input[i]);
        }
    }
    free(coveredBoxes);
    return noOverlap;
}

//
//  main.cpp
//  Motion Tracking
//
//  Created by Allen Tang on 2/22/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "motiontrack.hpp"
#include "kalman.hpp"
#include "qt_decompose.hpp"
#include "facedetect.hpp"
#include "motion_structure.hpp"

using namespace cv;

int main(int argc, const char * argv[]) {

    //  Open camera session
    VideoCapture stream1(0);
    if (!stream1.isOpened()) {std::cout << "Cannot open camera";}
    
    //  Get initial frame
    Mat testFrame;
    stream1.read(testFrame);
    int rows = testFrame.rows;
    int cols = testFrame.cols;
    
    // Frame Rect
    Rect frameRect;
    frameRect.x = 0;
    frameRect.y = 0;
    frameRect.width = 512;
    frameRect.height = 512;
    
    // Create ROI
    Rect roi;
    roi.x = (cols-512)/2;
    roi.y = (rows-512)/2;
    roi.width = 512;
    roi.height = 512;
    
    // Initialize Additional Frames
    Mat bgrFrame, crop;
    
    // Initialize Motion Trackers
    // Background Subtraction
    flip(testFrame,crop,1);
    MotionTracker BSTracker(crop(roi));
    Rect BSBox;

    // KCF
    Ptr<Tracker> KCFTracker;
    KCFTracker = TrackerKCF::create();
    Rect2d KCFBox;
    bool KCFTracking = false;
    
    // Initialize Kalman Filter Tracker
    TKalmanFilter kalmanTracker(roi.width/2, roi.height/2, 50, true);
    float u_estimate, v_estimate, u_predict, v_predict;
    Rect KFBox, prevKFBox;
    prevKFBox.x = (512-80)/2;
    prevKFBox.y = (512-80)/2;
    prevKFBox.width = 80;
    prevKFBox.height = 80;
    
    // Initialize Face Detector
    //HaarDetector faceTracker("ALT");
    
    // Initialize Motion Struct
    Mat structure = Mat::zeros(512, 512, CV_8UC1);
    MotionStruct mStruct;
    
    
    //  Stream until keypress
    while (true) {
        stream1.read(bgrFrame);
        flip(bgrFrame, crop, 1);
        crop = crop(roi);
        BSTracker.update(crop);
        BSTracker.findBoundingBox(25, 1000.0);
//        faceTracker.detectFace(crop);
//        BSBox = faceTracker.faceBoxes[0];
        BSBox = BSTracker.largestBox;
        
        if (!KCFTracking && BSTracker.largestArea > 3000 && BSTracker.largestArea < (350*350)) {
            KCFBox = BSBox;
            KCFTracker->init(crop, KCFBox);
            KCFTracking = true;
        } else if (KCFTracking) {
            KCFTracker->update(crop, KCFBox);
        } else {
            ;
        }
        
        // Kalman Filter
        u_estimate = (float) (BSBox.x + BSBox.width/2);
        v_estimate = (float) (BSBox.y + BSBox.height/2);
        
//        if (BSBox.width < prevKFBox.width - 20 || BSBox.height < prevKFBox.height - 20) {
//            ;
//        } else {
            if (kalmanTracker.KalmanTracking(&u_estimate, &v_estimate, &u_predict, &v_predict)) {
                // If the KF filter is not lost, update KFBox
                KFBox.x = (int) (u_estimate - BSBox.width/2);
                KFBox.y = (int) (v_estimate - BSBox.height/2);
                KFBox.width = BSBox.width;
                KFBox.height = BSBox.height;
            } else {
                // If lost, keep previous box
                KFBox = prevKFBox;
            }
//        }


//        faceTracker.detectFace(crop);


        structure = Mat::zeros(512, 512, CV_8UC1);
        for (int i = 0; i < BSTracker.boundingBoxes.size(); i++) {
            Rect qtBox = BSTracker.boundingBoxes[i];

//        for (int i = 0; i < faceTracker.faceBoxes.size(); i++) {
//            Rect qtBox = faceTracker.faceBoxes[i];
            //rectangle(crop, qtBox, Scalar(0,255,0),3);
            
            vector<Point> fillPoints;
            fillPoints.push_back(Point(qtBox.x, qtBox.y));
            fillPoints.push_back(Point(qtBox.x + qtBox.width, qtBox.y));
            fillPoints.push_back(Point(qtBox.x + qtBox.width, qtBox.y + qtBox.height));
            fillPoints.push_back(Point(qtBox.x, qtBox.y + qtBox.height));
            
            fillConvexPoly(structure, fillPoints, Scalar(255));
        }

        
        threshold(structure, structure, 100, 255, CV_THRESH_BINARY);
        mStruct.update(structure);
        structure = mStruct.compareStructs();
        imshow("",structure);
        
		int newBytes = 262144;
		qt_decomp(&crop, &structure, frameRect, &newBytes, 10);
		cout << "QT Bytes : Original Bytes = " << to_string(double(newBytes)/262144.0) << "\n";
        rectangle(crop, KFBox, Scalar(0,0,255),3);
        if (KCFTracking) {
            rectangle(crop, KCFBox, Scalar(255,0,0),3);
        }

        imshow("QT",structure);
        imshow("Test",crop);
        
        prevKFBox = KFBox;
        
        if (waitKey(30) >= 0) {break;}
    }
    
    
    return 0;
}


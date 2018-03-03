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
    MotionTracker BSTracker(testFrame(roi));
    Rect BSBox;
    // KCF
//    Ptr<Tracker> KCFTracker;
//    KCFTracker = TrackerKCF::create();
//    Rect2d KCFBox = roi;
//    KCFTracker->init(testFrame(roi), KCFBox);
    
    
    // Initialize Kalman Filter Tracker
    TKalmanFilter kalmanTracker((roi.x + roi.width)/2, (roi.y + roi.height)/2,true);
    float u_estimate, v_estimate, u_predict, v_predict;
    Rect KFBox;
    
    // Initialize Quad Tree

    
    //  Stream until keypress
    while (true) {
        stream1.read(bgrFrame);
        flip(bgrFrame, crop, 1);
        crop = crop(roi);
        BSTracker.update(crop);
        BSTracker.findBoundingBox(10, 1000.0);
        BSBox = BSTracker.boundingBoxes[0];
//        KCFTracker->update(crop, KCFBox);
        u_estimate = (float) (BSBox.x + BSBox.width)/2;
        v_estimate = (float) (BSBox.y + BSBox.height)/2;
        kalmanTracker.KalmanTracking(&u_estimate, &v_estimate, &u_predict, &v_predict);
        KFBox.x = (int) u_estimate - BSBox.width/2;
        KFBox.y = (int) v_estimate - BSBox.height/2;
        KFBox.width = BSBox.width;
        KFBox.height = BSBox.height;
        rectangle(crop, KFBox, Scalar(0,0,255),3);
//        rectangle(crop, KCFBox, Scalar(255,0,0),3);
        Mat structure = Mat::zeros(512, 512, CV_8UC1);
        for (int i = 0; i < BSTracker.boundingBoxes.size(); i++) {
            Rect qtBox = BSTracker.boundingBoxes[i];
            rectangle(crop, qtBox, Scalar(0,255,0),3);
            
            vector<Point> fillPoints;
            fillPoints.push_back(Point(qtBox.x, qtBox.y));
            fillPoints.push_back(Point(qtBox.x + qtBox.width, qtBox.y));
            fillPoints.push_back(Point(qtBox.x + qtBox.width, qtBox.y + qtBox.height));
            fillPoints.push_back(Point(qtBox.x, qtBox.y + qtBox.height));
            
            fillConvexPoly(structure, fillPoints, Scalar(255));
        }
        Scalar mean, stddev;
        meanStdDev(crop, mean, stddev);
        cout << stddev << "\n";
        qt_decomp(&crop, &structure, frameRect);
        
        imshow("QT",structure);
        imshow("Test",crop);
        if (waitKey(30) >= 0) {break;}
    }
    
    
    return 0;
}


//
//  main.cpp
//  Motion Tracking
//
//  Created by Allen Tang on 2/22/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "motiontrack.hpp"
#include "kalman.hpp"

using namespace cv;

int main(int argc, const char * argv[]) {
    
    //  Open camera session
    VideoCapture stream1(0);
    if (!stream1.isOpened()) {std::cout << "Cannot open camera";}
    
    //  Get initial frame
    Mat test_frame;
    stream1.read(test_frame);
    int rows = test_frame.rows;
    int cols = test_frame.cols;
    
    
    // Create ROI
    Rect roi;
    roi.x = (cols-512)/2;
    roi.y = (rows-512)/2;
    roi.width = 512;
    roi.height = 512;
    
    // Initialize Motion Tracker
    MotionTracker tracker(test_frame(roi));
    Mat prevTemp;
    Mat bgrFrame, crop;
    Rect measuredBox, trackBox;
    
    // Initialize Kalman Filter Tracker
    TKalmanFilter kalmanTracker((roi.x + roi.width)/2, (roi.y + roi.height)/2,true);
    float u_estimate, v_estimate, u_predict, v_predict;
    
    //  Stream until keypress
    while (true) {
        stream1.read(bgrFrame);
        flip(bgrFrame, crop, 1);
        crop = crop(roi);
        tracker.update(crop);
        tracker.findBoundingBox(15, 1000.0);
        measuredBox = tracker.boundingBoxes[0];
        u_estimate = (float) (measuredBox.x + measuredBox.width)/2;
        v_estimate = (float) (measuredBox.y + measuredBox.height)/2;
        kalmanTracker.KalmanTracking(&u_estimate, &v_estimate, &u_predict, &v_predict);
        trackBox.x = (int) u_estimate - measuredBox.width/2;
        trackBox.y = (int) v_estimate - measuredBox.height/2;
        trackBox.width = measuredBox.width;
        trackBox.height = measuredBox.height;
        rectangle(crop, trackBox, Scalar(0,0,255),3);
        for (int i = 0; i < tracker.boundingBoxes.size(); i++) {
            rectangle(crop, tracker.boundingBoxes[i], Scalar(0,255,0),3);
        }
        imshow("Test",crop);
        if (waitKey(30) >= 0) {break;}
    }
    
    
    return 0;
}


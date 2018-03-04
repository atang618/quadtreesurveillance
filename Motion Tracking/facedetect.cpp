//
//  facedetect.cpp
//  Motion Tracking
//
//  Created by Allen Tang on 3/3/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#include "facedetect.hpp"


HaarDetector::HaarDetector(string type) {
    if (type == "ALT") {
        classifier.load("../../../Data/haarcascades/haarcascade_frontalface_alt.xml");
    } else if (type == "CAT") {
        classifier.load("../../../Data/haarcascades/haarcascade_frontalfacecatface.xml");
    }else {
        classifier.load("../../../Data/haarcascades/haarcascade_frontalface_default.xml");
    }
}

void HaarDetector::detectFace(Mat frame) {
    Mat frame_gray;
    cvtColor(frame, frame_gray, CV_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);
    faceBoxes.clear();
    
    // Detect faces
    classifier.detectMultiScale(frame_gray, faceBoxes, 1.1, 3,
                                  0|CASCADE_SCALE_IMAGE, Size(30, 30));
        
}

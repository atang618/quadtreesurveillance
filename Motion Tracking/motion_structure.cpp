//
//  motion_structure.cpp
//  Motion Tracking
//
//  Created by Henry Chopp on 3/6/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#include "motion_structure.hpp"


Mat MotionStruct::prevStruct;
Mat MotionStruct::currStruct;
Mat MotionStruct::background;

MotionStruct::MotionStruct(Mat init, Mat backgnd) {
    currStruct = init;
    background = backgnd;
}

void MotionStruct::update(Mat structure) {
    currStruct.copyTo(prevStruct);
    currStruct = structure;
}

Mat MotionStruct::generateStructs(Mat backgnd) {
    Mat labelsP, statsP, centroidsP;
    Mat labelsC, statsC, centroidsC;
    int nlabelsP = connectedComponentsWithStats(prevStruct, labelsP, statsP, centroidsP, 4, CV_32S);
    int nlabelsC = connectedComponentsWithStats(currStruct, labelsC, statsC, centroidsC, 4, CV_32S);
    
    cout << "P,C = " << nlabelsP << "," << nlabelsC << "\n";
    // This case works
    if (nlabelsP < nlabelsC) {
        Mat element = getStructuringElement(MORPH_RECT, Size(10,10));
        dilate(currStruct,currStruct,element);
    } else if (nlabelsP > nlabelsC) {
        for (int i = 0; i < nlabelsP; i++) {
            Rect roi;
            roi.x = statsP.at<int>(i,CC_STAT_LEFT);
            roi.y = statsP.at<int>(i,CC_STAT_TOP);
//            if (roi.x > 10) {
//                roi.x -= 10;
//            }
//            if (roi.y > 10) {
//                roi.y -= 10;
//            }
            
            roi.width = statsP.at<int>(i,CC_STAT_WIDTH);
            roi.height = statsP.at<int>(i,CC_STAT_HEIGHT);
//            if (roi.width + 30 > 512) {
//                roi.width += 30;
//            }
//            if (roi.height + 30 > 512) {
//                roi.height += 30;
//            }
            
            
            //			Mat xorMat;
            //			bitwise_xor(prevStruct(roi), currStruct(roi), xorMat);
            //
            //			if ((double)(countNonZero(xorMat))/roi.area() > 0) {
            //				bitwise_or(prevStruct(roi), currStruct(roi), currStruct(roi));
            //			}
            Mat backgrayCurr, backgrayPrev, backDiff;
            cvtColor(backgnd, backgrayCurr, CV_BGR2GRAY);
            cvtColor(background, backgrayPrev, CV_BGR2GRAY);
            absdiff(backgrayCurr(roi), backgrayPrev(roi), backDiff);
            threshold(backDiff, backDiff, 15, 255, CV_THRESH_BINARY); // 15
            double avg = countNonZero(backDiff);
            cout << "\tBackground: " << avg/roi.area() << "\n";
            if (avg/roi.area() > 0.05) {
                bitwise_or(prevStruct(roi), currStruct(roi), currStruct(roi));
            }
        }
    } else if (nlabelsP == nlabelsC) {
//        for (int i = 1; i < nlabelsP; i++) {
//            Rect roi;
//            roi.x = statsP.at<int>(1,CC_STAT_LEFT);
//            roi.y = statsP.at<int>(1,CC_STAT_TOP);
//            roi.width = statsP.at<int>(1,CC_STAT_WIDTH);
//            roi.height = statsP.at<int>(1,CC_STAT_HEIGHT);
//            
//            Mat xorMat;
//            bitwise_xor(prevStruct(roi), currStruct(roi), xorMat);
//            
//            if ((double)(countNonZero(xorMat))/roi.area() > 0.8) {
//                bitwise_or(prevStruct(roi), currStruct(roi), currStruct(roi));
//            }
//        }
    }
    
    if (countNonZero(currStruct) == 512*512) {
        bitwise_xor(currStruct, currStruct, currStruct);
    }
    //	if (countNonZero(currStruct) == 0) {
    //		background = backgnd;
    //		cout << "\tBackground Updated\n";
    //	}
    Mat backgroundMask, foregroundMask;
    bitwise_not(currStruct, backgroundMask);
    cvtColor(currStruct, foregroundMask, CV_GRAY2BGR);
    cvtColor(backgroundMask, backgroundMask, CV_GRAY2BGR);
    bitwise_and(foregroundMask, background, foregroundMask);
    bitwise_and(backgroundMask, backgnd, background);
    bitwise_or(foregroundMask, background, background);
    imshow("Background",background);
    
    //imshow("Curr", currStruct);
    return currStruct;
}

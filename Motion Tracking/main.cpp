//
//  main.cpp
//  Motion Tracking
//
//  Created by Allen Tang on 2/22/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//
#include <iostream>
#include <fstream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "motiontrack.hpp"
#include "kalman.hpp"
#include "qt_decompose.hpp"
#include "facedetect.hpp"
#include "motion_structure.hpp"

#define THRESH 15
#define WINSIZE 15
#define MIN_AREA 1000
#define MAX_AREA 25000
#define MIN_SEP 20
#define MIN_OVERLAP 0.4
#define COR_ENCLOSED 0.7
#define TO_ENCLOSED 0.7
#define TOL 0.8
#define TOH 1.2
#define CONS_FRAMES 7
#define OVERLAP_FRAMES 2
#define LOST_FRAMES 3

using namespace cv;

int main(int argc, const char * argv[]) {
    clock_t start;
//    ofstream res;
//    res.open("PETS09-S2L1.txt");
    //  Open camera session
    //VideoCapture stream1(0);
    //VideoCapture stream1("../../../TestVideos/MOT16-01.mp4");
    VideoCapture stream1("../../../TestVideos/PETS09-S2L1.mp4");
    //VideoCapture stream1("../../../TestVideos/SecCam2.avi");
    if (!stream1.isOpened()) {std::cout << "Cannot open video";}
    
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
    // Frame Subtraction
    MotionTracker BSTracker(testFrame(roi));
    //MotionTracker BSTracker(testFrame);
    
    // KCF
    vector<COR> candidates;
    vector<TO> trackedObj;
    
    
    // Initialize Face Detector
    //HaarDetector faceTracker("ALT");
    
    // Initialize Motion Struct
    MotionStruct mStruct(Mat::zeros(512, 512, CV_8UC1), Mat::zeros(512, 512, CV_8UC3));
//    MotionStruct mStruct(Mat::zeros(rows, cols, CV_8UC1), Mat::zeros(rows, cols, CV_8UC3));
    
    int imgInd = 0;
    
    //  Stream until keypress
    while (true) {
        imgInd++;
        stream1.read(bgrFrame);
        //stream1.read(crop);
        if (bgrFrame.empty()){
            break;
        }
        start = clock();
        crop = bgrFrame(roi);
        Mat cropOrg = crop.clone();
        //Mat canvasCOR = crop.clone();
//        imshow("Original", crop);
        
        // Frame Subtraction
        BSTracker.update(crop);
        BSTracker.findBoundingBox(THRESH, WINSIZE, MIN_AREA, MAX_AREA, MIN_SEP);
        

        // Background Subtraction
        Mat structure = Mat::zeros(512, 512, CV_8UC1);
        //Mat structure = Mat::zeros(rows, cols, CV_8UC1);
        
        for (int i = 0; i < BSTracker.FSBoxes.size(); i++){
            rectangle(structure, BSTracker.FSBoxes[i], Scalar(255),-1);
        }
        mStruct.update(structure);
        structure = mStruct.generateStructs(cropOrg);
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(structure, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
        
        candidates.clear();
        for (int i = 0; i < contours.size(); i++){
            COR newCandidate;
            newCandidate.BBox = boundingRect(contours[i]);
            candidates.push_back(newCandidate);
        }
        
        // Matching TO to COR
        if (trackedObj.size() == 0) {
            // No tracked objects yet
            for (int i = 0; i < candidates.size(); i++) {
                TO newTracked;
                newTracked.BBox = candidates[i].BBox;
//                newTracked.tracker = TrackerKCF::create();
//                newTracked.tracker->init(crop, newTracked.BBox);
                newTracked.tracker.init(newTracked.BBox,crop);
                newTracked.state = Tracked;
                newTracked.matchedCOR = i;
                newTracked.counterOCC = 0;
                newTracked.counterOL = 0;
                newTracked.counterLOST = 0;
                trackedObj.push_back(newTracked);
            }
        } else {
            for (int i = 0; i < trackedObj.size(); i++) {
                // Update each TO and assign states
//                bool trackSuccess = trackedObj[i].tracker->update(crop,trackedObj[i].BBox);
                trackedObj[i].BBox = trackedObj[i].tracker.update(crop);
                trackedObj[i].state = Lost; // Default is no COR match
                // Compare with COR
                for (int j = 0; j < candidates.size(); j++) {
                    Rect2d CORBox = candidates[j].BBox;
                    Rect2d TOBox = trackedObj[i].BBox;
                    double overlapArea = findOverlap(CORBox,TOBox);
                    double CORenclosed = findPercentIntersect(CORBox, TOBox);
                    double TOenclosed = findPercentIntersect(TOBox, CORBox);
                    if (overlapArea > MIN_OVERLAP || CORenclosed > COR_ENCLOSED || TOenclosed > TO_ENCLOSED) {
                        trackedObj[i].state = Tracked;
                        trackedObj[i].matchedCOR = j;
                        candidates[j].matchedTO.push_back(i);
                        if (candidates[j].matchedTO.size() > 1) {
                            if (candidates[j].matchedTO.size() == 2) {
                                // Mark first entry occluded as well
                                trackedObj[candidates[j].matchedTO[0]].state = Occluded;
                            }
                            trackedObj[i].state = Occluded;
                        }
                    }
                }
            }
            for (int i = 0; i < trackedObj.size(); i++) {
                // Go through each case
                switch (trackedObj[i].state){
                    case Tracked: {
                        int idx = trackedObj[i].matchedCOR;
                        double proportion = trackedObj[i].BBox.area()/candidates[idx].BBox.area();
                        // if the KCF box is much bigger or smaller than the COR, use the COR box and reinitialize the tracker
                        if (proportion < TOL || proportion > TOH) {
                            trackedObj[i].BBox = candidates[idx].BBox;
                            trackedObj[i].tracker.init(trackedObj[i].BBox, crop);
                            //trackedObj[i].BBox = trackedObj[i].tracker.update(crop);
                        }
                        trackedObj[i].counterOCC = 0;
                        trackedObj[i].counterLOST = 0;
                        break;
                    }
                    case Occluded: {
                        // Will be handled by next COR loop
                        trackedObj[i].counterLOST = 0;
                        break;
                    }
                    case Lost: {
                        // Remove the tracked object only if tracker has also failed
//                        if (!trackSuccess) {
//                            trackedObj.erase(trackedObj.begin() + i);
//                        }

                        trackedObj[i].counterLOST++;
                        
                        break;
                    }
                }
                
            }
            // Check if any COR did not match with a TO or has multiple matches
            for (int i = 0; i < candidates.size(); i++) {
                // If no TO match, create new target
                if (candidates[i].matchedTO.size() == 0) {
                    TO newTracked;
                    newTracked.BBox = candidates[i].BBox;
//                    newTracked.tracker = TrackerKCF::create();
//                    newTracked.tracker->init(crop, newTracked.BBox);
                    newTracked.tracker.init(newTracked.BBox,crop);
                    newTracked.state = Tracked;
                    newTracked.matchedCOR = i;
                    newTracked.counterOCC = 0;
                    newTracked.counterOL = 0;
                    newTracked.counterLOST = 0;
                    trackedObj.push_back(newTracked);
                }
                // If COR matches with multiple TO, there is occlusion
                else if (candidates[i].matchedTO.size() > 1) {
                    double totalArea = 0.0;
                    for (int j = 0; j < candidates[i].matchedTO.size(); j++) {
                        totalArea += trackedObj[candidates[i].matchedTO[j]].BBox.area();
                    }
                    // Increment occlusion counter for secondary KCF trackers if total area of TO is greater than COR
                    if (totalArea > candidates[i].BBox.area()){
                        int primaryTO = candidates[i].matchedTO[0];
                        for (int j = 1; j < candidates[i].matchedTO.size(); j++) {
                            int indTO = candidates[i].matchedTO[j];
                            int matchedInd = 0;
                            // Check if this secondary TO has overlapped with primary TO before
                            bool occBefore = false;
                            for (int k = 0; k < trackedObj[primaryTO].occludeTO.size();k++){
                                if (trackedObj[primaryTO].occludeTO[k] == indTO) {
                                    occBefore = true;
                                    matchedInd = k;
                                    break;
                                }
                            }
                            if (occBefore) {
                                trackedObj[indTO].counterOCC++; // increment occlusion counter
                                if (trackedObj[indTO].counterOCC > CONS_FRAMES) {
                                    // Secondary TO will be deleted. Delete secondary TO from occludeTO and re-initialize primary TO
                                    trackedObj[primaryTO].occludeTO.erase(trackedObj[primaryTO].occludeTO.begin() + matchedInd);
                                    trackedObj[primaryTO].tracker.init(candidates[i].BBox, crop);
                                }
                            } else {
                                trackedObj[primaryTO].occludeTO.push_back(indTO); // add this TO to the list of occluded TOs
                                trackedObj[indTO].counterOCC = 1;
                            }
                            
                        }
                    } else { // If not tracking same object, reset everyone
                        for (int j = 0; j < candidates[i].matchedTO.size(); j++) {
                            int indTO = candidates[i].matchedTO[j];
                            trackedObj[indTO].counterOCC = 0;
                            trackedObj[indTO].occludeTO.clear();
                        }
                    }
                }
            }
//            // Find the TOs that overlap with the original TO
//            for (int i = 0; i < trackedObj.size(); i++) {
//                for (int j = i+1; j < trackedObj.size(); j++){
//                    Rect2d A = trackedObj[i].BBox;
//                    Rect2d B = trackedObj[j].BBox;
//                    // Check if there is overlap or close to each other
//                    if ((A & B).area() > 0 || (norm(findCenter(A) - findCenter(B)) < MIN_SEP)) {
//                        // Check if the TO has already overlapped before
//                        bool existingOverlap = false;
//                        for (int k = 0; k < trackedObj[i].overlapTO.size(); k++) {
//                            if (trackedObj[i].overlapTO[k] == j){
//                                trackedObj[j].counterOL++; // increment overlap counter
//                                // if this pushes the counter over, delete this TO from the overlapTO queue
//                                if (trackedObj[j].counterOL > CONS_FRAMES) {
//                                    trackedObj[i].overlapTO.erase(trackedObj[i].overlapTO.begin()+k);
//                                }
//                                existingOverlap = true;
//                                break;
//                            }
//                        }
//                        if (!existingOverlap){
//                            trackedObj[i].overlapTO.push_back(j);
//                        }
//                    } else {
//                        // Remove the TO from overlap list if its there
//                        for (int k = 0; k < trackedObj[i].overlapTO.size(); k++) {
//                            if (trackedObj[i].overlapTO[k] == j){
//                                trackedObj[i].overlapTO.erase(trackedObj[i].overlapTO.begin() + k); // remove the TO from the list
//                                trackedObj[j].counterOL = 0; // reset overlap counter
//                                break;
//                            }
//                        }
//                    }
//                }
//            }
            
            //Delete KCF trackers that have overlapped or have been stationary for consecutive frames
            for (vector<TO>::iterator it = trackedObj.begin(); it != trackedObj.end();) {
                if (it->counterOCC > CONS_FRAMES || it->counterOL > OVERLAP_FRAMES || it->counterLOST > LOST_FRAMES) {
                    it = trackedObj.erase(it);
                } else {
                    ++it;
                }
            }
        }
        
//        if (!KCFTracking && BSTracker.largestArea > 3000 && BSTracker.largestArea < (350*350)) {
//            KCFBox = BSBox;
//            KCFTracker->init(crop, KCFBox);
//            KCFTracking = true;
//        } else if (KCFTracking) {
//            KCFTracker->update(crop, KCFBox);
//        } else {
//            ;
//        }
        
        // Kalman Filter
//        u_estimate = (float) (BSBox.x + BSBox.width/2);
//        v_estimate = (float) (BSBox.y + BSBox.height/2);
//        kalmanTracker.KalmanTracking(&u_estimate, &v_estimate, &u_predict, &v_predict);
//        KFBox.x = (int) u_estimate - BSBox.width/2;
//        KFBox.y = (int) v_estimate - BSBox.height/2;
//        KFBox.width = BSBox.width;
//        KFBox.height = BSBox.height;

//        if (kalmanTracker.KalmanTracking(&u_estimate, &v_estimate, &u_predict, &v_predict)) {
//                // If the KF filter is not lost, update KFBox
//                KFBox.x = (int) (u_estimate - BSBox.width/2);
//                KFBox.y = (int) (v_estimate - BSBox.height/2);
//                KFBox.width = BSBox.width;
//                KFBox.height = BSBox.height;
//        } else {
//                // If lost, keep previous box
//                KFBox = prevKFBox; 
//        }

//        faceTracker.detectFace(crop);



//        for (int i = 0; i < BSTracker.boundingBoxes.size(); i++) {
//            Rect qtBox = BSTracker.boundingBoxes[i];

//        for (int i = 0; i < faceTracker.faceBoxes.size(); i++) {
//            Rect qtBox = faceTracker.faceBoxes[i];
            //rectangle(crop, qtBox, Scalar(0,255,0),3);
        
        Mat finalStructure = Mat::zeros(512, 512, CV_8UC1);
        //Mat finalStructure = Mat::zeros(rows, cols, CV_8UC1);
        for (int i = 0; i < trackedObj.size(); i++){
            rectangle(finalStructure, trackedObj[i].BBox, Scalar(255),-1);
        }
                      
        int newBytes = 262144;
        qt_decomp(&crop, &finalStructure, frameRect, &newBytes, 10);
        
        //prevKFBox = KFBox;
        double duration = (clock() - start)/ (double)CLOCKS_PER_SEC;
        double fps = 1/duration;
        
        char str[200];
        sprintf(str,"QT:ORG = %f", double(newBytes)/262144.0);
        putText(crop, str, Point2f(10,20), FONT_HERSHEY_PLAIN, 1,  Scalar(0,0,255,255));
//
        sprintf(str,"FPS = %f", fps);
        putText(crop, str, Point2f(10,35), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255,255));
//
        //Draw KCF boxes in Green
        for (int i = 0; i < trackedObj.size(); i++) {
            rectangle(crop, trackedObj[i].BBox, Scalar(0,255,0),2);
        }
        //Draw BS boxes in Red
        for (int i = 0; i < candidates.size(); i++){
            rectangle(crop, candidates[i].BBox, Scalar(0,0,255),1);
        }
        
        Mat OrgAndTest = Mat::zeros(512, 512*2+10, CV_8UC3);
        Rect org, test;
        org.x = 0;
        org.y = 0;
        org.width = 512;
        org.height = 512;
        test.x = 522;
        test.y = 0;
        test.height = 512;
        test.width = 512;
        
        bitwise_or(cropOrg, OrgAndTest(org), OrgAndTest(org));
        bitwise_or(crop, OrgAndTest(test), OrgAndTest(test));
//
//        char str[200];
//        for (int i = 0; i < trackedObj.size(); i++) {
//            Rect2d box = trackedObj[i].BBox;
//            sprintf(str, "%d,%d,%f,%f,%f,%f,-1,-1,-1,-1\n", imgInd, i, box.x, box.y, box.width, box.height);
//            res << str;
//        }
        imshow("Test",OrgAndTest);
//        char filename[200];
//        sprintf(filename, "Img%d.jpg",imgInd);
//        imwrite(filename, OrgAndTest);
        if (waitKey(30) >= 0) {break;}
    }
    
//    res.close();
    return 0;
}


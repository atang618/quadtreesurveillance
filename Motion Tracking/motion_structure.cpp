//
//  motion_structure.cpp
//  Motion Tracking
//
//  Created by Henry Chopp on 3/6/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#include "motion_structure.hpp"

MotionStruct::MotionStruct() {
	this->prevStruct = Mat::zeros(512, 512, CV_8UC1);
	this->currStruct = Mat::zeros(512, 512, CV_8UC1);
	
	SimpleBlobDetector::Params params;
	params.filterByArea = false;
	params.filterByColor = false;
	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByCircularity = false;
	this->detector = *SimpleBlobDetector::create(params);
}

void MotionStruct::update(Mat structure) {
	this->prevStruct = currStruct.clone();
	this->currStruct = structure.clone();
}

Mat MotionStruct::compareStructs() {
	Mat labels, stats, centroids;
	int nlabels = connectedComponentsWithStats(prevStruct, labels, stats, centroids, 4, CV_32S);
	for (int i = 1; i < nlabels; i++) {
		Rect cc;
		cc.x = stats.at<int>(i, CC_STAT_LEFT);
		cc.y = stats.at<int>(i, CC_STAT_TOP);
		cc.width = stats.at<int>(i, CC_STAT_WIDTH);
		cc.height = stats.at<int>(i, CC_STAT_HEIGHT);
		
		Mat andMat, xandMat;
		bitwise_and(prevStruct(cc), currStruct(cc), andMat);
		bitwise_xor(andMat, currStruct(cc), xandMat);
		
		if (countNonZero(xandMat) <= 1000) {
			prevStruct(cc).copyTo(currStruct(cc));
		}
		imshow("",xandMat);
	}
	return currStruct;
}

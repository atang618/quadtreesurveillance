//
//  qt_decompose.cpp
//  QT_Track
//
//  Created by Henry Chopp on 3/1/18.
//  Copyright © 2018 Henry Chopp. All rights reserved.
//

#include "qt_decompose.hpp"

void qt_decomp(Mat* im, Mat* structure, Rect portion, int* byteCount, double stdThresh) {
	
	if (portion.width > 1) {
		Scalar mean1, stddev1;
		meanStdDev((*im)(portion), mean1, stddev1);
		double stdavg = (stddev1[0] + stddev1[1] + stddev1[2])/3.0;
		if (countNonZero((*structure)(portion)) == 0 && stdavg < stdThresh) {
			float avg1 = mean((*im)(portion))[0];
			float avg2 = mean((*im)(portion))[1];
			float avg3 = mean((*im)(portion))[2];
			
			vector<Point> points;
			points.push_back(Point(portion.x, portion.y));
			points.push_back(Point(portion.x + portion.width, portion.y));
			points.push_back(Point(portion.x + portion.width, portion.y + portion.height));
			points.push_back(Point(portion.x, portion.y + portion.height));
			
			fillConvexPoly(*im, points, Scalar(avg1, avg2, avg3));
			*byteCount = *byteCount - portion.area() + 1;
		}
		else {
			Rect tempRect;
			tempRect.x = portion.x;
			tempRect.y = portion.y;
			tempRect.width = portion.width/2;
			tempRect.height = portion.height/2;
			qt_decomp(im, structure, tempRect, byteCount, stdThresh);
			
			tempRect.x = portion.x + portion.width/2;
			qt_decomp(im, structure, tempRect, byteCount, stdThresh);
			
			tempRect.x = portion.x;
			tempRect.y = portion.y + portion.height/2;
			qt_decomp(im, structure, tempRect, byteCount, stdThresh);
			
			tempRect.x = portion.x + portion.width/2;
			qt_decomp(im, structure, tempRect, byteCount, stdThresh);
		}
	}
}


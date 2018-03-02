//
//  kalman.hpp
//  Motion Tracking
//
//  Created by Allen Tang on 2/27/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#ifndef kalman_hpp
#define kalman_hpp

#include <memory>
#include <deque>
#include <stdio.h>
#include <opencv2/video/tracking.hpp>


using namespace cv;
using namespace std;

class TKalmanFilter {
    KalmanFilter kalman;
    Mat_<float> measurement;
    bool acceleration;
    
public:
    TKalmanFilter(float InitPositionU,
                  float InitPositionV,
                  bool accel = false);
    void KalmanTracking(float *PositionU_in_estimate,
                        float *PositionV_in_estimate,
                        float *PositionU_predict,
                        float *PositionV_predict);
    
};

#endif /* kalman_hpp */

//
//  kalman.cpp
//  Motion Tracking
//
//  Created by Allen Tang on 2/27/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#include "kalman.hpp"

TKalmanFilter::TKalmanFilter(float InitPositionU,
                             float InitPositionV,
                             float maxJump,
                             bool accel
                             )
: acceleration(accel), measurement(2,1), kalman(accel ? 6 : 4, 2, 0), maxDist(maxJump), lastU(InitPositionU), lastV(InitPositionV)
{
    if (acceleration) {
        // Considering acceleration
        // 6 dynamic parameters (u,v position and du, dv velocity and d2u, d2v acceleration) and 2 measurement parameters (u,v position)
        // State vector of six components
        // X = [x, y, v_x, v_y, a_x, a_y]T
        // Next state
        // x|t+1 = x|t + v_x|t + 0.5 a_x|t
        // y|t+1 = y|t + v_y|t + 0.5 a_y|t
        // v_x|t+1 = v_x|t + a_x|t
        // v_y|t+1 = v_t|t + a_t|t
        // a_x|t+1 = a_x|t					-> constant acceleration
        // a_y|t+1 = a_y|t
        
        kalman.transitionMatrix =  (Mat_<float>(6,6) << 1,0,1,0,0.5,0,   0,1,0,1,0,0.5,   0,0,1,0,1,0,   0,0,0,1,0,1,   0,0,0,0,1,0,   0,0,0,0,0,1); // Transition Matrix
        measurement.setTo(Scalar(0));
        kalman.statePre.at<float>(0) = InitPositionU;
        kalman.statePre.at<float>(1) = InitPositionV;
        kalman.statePre.at<float>(2) = 0;
        kalman.statePre.at<float>(3) = 0;
        kalman.statePre.at<float>(4) = 0;
        kalman.statePre.at<float>(5) = 0;
        setIdentity(kalman.measurementMatrix);
        setIdentity(kalman.processNoiseCov, Scalar::all(1e-4));
        setIdentity(kalman.measurementNoiseCov, Scalar::all(0.01));
        setIdentity(kalman.errorCovPost, Scalar::all(10));
    } else {
        // Considering velocity
        // 4 dynamic parameters (u,v position and du, dv velocity) and 2 measurement parameters (u,v position)
        // State vector of four components
        // X = [x, y, v_x, v_y]T
        // Next state
        // x|t+1 = x|t + v_x|t
        // y|t+1 = y|t + v_y|t
        // v_x|t+1 = v_x|t					-> constant velocity
        // v_y|t+1 = v_t|t
        kalman.transitionMatrix = (Mat_<float>(4,4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1); // Transition Matrix
        measurement.setTo(Scalar(0));
        kalman.statePre.at<float>(0) = InitPositionU;
        kalman.statePre.at<float>(1) = InitPositionV;
        kalman.statePre.at<float>(2) = 0;
        kalman.statePre.at<float>(3) = 0;
        setIdentity(kalman.measurementMatrix);
        setIdentity(kalman.processNoiseCov, Scalar::all(1e-4));
        setIdentity(kalman.measurementNoiseCov, Scalar::all(1e-2));
        setIdentity(kalman.errorCovPost, Scalar::all(1));
    }
    
}

int TKalmanFilter::KalmanTracking(float *PositionU_in_estimate,
                                   float *PositionV_in_estimate,
                                   float *PositionU_predict,
                                   float *PositionV_predict)
{
    bool lost = false;
    Mat predict = kalman.predict(); // Kalman prediction
    (*PositionU_predict) = predict.at<float>(0); // Copy the predicted values
    (*PositionV_predict) = predict.at<float>(1);
    
    measurement(0) = (*PositionU_in_estimate); // Measurement
    measurement(1) = (*PositionV_in_estimate);
    
    Mat estimate = kalman.correct(measurement); // Update the predicted state
    
    (*PositionU_in_estimate) = estimate.at<float>(0); // Copy the estimated values
    (*PositionV_in_estimate) = estimate.at<float>(1);
    
    // check if Kalman Filter is lost
    if (abs((*PositionU_in_estimate)-lastU) > maxDist ||
        abs((*PositionV_in_estimate)-lastV) > maxDist){
        lost = true;
    }
    
    if (lost) {
        return 0;
    } else {
        lastU = (*PositionU_in_estimate);
        lastV = (*PositionV_in_estimate);
        return 1;
    }
}

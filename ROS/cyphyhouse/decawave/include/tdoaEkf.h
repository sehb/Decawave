/*************************************************
 *  
 *  Class for computing 3D position from Time-Difference of Arrival information.
 *  Position is computed using an Extended Kalman Filter.
 *
 *  
 *
 *  Created on: 06/19/2017
 *  Author: Joao Paulo Jansch Porto <janschp2(at)illinois.edu>
 *
 *  Changelog:
 *      v0.3 - Added extra library functions (06/26/2017)
 *      v0.2 - initial release (06/19/2017)
 *
 *************************************************/

#ifndef _TDOA_EKF_h
#define _TDOA_EKF_h

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

#include "definitions.h"

#define MAX_COVARIANCE 100
#define MIN_COVARIANCE 1e-6f


class TDOAEkf
{
public:
    
    // Contructor
    TDOAEkf();
    TDOAEkf(Eigen::MatrixXf transition_mat, Eigen::MatrixXf prediction_mat, Eigen::MatrixXf covariance_mat, vec3d_t init_pos);
    
    // Set Functions
    void setTransitionMat(Eigen::MatrixXf transition_mat);
    void setPredictionMat(Eigen::MatrixXf prediction_mat);
    void setCovarianceMat(Eigen::MatrixXf covariance_mat);
    
    void setAncPosition(int anc_num, vec3d_t anc_pos);
    void setAncPosition(int anc_num, float x, float y, float z);
    
    void setStdDev(float sdev);
    
    // Update functions
    void scalarTDOADistUpdate(uint8_t Ar, uint8_t An, float distanceDiff);
    void stateEstimatorPredict(double dt);
    void stateEstimatorFinalize();
    void stateEstimatorAddProcessNoise();
    
    
    // Get functions
    vec3d_t getLocation();
	vec3d_t getVelocity();
    vec3d_t getAncPosition(int anc_num);
    
private:
    
    //variables
    uint32_t tdoaCount;
    uint32_t nr_states;
    float stdDev;
    
    vec3d_t anchorPosition[MAX_NR_ANCHORS];
    
    // Matrices used by the kalman filter
    Eigen::VectorXf S;
    Eigen::MatrixXf P;
    Eigen::MatrixXf A;
    Eigen::MatrixXf Q;
    
    //Functions
    void stateEstimatorScalarUpdate(Eigen::RowVectorXf H, float error, float stdMeasNoise);
    
    void PredictionBound();

};

#endif


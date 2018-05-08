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

#ifndef _TDOA_h
#define _TDOA_h

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/QR>

#define STATE_X   0
#define STATE_Y   1
#define STATE_Z   2
#define STATE_DIM 3

#define MAX_NR_ANCHORS 8


typedef struct vec3d_s
{
    float   x;
    float   y;
    float   z;
}vec3d_t;

class TDOA
{
public:
    
    // Contructor
    TDOA(vec3d_t init_pos);
    
    // Set Functions
    void setAncPosition(int anc_num, float x, float y, float z);
    void setAncPosition(int anc_num, vec3d_t anc_pos);

    void setAncDiff();

    void storeTdoaData(uint8_t Ar, uint8_t An, float distanceDiff);

    // Update functions
    void updateF(void);
    void updateD(void);
    void updateJ(void);
    void updateS(void);

    
    // Get functions
    vec3d_t getLocation();
    vec3d_t getVelocity();
    vec3d_t getAncPosition(int anc_num);
    
private:
    
    //variables
    vec3d_t anchorPosition[MAX_NR_ANCHORS];
    float anchorDiff[MAX_NR_ANCHORS];
    float tdoa_data[MAX_NR_ANCHORS];
    
    // Matrices
    Eigen::VectorXf S_prev;
    Eigen::VectorXf S_curr;
    Eigen::VectorXf D; // distance bewteen tag and each anchors
    Eigen::VectorXf F; // f(r) where r = [r_0,prev, ..., r_7,prev]
    Eigen::MatrixXf J; // Jacobian
    Eigen::MatrixXf J_inv;

    // Matrix funcs
    void pinv(Eigen::MatrixXf& pinvmat) const;

};

#endif


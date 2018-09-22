/*************************************************
 *  
 *  Class for computing 3D position from Time-Difference of Arrival information.
 *  Position is computed using an Newton's Method
 *
 *  
 *
 *  Created on: 06/19/2017
 *  Author: Joao Paulo Jansch Porto <janschp2(at)illinois.edu>
 *  Modified: Eric Yeh
 *
 *  Changelog:
 *      v0.3 - Added extra library functions (06/26/2017)
 *      v0.2 - initial release (06/19/2017)
 *
 *************************************************/


#include "tdoaNewton.h"

TDOANewton::TDOANewton(vec3d_t init_pos)
{
    S_prev.setZero(STATE_DIM_NEWTON);
    S_curr.setZero(STATE_DIM_NEWTON);
    D.setZero(MAX_NR_ANCHORS);
    J.setZero(MAX_NR_ANCHORS, STATE_DIM_NEWTON);
    F.setZero(MAX_NR_ANCHORS);
}

void TDOANewton::setAncPosition(int anc_num, float x, float y, float z)
{
    vec3d_t temp;
    temp.x = x;
    temp.y = y;
    temp.z = z;
    setAncPosition(anc_num, temp);
    std::cout << "tdoa.cpp: Anchor position set" << std::endl;
}

void TDOANewton::setAncPosition(int anc_num, vec3d_t anc_pos)
{
    if( (anc_num < 0) || (anc_num > MAX_NR_ANCHORS) )
    {
        //invalid anchor number
        return;
    }

    anchorPosition[anc_num] = anc_pos;
}

void TDOANewton::setAncDiff()
{
    // anchorDiff[i] = diff between anchorPosition[i] and anchorPosition[i+1]
    for (int i = 0; i < MAX_NR_ANCHORS; i++) {
        int j = i+1;
        if (j == 8)
            j = 0;

        anchorDiff[i] = sqrtf(
            powf(anchorPosition[i].x - anchorPosition[j].x, 2) + 
            powf(anchorPosition[i].y - anchorPosition[j].y, 2) + 
            powf(anchorPosition[i].z - anchorPosition[j].z, 2));
    }
    std::cout << "tdoa.cpp: Anchor difference set" << std::endl;
}

void TDOANewton::storeTdoaData(uint8_t Ar, uint8_t An, float distanceDiff)
{
    if (Ar==An-1 || (Ar==7 && An==0)) {
        //std::cout << "update data:: Ar:" << std::to_string(Ar) << ", An:" <<
        //    std::to_string(An) << ", DistDiff:" << std::to_string(distanceDiff) << std::endl;

        tdoa_data[Ar] = distanceDiff;
    }
}

void TDOANewton::updateF(void)
{
    // std::cout << "tdoa.cpp: @updateF" << std::endl;
    // update F
    for (int i=0; i < MAX_NR_ANCHORS; i++){
        int j = i+1;
        if (j==8)
            j = 0;

        float dist_Ar = sqrtf(powf(S_prev[STATE_X]-anchorPosition[i].x, 2) + 
                              powf(S_prev[STATE_Y]-anchorPosition[i].y, 2) +
                              powf(S_prev[STATE_Z]-anchorPosition[i].z, 2));
        float dist_An = sqrtf(powf(S_prev[STATE_X]-anchorPosition[j].x, 2) + 
                              powf(S_prev[STATE_Y]-anchorPosition[j].y, 2) +
                              powf(S_prev[STATE_Z]-anchorPosition[j].z, 2));
        F[i] = anchorDiff[i] - dist_Ar + dist_An - tdoa_data[i];
    }
}


void TDOANewton::updateD(void)
{
    // update D
    for (int i = 0; i < MAX_NR_ANCHORS; i++) {
        D[i] = sqrtf(
            powf(S_prev[STATE_X] - anchorPosition[i].x, 2) + 
            powf(S_prev[STATE_Y] - anchorPosition[i].y, 2) + 
            powf(S_prev[STATE_Z] - anchorPosition[i].z, 2));
    }
}

void TDOANewton::updateJ(void)
{
    // update J

    float x = S_prev[STATE_X];
    float y = S_prev[STATE_Y];
    float z = S_prev[STATE_Z];

    for (int i = 0; i < MAX_NR_ANCHORS; i++) {
        int j = i + 1;
        if (j == 8)
            j = 0;

        float d0 = anchorDiff[i];
        float d1 = anchorDiff[j];

        vec3d_t ancPos_i = anchorPosition[i];
        vec3d_t ancPos_j = anchorPosition[j];

        J(i, STATE_X) = - (x-ancPos_i.x)/d0 + (x-ancPos_j.x)/d1;
        J(i, STATE_Y) = - (y-ancPos_i.y)/d0 + (y-ancPos_j.y)/d1;
        J(i, STATE_Z) = - (z-ancPos_i.z)/d0 + (z-ancPos_j.z)/d1;
    }
}

void TDOANewton::updateS(void)
{
    // assume received all 7 distanceDiff

    // r_cur = r_prev + delta_r
    // delta_r = -J^{-1} + F

    // for each cycle:
    // delta_r needs: J and F
    // J needs: D

    //std::cout << "tdoa.cpp: @updateS" << std::endl;
    int iter = 4;
    //float F_norm = 100;

    while (iter > 0){
        updateF();
        updateD();
        updateJ();
        //(A^t * A)^-1 * A^t
        Eigen::MatrixXf J_inv = (J.transpose() * J).inverse() * J.transpose();
        S_curr = S_prev - (J_inv * F);
        S_prev = S_curr;

        //F_norm = F.norm();
        //std::cout << "iter: " << std::to_string(iter) << " F_norm: " << std::to_string(F_norm) << std::endl;

        iter--;
    }
}


vec3d_t TDOANewton::getAncPosition(int anc_num) { return anchorPosition[anc_num]; }

vec3d_t TDOANewton::getLocation(void)
{
    vec3d_t pos;
    pos.x = S_curr(STATE_X);
    pos.y = S_curr(STATE_Y);
    pos.z = S_curr(STATE_Z);
    
    // std::cout << S_curr.transpose() << std::endl;
    return pos;
}

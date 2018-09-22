#ifndef _DEF_h
#define _DEF_h

#define STATE_X   0
#define STATE_Y   1
#define STATE_Z   2
#define STATE_VX  3
#define STATE_VY  4
#define STATE_VZ  5
#define STATE_DIM_EKF 6 //TODO: this is 3 for Newtons
#define STATE_DIM_NEWTON 3

#define MAX_NR_ANCHORS 8

typedef struct vec3d_s
{
    float   x;
    float   y;
    float   z;
}vec3d_t;


#endif
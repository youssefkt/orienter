#include "y_orienter_utils.h"


void eulerZYXToFixXYZ(float* euler, float* fix)
{
    fix[0] = euler[2];
    fix[1] = euler[1];
    fix[2] = euler[0];
}

void fixXYZToEulerZYX(float* fix, float* euler)
{
    euler[0] = fix[2];
    euler[1] = fix[1];
    euler[2] = fix[0];
}

void matrixtoXYZ(float* R, float* roll,float* pitch, float* yaw){

    *pitch = atan2(-1*R[6],sqrt(R[0]*R[0]+R[3]*R[3]));
    if(*pitch>Y_PI)*pitch-=Y_2PI;
    if(*pitch<-Y_PI)*pitch+=Y_2PI;

    *roll = atan2(R[7],R[8]);
    if(*roll>Y_PI)*roll-=Y_2PI;
    if(*roll<-Y_PI)*roll+=Y_2PI;

    *yaw = atan2(R[3],R[0]);
    if(*yaw>Y_PI)*yaw-=Y_2PI;
    if(*yaw<-Y_PI)*yaw+=Y_2PI;

}

void matrixtoQuat(float* R, float* q){


    q[0] = 0.5*sqrt(1+R[0]+R[4]+R[8]);

    if(q[0] == 0) q[0] += 0.0001;

    float q_1 = 0.25/q[0];

    q[1] = (R[7]-R[5])*q_1;
    q[2] = (R[2]-R[6])*q_1;
    q[3] = (R[3]-R[1])*q_1;

    float norme = sqrt(quat[0]*quat[0]+quat[1]*quat[1]+quat[2]*quat[2]+quat[3]*quat[3]);
    float norme_1 = 1./norme;

    q[0] *= norme_1;
    q[1] *= norme_1;
    q[2] *= norme_1;
    q[3] *= norme_1;

}

void quatToMatrix(float* q, float* R){

    float norme = sqrt(q[0]*q[0]+q[1]*quat[1]+quat[2]*quat[2]+quat[3]*quat[3]);
    float norme_1 = 1./norme;

    q[0] *= norme_1;
    q[1] *= norme_1;
    q[2] *= norme_1;
    q[3] *= norme_1;

    R[0] = (1-2*(q[2]*q[2]+q[3]*q[3]));
    R[1] = 2*(q[1]*q[2]-q[0]*q[3]);
    R[2] = 2*(q[1]*q[3]+q[0]*q[2]);
    R[3] = 2*(q[1]*q[2]+q[0]*q[3]);
    R[4] = (1-2*(q[1]*q[1]+q[3]*q[3]));
    R[5] = 2*(q[2]*q[3]-q[0]*q[1]);
    R[6] = 2*(q[1]*q[3]-q[0]*q[2]);
    R[7] = 2*(q[2]*q[3]+q[0]*q[1]);
    R[8] = (1-2*(q[1]*q[1]+q[2]*q[2]));

}

void normalize(float* a, int size){

    float norme = 0;
    for(int i=0;i<size;i++)
        norme+=a[i]*a[i];

    float norme_1 = 1./norme;

    for(int i=0;i<size;i++)
        a[i]*=norme_1;

}

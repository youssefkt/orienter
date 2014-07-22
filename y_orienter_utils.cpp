#include "y_orienter_utils.h"


void fixToEuler(float* fix, float* euler)
{
    euler[0] = fix[2];
    euler[1] = fix[1];
    euler[2] = fix[0];
}


void eulerToFix(float* euler, float* fix)
{
    fix[0] = euler[2];
    fix[1] = euler[1];
    fix[2] = euler[0];
}

void matrixToFix(float* R, float* f){

    f[1] = atan2(-1*R[6],sqrt(R[0]*R[0]+R[3]*R[3]));
    if(f[1]>Y_PI)f[1]-=Y_2PI;
    if(f[1]<-Y_PI)f[1]+=Y_2PI;

    f[0] = atan2(R[7],R[8]);
    if(f[0]>Y_PI)f[0]-=Y_2PI;
    if(f[0]<-Y_PI)f[0]+=Y_2PI;

    f[2] = atan2(R[3],R[0]);
    if(f[2]>Y_PI)f[2]-=Y_2PI;
    if(f[2]<-Y_PI)f[2]+=Y_2PI;

}

void matrixToQuat(float* R, float* q){

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

void eulerToQuat(float* E, float* q){

    float e1 = E[0]*0.5;
    float e2 = E[1]*0.5;
    float e3 = E[2]*0.5;

    float c1 = cos(e1);float s1 = sin(e1);
    float c2 = cos(e2);float s2 = sin(e2);
    float c3 = cos(e3);float s3 = sin(e3);

    q[0] = c1*c2*c3 + s1*s2*s3;
    q[1] = s3*c1*c2 - s1*s2*c3;
    q[2] = c1*c3*s2 + s1*c2*s3;
    q[3] = s1*c2*c3 - c1*s2*s3;

}

void fixToQuat(float* F, float* q){

    float e1 = F[2]*0.5;
    float e2 = F[1]*0.5;
    float e3 = F[0]*0.5;

    float c1 = cos(e1);float s1 = sin(e1);
    float c2 = cos(e2);float s2 = sin(e2);
    float c3 = cos(e3);float s3 = sin(e3);

    q[0] = c1*c2*c3 + s1*s2*s3;
    q[1] = s3*c1*c2 - s1*s2*c3;
    q[2] = c1*c3*s2 + s1*c2*s3;
    q[3] = s1*c2*c3 - c1*s2*s3;

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

void eulerToMatrix(float* e, float* R){

    float c1 = cos(e[0]);float s1 = sin(e[0]);
    float c2 = cos(e[1]);float s2 = sin(e[1]);
    float c3 = cos(e[2]);float s3 = sin(e[2]);

    R[0] = c1*c2;
    R[1] = c1*s2*s3-s1*c3;
    R[2] = c1*s2*c3 + s1*s3;
    R[3] = s1*c2;
    R[4] = s1*s2*s3 + c1*c3;
    R[5] = s1*s2*c3 - c1*s3;
    R[6] = -s2;
    R[7] = c2*s3;
    R[8] = c2*c3;

}

void fixToMatrix(float* f, float* R){

    float c1 = cos(f[2]);float s1 = sin(f[2]);
    float c2 = cos(f[1]);float s2 = sin(f[1]);
    float c3 = cos(f[0]);float s3 = sin(f[0]);

    R[0] = c1*c2;
    R[1] = c1*s2*s3-s1*c3;
    R[2] = c1*s2*c3 + s1*s3;
    R[3] = s1*c2;
    R[4] = s1*s2*s3 + c1*c3;
    R[5] = s1*s2*c3 - c1*s3;
    R[6] = -s2;
    R[7] = c2*s3;
    R[8] = c2*c3;

}


void normalize(float* a, int size){

    float norme = 0;
    for(int i=0;i<size;i++)
        norme+=a[i]*a[i];

    float norme_1 = 1./norme;

    for(int i=0;i<size;i++)
        a[i]*=norme_1;

}

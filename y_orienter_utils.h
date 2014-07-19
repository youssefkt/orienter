//Youssef KTIRI, JSK 2012

#ifndef Y_ORIENTER_UTILs_H
#define Y_ORIENTER_UTILs_H


#include "y_orienter_general.h".h"


extern void eulerZYXToFixXYZ(float* euler, float* fix);

extern void fixXYZToEulerZYX(float* fix, float* euler);

extern void matrixtoXYZ(float* R, float& roll,float& pitch, float& yaw);

extern void matrixtoQuat(float* R, float* q);

extern void quatToMatrix(float* q, float* R);

extern void normalize(float* a, int size);







//template<class V>
//void normalizeData(V* a, int size, int type = 2)
//{
//    if(type==2){
//        V norme = (V)0;
//        for(int i=0;i<size;i++)
//            norme+=a[i]*a[i];
//        for(int i=0;i<size;i++)
//            a[i]=a[i]/norme;
//    }else if(type ==1){
//        V norme = (V)0;
//        for(int i=0;i<size;i++)
//            norme+=a[i]*a[i];
//        for(int i=0;i<size;i++)
//            a[i]=a[i]/norme;
//    }
//}

////ZXY to quaternion
//template <class V>
//void eulerZYXToQuat(V* euler, V* quat)
//{
//    V ca = cos(euler[0]/2);
//    V sa = sin(euler[0]/2);

//    V cb = cos(euler[1]/2);
//    V sb = sin(euler[1]/2);

//    V cg = cos(euler[2]/2);
//    V sg = sin(euler[2]/2);

//    quat[0] = cg*cb*ca - sg*sg*sa;
//    quat[1] = -sg*cb*ca - cg*sb*sa;
//    quat[2] = -cg*sb*ca + sg*cb*sa;
//    quat[3] = -cg*cb*sa - sg*sb*ca;
//}

//template <class V>
//void quatToEulerZYX(V* quat, V* euler)
//{
//    euler[0] = atan2(2*(quat[1]*quat[2] - quat[0]*quat[3]), 2*(quat[0]*quat[0]+quat[1]*quat[1]) - 1);
//    euler[1] = - atan2((2*(quat[1]*quat[3]+quat[0]*quat[2])), sqrt(1-4*(quat[1]*quat[3]+quat[0]*quat[2])*(quat[1]*quat[3]+quat[0]*quat[2])));
//    euler[2] = atan2(2*(quat[2]*quat[3]-quat[0]*quat[1]), 2*(quat[0]*quat[0]+quat[3]*quat[3])-1);
//}


#endif

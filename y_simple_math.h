#ifndef Y_SIMPLE_MATH_H
#define Y_SIMPLE_MATH_H



extern void matrixMul(float* A, float* B, float* C, int r, int m, int c);

extern void matrixTrs(float* A, float* At, int r, int c);

extern bool matrixInv44(const float m[16],  float invOut[16]);








#endif

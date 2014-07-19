/*!
 * @Author KTIRI Youssef, JSK Tokyo 2012
 * @file  visualiser.h
 * @brief visualize rpy
 *
 */

#ifndef VISUALISEr_H
#define VISUALISEr_H

//#include <GL/gl.h>
//#include <GL/glu.h>
#include <GL/glut.h>
#include <pthread.h>
#include "y_orienter.h"

//by defaut the cell width is 1 by 1
class OrienterVisu
{

public:

    void init();
    //void update();
    void run(pthread_mutex_t* _mu_visu, double* _roll, double* _pitch, double* _yaw, bool* isupdated, double* _x, double* _y, double* _z);
    //void reset();
    OrienterVisu(){}

protected:

    void initializeGL();
    //void resizeGL(int width, int height);
    //static void paintGL();
public:
    static void Timer(int extra);
    static void paintGL();
    static double* roll;
    static double* pitch;
    static double* yaw;
    static double* x;
    static double* y;
    static double* z;
    static bool* isupdated;
    static pthread_mutex_t* mu_visu;

};


#endif



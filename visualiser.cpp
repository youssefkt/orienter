#include "visualiser.h"


double* OrienterVisu::roll = NULL;
double* OrienterVisu::pitch = NULL;
double* OrienterVisu::yaw = NULL;
double* OrienterVisu::x = NULL;
double* OrienterVisu::y = NULL;
double* OrienterVisu::z = NULL;

bool* OrienterVisu::isupdated = NULL;
pthread_mutex_t* OrienterVisu::mu_visu = NULL;

void OrienterVisu::initializeGL()
{
    //glClearColor(0,0,0,0);
    glShadeModel(GL_FLAT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
}

void OrienterVisu::init(){

    int gg = 1;
    glutInit(&gg,NULL);
    glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB);
    glutInitWindowSize(400,400);
    glutInitWindowPosition(0,0);
    glutCreateWindow("orienter Visu");
    glutDisplayFunc(paintGL);
    glutTimerFunc(50,Timer,0);
    initializeGL();

}


void OrienterVisu::run(pthread_mutex_t* _mu_visu, double* _roll, double* _pitch, double* _yaw, bool* isupdated, double* _x, double* _y, double* _z){

    mu_visu = _mu_visu;
    this->isupdated = isupdated;
    roll = _roll;
    pitch = _pitch;
    yaw = _yaw;
    x = _x;
    y = _y;
    z = _z;
    glutMainLoop();

}


void OrienterVisu::Timer(int extra){

    glutPostRedisplay();
    glutTimerFunc(50,Timer,0);

}


void OrienterVisu::paintGL()
{

    cerr<<mu_visu<<" lock adsads "<<endl;
    pthread_mutex_lock(mu_visu);


    double _roll = *roll;
    double _pitch = *pitch;
    double _yaw = *yaw;

    pthread_mutex_unlock(mu_visu);
    cerr<<" unlock adsads "<<endl;
    //if(isupdated){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0,0,400,400);

    glColor3f(1.0, 1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    //glRotatef(90, 1.0, 0.0, 0.0);
    glRotatef(90, 0.0, 1.0, 0.0);
    glRotatef(-90, 1.0, 0.0, 0.0);

    //glTranslatef(*x,*y,*z);

    if(roll!=NULL){
        glRotatef(_roll, 1.0, 0.0, 0.0);
        glRotatef(_pitch, 0.0, 1.0, 0.0);
        glRotatef(_yaw, 0.0, 0.0, 1.0);
    }


    //pthread_mutex_unlock(mu_visu);

    glLineWidth(3.0);
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINES);
      glVertex3f(0.0, 0.0, 0.0);
      glVertex3f(0.8, 0.0, 0.0);
    glEnd();

    glColor3f(0.0, 1.0, 0.0);
    glBegin(GL_LINES);
      glVertex3f(0.0, 0.0, 0.0);
      glVertex3f(0.0, 0.8, 0.0);
    glEnd();

    glColor3f(0.0, 0.0, 1.0);
    glBegin(GL_LINES);
      glVertex3f(0.0, 0.0,0.0);
      glVertex3f(0.0, 0.0,0.8);
    glEnd();

//    //draw tetra
    glBegin(GL_TRIANGLES);

    glColor3f(1.0,0,0);
    glVertex3f(0.0,0.0,0.0);
    glVertex3f(0.3,0.0,0.0);
    glVertex3f(0.0,0.3,0.0);

    glColor3f(0.0,1.0,0.0);
    glVertex3f(0.0,0.0,0.0);
    glVertex3f(0.0,0.3,0.0);
    glVertex3f(0.0,0.0,0.3);

    glColor3f(0.0,0.0,1.0);
    glVertex3f(0.0,0.0,0.0);
    glVertex3f(0.0,0.0,0.3);
    glVertex3f(0.3,0.0,0.0);

    glColor3f(1.0,0.0,1.0);
    glVertex3f(0.3,0.0,0.0);
    glVertex3f(0.0,0.0,0.3);
    glVertex3f(0.3,0.0,0.0);

            //6 faces
    glEnd();

    glFlush();
    //}


}

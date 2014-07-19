#ifndef Y_ORIENTER_GENERAl_H
#define Y_ORIENTER_GENERAl_H

//csts
static const float Y_PI=3.14159;
static const float Y_PI2=1.57079;
static const float Y_2PI=6.28318;
static const float Y_TO_RAD = 0.01745;
static const float Y_TO_DEG = 57.2958;

//filter csts
const float YP_lp_freq_ = 100;
const float YP_filter_tau = 0.1;//we want 100ms

//const double Yfix_MagnField = 0.463689;//Tokyo
const float Yfix_MagnField = 0.4636;//morocco
const float Yfix_MagnInclination = 0.861;//inclination degree in rad = 40,56 degree
const float Yfix_GravField = 1.0;

//validity check
const float YP_Head_validity = 1.15;// if the norme is bigger than the mqgnfield intensity * validity then heading non valid
const float YP_Acc_validity = 1.05;

struct MARG_RAW_MESSAGE{

    MARG_RAW_MESSAGE(float ax, float ay, float az,
                               float gx, float gy,float gz,
                               float mx, float my,float mz,
                               float _dt){
        dt = _dt;
        accsx = ax;//body coordinate system x front and z up
        accsy = ay;
        accsz = az;
        gyrosx = gx;//
        gyrosy = gy;
        gyrosz = gz;
        magsx = mx;//body earth coordinate system x front and z down
        magsy = my;
        magsz = mz;
    }

    float dt;
    float accsx;
    float accsy;
    float accsz;
    float gyrosx;
    float gyrosy;
    float gyrosz;
    float magsx;
    float magsy;
    float magsz;
};


#endif


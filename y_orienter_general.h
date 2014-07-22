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
const float YP_Head_validity_1 = 1.0/YP_Head_validity;
const float YP_Acc_validity = 1.05;
const float YP_Acc_validity_1 = 1.0/YP_Acc_validity;

//nbr of samples
const int YP_nbr_mags = 30;
const int YP_nbr_accs = 100;
const int YP_nbr_gyros = 100;

struct MARG_DATA{

    float accs[3];
    float gyros[3];
    float mags[3];
    uint64_t dt;
    uint8_t flag;
    bool isupdated_gyros;
    bool isupdated_accs;
    bool isupdated_mags;

    MARG_DATA(){
        isupdated_gyros = false;
        isupdated_accs = false;
        isupdated_mags = false;
        flag = 0;
    }

};

struct MARG_DATA_0{

    MARG_DATA_0()
    {
        accs_s[0] = 1;accs_s[1] = 1;accs_s[2] = 1;
        gyros_s[0] = 1;gyros_s[1] = 1;gyros_s[2] = 1;
        mags_s[0] = 1;mags_s[1] = 1;mags_s[2] = 1;

        accs_0[0] = 0;accs_0[1] = 0;accs_0[2] = 0;
        gyros_0[0] = 0;gyros_0[1] = 0;gyros_0[2] = 0;
        mags_0[0] = 0;mags_0[1] = 0;mags_0[2] = 0;

        accs_n = 1;
        gyros_n = 1;
        mags_n = 1;

        accs_xyz[0] = 1;accs_xyz[1] = 1;accs_xyz[2] = 1;
        gyros_xyz[0] = 1;gyros_xyz[1] = 1;gyros_xyz[2] = 1;
        mags_xyz[0] = 1;mags_xyz[1] = 1;mags_xyz[2] = 1;

        accs_lp = 1.0/(1.0 + YP_lp_freq_*YP_filter_tau);
        gyros_lp = 1.0/(1.0 + YP_lp_freq_*YP_filter_tau*0.5);
        mags_lp = 1.0/(1.0 + YP_lp_freq_*3*YP_filter_tau);

        accs_lp_ = 1.0 - accs_lp;
        gyros_lp_ = 1.0 - gyros_lp;
        mags_lp_ = 1.0 - mags_lp;

        accs_nbr = YP_nbr_accs;
        gyros_nbr = YP_nbr_gyros;
        mags_nbr = YP_nbr_mags;

    }

    float accs_s[3];
    float gyros_s[3];
    float mags_s[3];

    float accs_0[3];
    float gyros_0[3];
    float mags_0[3];

    float accs_n;
    float gyros_n;
    float mags_n;

    float accs_xyz[3];
    float gyros_xyz[3];
    float mags_xyz[3];

    float accs_lp;
    float gyros_lp;
    float mags_lp;

    float accs_lp_;
    float gyros_lp_;
    float mags_lp_;

    int accs_nbr;
    int gyros_nbr;
    int mags_nbr;

    float rph_0[3];

};


#endif


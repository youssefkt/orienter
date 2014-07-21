//********************************************************//
/**
*9 Axis MARG Sensor orientation calculation based on quaternions
*@Author
*KTIRI Youssef, JSK Tokyo 2012
*@brief
*Extended Kalman fusion implementation for data fusion
*@input
*calibrated imu data in g/deg.s/uT
*@output
*rpy
*/
//********************************************************//

/*
 - the system works fine ine the normal range yaw +- 180, roll +- 90 and pitch +- 90
  in this range yaw = heading, out of the normal range for instance roll> 90
  the "heading"(actually its not but like the direction where the sensor points) becomes the picth
  !!!!!!! yaw != heading in
 - The initializing position is gonna become 0,0,0 but but for consistent displcement calculation
 we have to initialize on a nearly plane surface
  */

#ifndef Y_ORIENTEr_H
#define Y_ORIENTEr_H


//???#//Global////////////////////////////////////////////

#include "y_orienter_general.h"
#include "y_orienter_utils.h"


struct Orienter{

    public:

        bool init();
        void update();

    public:
          //*******Error model*******//
           //heading error model  mmmm no gauss markov ??
       float m0_norm;
       float head_0;
       bool isvalid_head;
       bool isvalid_acc;
       bool isupdated_marg;

private:
           //***********useful functions********//
       float calculatePitchAcc();
       float calculateRollAcc();
       float calculateHeading();
       void checkHeading();
       void checkAcc();

public:

           //**********ekf data*************//

           //entry
           float p,q,r,dt,ax,ay,az,mx,my,mz;
           float lp_coeff;//coeff = dt/(dt + rc)
           float lpm_coeff;
           float lp_ax,lp_ay,lp_az;
           float lp_mx,lp_my,lp_mz;

           //state
           VectorXd Xk;//dimX,1 q0 qv bp bq br
           MatrixXd Pk;//dimX,dimX
           VectorXd X0;//initial values
           MatrixXd P0;
           MatrixXd P0bis;

           //prediction
           void predict();
           void setAlpha(double _alpha){alpha=_alpha;}
           VectorXd Xk_;//dimX,1
           MatrixXd Pk_;//dimX,dimX
           MatrixXd Fk;//dimX,dimX
           MatrixXd Fkt;//dimX,dimX
           MatrixXd Qk;//dimX,dim
           MatrixXd Qmap;
           MatrixXd Qmapt;
           MatrixXd Qkinit;
           MatrixXd Omega;//given by quaternion formula
           double alpha,beta;//gains for tuning

           //observation update
           void observe();
           VectorXd Yk;//dimZ,1
           VectorXd Zk;//dimZ,1
           VectorXd Zk_;
           MatrixXd Sk;//
           MatrixXd Hk;
           MatrixXd Ro;//dimZ,dimZ
           MatrixXd Kk;//dimX,dimZ

           //storage to speed up, never access this
           Vector3d Vv;
           Vector3d Vrotated;
           Matrix3d Rx;
           Matrix3d Ry;
           Matrix3d Rz;
           Matrix3d R;
           MatrixXd ObsS;
           Matrix3d Rpredict;
           double roll,pitch,heading,norme;

           //displacement
           Vector3d Xdis;
           Vector3d Accs;

           //output
           Vector4d ResultQ;
           Vector3d ResultEuler;

           double res_roll,res_pitch,res_yaw;

           //for distance calc
           float deltax,deltay,deltaz;
           float vx_,vy_,vz_;
           float x,y,z;
           float sx,sy,sz;//sp = plane sensitivity ie sx and sy

           bool ismoving;
           bool isaccelerated;
           double t1,t2;

};


#endif

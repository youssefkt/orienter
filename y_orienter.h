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

       bool isupdated_acc;
       bool isupdated_gyro;
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

           float Xk[4];
           float Pk[16];

           //prediction
           void predict();
           float Xk_[4];//dimX,1
           float Pk_[16];//dimX,dimX

           float Fk[16];//dimX,dimX
           float Fkt[16];//dimX,dimX
           float Qk[16];//dimX,dim
           float Qmap[12];
           float Qmapt[12];
           float Qkinit[16];
           float Omega[16];//given by quaternion formula

           //observation update
           void observe();
           VectorXd Yk[4];//dimZ,1
           VectorXd Zk[4];//dimZ,1
           VectorXd Zk_[4];
           MatrixXd Sk[16];//
           MatrixXd Hk[16];
           MatrixXd Ro[16];//dimZ,dimZ
           MatrixXd Kk[16];//dimX,dimZ

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

           //output
           float ResultQ[4];
           float ResultEuler[3];
           float ResultFix[3];
           float ResultMatrix[16];


};


#endif

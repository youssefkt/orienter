#include "y_orienter.h"

Orienter::Orienter(){

    isinit = false;
    isupdated = false;

}

bool Orienter::init(){

    if(isupdated && !isinit){

        isupdated = false;

        if(!data_0.mags_nbr && !data_0.gyros_nbr && !data_0.accs_nbr){

            isinit = true;
            float acc_div =  1.0/((YP_nbr_accs>0)?YP_nbr_accs:1);
            float gyro_div =  1.0/((YP_nbr_gyros>0)?YP_nbr_gyros:1);
            float mag_div =  1.0/((YP_nbr_mags>0)?YP_nbr_mags:1);

            int i;
            for(i = 0; i<3;++i){
                data_0.accs_0[i]*=acc_div;
                data_0.gyros_0[i]*=gyro_div;
                data_0.mags_0[i]*=mag_div;
            }

            data_0.accs_n = sqrt(data_0.accs_0[0]*data_0.accs_0[0]+data_0.accs_0[1]*data_0.accs_0[1]+data_0.accs_0[2]*data_0.accs_0[2]);
            data_0.gyros_n = sqrt(data_0.gyros_0[0]*data_0.gyros_0[0]+data_0.gyros_0[1]*data_0.gyros_0[1]+data_0.gyros_0[2]*data_0.gyros_0[2]);
            data_0.mags_n = sqrt(data_0.mags_0[0]*data_0.mags_0[0]+data_0.mags_0[1]*data_0.mags_0[1]+data_0.mags_0[2]*data_0.mags_0[2]);

            for(i = 0; i<3; ++i){
                data.accs[i] = data_0.accs_xyz[i]*(data_0.accs_s[i]*data.accs[i]);
                data.gyros[i] = data_0.accs_xyz[i]*(data_0.gyros_s[i]*data.gyros[i] - data_0.gyros_0[i]);
                data.mags[i] = data_0.accs_xyz[i]*(data_0.mags_s[i]*data.mags[i]);
            }

            for(i =0 ;i<3; ++i){
                last_data.accs[i] = data.accs[i];
                last_data.gyros[i] = data.gyros[i];
                last_data.mags[i] = data.mags[i];
            }

            calculateRPH();

            for(i = 0; i<3;++i){
                data_0.rph_0[i] = rph[i];
            }

            //make X0 Pk0
            fixToQuat(rph,Xk);

            Pk[0] = 0.015; Pk[1] =  0.0; Pk[2] = 0.0; Pk[3] = 0.0;
            Pk[4] = 0.0; Pk[5] = 0.015; Pk[6] = 0.0; Pk[7] = 0.0;
            Pk[8] = 0.0; Pk[9] = 0.0; Pk[10] = 0.015; Pk[11] = 0.0;
            Pk[12] = 0.0; Pk[13] = 0.0; Pk[14] = 0.0; Pk[15] = 0.015;

            R0[0] = 0.015; R0[1] =  0.0; R0[2] = 0.0; R0[3] = 0.0;
            R0[4] = 0.0; R0[5] = 0.015; R0[6] = 0.0; R0[7] = 0.0;
            R0[8] = 0.0; R0[9] = 0.0; R0[10] = 0.015; R0[11] = 0.0;
            R0[12] = 0.0; R0[13] = 0.0; R0[14] = 0.0; R0[15] = 0.02;

            Q0[0] = 0.03; Q0[1] =  -0.01; Q0[2] = -0.01; Q0[3] = -0.01;
            Q0[4] = -0.01; Q0[5] = 0.03; Q0[6] = -0.01; Q0[7] = -0.01;
            Q0[8] = -0.01; Q0[9] = -0.01; Q0[10] = 0.03; Q0[11] = -0.01;
            Q0[12] = -0.01; Q0[13] = -0.01; Q0[14] = -0.01; Q0[15] = 0.03;

        }else{
            if(data.isupdated_accs && data_0.accs_nbr>0){
                for(i = 0; i<3;++i){
                    data_0.accs_0[i] += data.accs[i];
                }
                --data_0.accs_nbr;
            }
            if(data.isupdated_gyros && data_0.gyros_nbr>0){
                for(i = 0; i<3;++i){
                    data_0.gyros_0[i] += data.gyros[i];
                }
                --data_0.gyros_nbr;
            }
            if(data.isupdated_mags && data_0.mags_nbr>0){
                for(i = 0; i<3;++i){
                    data_0.mags_0[i] += data.mags[i];
                }
                --data_0.mags_nbr;
            }
        }
    }

}

void Orienter::update(){

    issend_orienter=false;
    bool fuse = true;

    if(isreceived_imu){
        isreceived_imu = false;

        //we filtrate noise when no movement is done to avoid unecessary drift
        //this is bad for very slowly moving objects but in 99% of the cases its ok
        if(abs(p)<0.02)p=0;
        if(abs(q)<0.02)q=0;
        if(abs(r)<0.02)r=0;

        //if(p==0&&q==0&&r==0)fuse = false;

        //filter
        ax = lp_coeff * ax + (1-lp_coeff)*lp_ax;
        ay = lp_coeff * ay + (1-lp_coeff)*lp_ay;
        az = lp_coeff * az + (1-lp_coeff)*lp_az;
        lp_ax = ax;
        lp_ay = ay;
        lp_az = az;
        mx = lpm_coeff * mx + (1-lpm_coeff)*lp_mx;
        my = lpm_coeff * my + (1-lpm_coeff)*lp_my;
        mz = lpm_coeff * mz + (1-lpm_coeff)*lp_mz;
        lp_mx = mx;
        lp_my = my;
        lp_mz = mz;

        //cerr<<"time now "<<time_receive_imu<<endl;

        //cerr<<"time prev "<<prev_time_receive_imu<<endl;
        //dt = 0.001;//time_receive_imu-prev_time_receive_imu;
        //prev_time_receive_imu = time_receive_imu;
        dt = time_receive_imu-prev_time_receive_imu;
        cerr<<"dt "<<dt<<endl;
        //cerr<<time_receive_imu-prev_time_receive_imu<<endl;
        checkHeading();

        //isvalid_head = 0;
        if(isvalid_head){
            calculateHeading();
        }else{
            calculatePitchAcc();
            calculateRollAcc();
        }

        ResultEuler(0)=roll;ResultEuler(1)=pitch;ResultEuler(2)=heading;
        fprintf(stderr,"+++OBSERVED Roll %4.1f Pitch %4.1f Heading %4.1f \n",ResultEuler(0)*Y_TO_DEG,ResultEuler(1)*Y_TO_DEG,ResultEuler(2)*Y_TO_DEG);

    //    //kalman update

        //if(fuse){
            predict();

            //calcultae Rpredict matrix
            //quaternion to Matrix
            Rpredict<<(1-2*(Xk_(2)*Xk_(2)+Xk_(3)*Xk_(3))),2*(Xk_(1)*Xk_(2)-Xk_(0)*Xk_(3)),2*(Xk_(1)*Xk_(3)+Xk_(0)*Xk_(2)),
               2*(Xk_(1)*Xk_(2)+Xk_(0)*Xk_(3)),(1-2*(Xk_(1)*Xk_(1)+Xk_(3)*Xk_(3))),2*(Xk_(2)*Xk_(3)-Xk_(0)*Xk_(1)),
               2*(Xk_(1)*Xk_(3)-Xk_(0)*Xk_(2)),2*(Xk_(2)*Xk_(3)+Xk_(0)*Xk_(1)), (1-2*(Xk_(1)*Xk_(1)+Xk_(2)*Xk_(2)));

            checkAcc();//we use the predicted rotation matrix as an updated matrix to check

            if(isvalid_acc){
                observe();
            }else{
                cerr<<"ACCS OFF!!!"<<endl;
            }

            //convert to Euler
            ResultQ(0) = Xk(0);
            ResultQ(1) = Xk(1);
            ResultQ(2) = Xk(2);
            ResultQ(3) = Xk(3);

            quatToRPY2(ResultEuler, ResultQ, Rx, Ry, Rz, R, norme);

        //}
        //save result in degree!
        res_roll = ResultEuler(0)*Y_TO_DEG;
        res_pitch = ResultEuler(1)*Y_TO_DEG;
        res_yaw = ResultEuler(2)*Y_TO_DEG;

        fprintf(stdout,"***FUSED Roll %4.1f Pitch %4.1f Yaw %4.1f \n",res_roll,res_pitch,res_yaw);
        //////////fprintf(stderr,"***ROUGH DISPLACEMENT X %2.4f Y %2.4f Z %2.4f \n",x,y,z);

        message_out_orienter.roll = ResultEuler(0);
        message_out_orienter.pitch = ResultEuler(1);
        message_out_orienter.yaw = ResultEuler(2);
        issend_orienter = true;

        isupdated_marg = true;

    }else{
        usleep(5000);
    }

     //fprintf(stderr, "\r\x1b[5A");

}

void Orienter::predict(){

    //thisquaternion represents the change of earth frame on sensor frame!!
    /*if we use gyros only we ll have and error of 200 degrees/hour
    since we have a residual error of 0.001rad/s in gyros which becomes 200degres/s
      */
    //cerr<<"dt"<<dt<<endl;

    Omega<<0.0,-p,-q,-r,p,0.0,r,-q,q,-r,0.0,p,r,q,-p,0.0;
    Omega*=0.5*dt;

    //update state
    Xk_= Xk + Omega*Xk;
    //update covariance
    //calculate jacobian F
    Fk<<1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0;
    //Fk += Omega;

    //cerr<<"Fk "<<Fk<<endl;
    Fkt = Fk.transpose();

    //cerr<<"Fkt "<<Fkt<<endl;

    //mmmm check which quternions act on the yaw and add error
    //Pk += P0bis; //see P0bis we
    //Pk = P0;
    //Pk += P0bis;
      Pk = P0;
//    Matrix3d Qinit;
//    Qinit<<(p*p*p_bw),0.0,0.0,0.0,q*q*q_bw,0.0,0.0,0.0,r*r*r_bw;
    //Qk = Qmap*Qinit*Qmapt + Qkinit;
    //Qk = Qkinit;
    //Qk = Qmap*Qinit*Qmapt + Qkinit;

    //normally should multiply Qk by dt^2
   // cerr<<"Pk before"<<Pk<<endl;

    Pk_ = Fk*Pk*Fkt;// + alpha*Qk;

    //cerr<<"First Pk_"<<Pk_<<endl;

    //normalize
    norme = Xk_(0)*Xk_(0)+Xk_(1)*Xk_(1)+Xk_(2)*Xk_(2)+Xk_(3)*Xk_(3);

    Xk_(0) /= norme;
    Xk_(1) /= norme;
    Xk_(2) /= norme;
    Xk_(3) /= norme;

    //to print the result of prediction we take quaternion* which represents the output of the system
    ResultQ(0) = Xk_(0);
    ResultQ(1) = Xk_(1);
    ResultQ(2) = Xk_(2);
    ResultQ(3) = Xk_(3);

    //in case observation data is corrupted
    Xk(0) = Xk_(0);
    Xk(1) = Xk_(1);
    Xk(2) = Xk_(2);
    Xk(3) = Xk_(3);

    Pk = Pk_;

    quatToRPY2(ResultEuler, ResultQ, Rx, Ry, Rz, R, norme);

    ResultEuler(0)*=Y_TO_DEG;ResultEuler(1)*=Y_TO_DEG;ResultEuler(2)*=Y_TO_DEG;

    fprintf(stderr,"---PREDICTED Roll %4.1f Pitch %4.1f Yaw %4.1f \n",ResultEuler(0),ResultEuler(1),ResultEuler(2));

}

void Orienter::observe(){

    //three ways to do this : 1-observe ax,ay magn then calculate z-q*vector*qt
    //2-observe roll pitch heading but the R error model has to be adapted
    //3-we observe ax ay and heading
    //we have rpy = h(q(t)) + v(t)->the expression of h can be derived :
    // h(q(t)) = (arctan(2*(q2*q3+q0*q1),(1-2*(q1*q1+q2*q2))))
    //          arctan(-2*(q1*q2-q0*q2),sqrt((....)))etc..... look at orienter utils
    //we take the jacobian (with respect to the state vector)....mmmm complicated should change to solution 1
    //ok we changed to the mix solution!


    //quaternion to Matrix
//    R<<(1-2*(Xk_(2)*Xk_(2)+Xk_(3)*Xk_(3))),2*(Xk_(1)*Xk_(2)-Xk_(0)*Xk_(3)),2*(Xk_(1)*Xk_(3)+Xk_(0)*Xk_(2)),
//       2*(Xk_(1)*Xk_(2)+Xk_(0)*Xk_(3)),(1-2*(Xk_(1)*Xk_(1)+Xk_(3)*Xk_(3))),2*(Xk_(2)*Xk_(3)-Xk_(0)*Xk_(1)),
//       2*(Xk_(1)*Xk_(3)-Xk_(0)*Xk_(2)),2*(Xk_(2)*Xk_(3)+Xk_(0)*Xk_(1)), (1-2*(Xk_(1)*Xk_(1)+Xk_(2)*Xk_(2)));

    R = Rpredict;

//    double a = 1/(R(2,2)*R(2,2)+R(2,1)*R(2,1));
//    double A = sqrt(R(0,0)*R(0,0)+R(1,0)*R(1,0));
//    double b = 1/(R(2,0)*R(2,0)+A*A);
    double c = 1/(R(1,0)*R(1,0)+R(0,0)*R(0,0));

//    Hk<<((2*Xk_(1)*R(2,2))*a), ((2*(Xk_(0)*R(2,2)+2*Xk_(1)*R(2,1)))*a), ((2*(Xk_(3)*R(2,2)+2*Xk_(2)*R(2,1)))*a), ((2*Xk_(2)*R(2,2))*a),
//            ((2*Xk_(2)*A*A)+R(2,0)*Xk_(3)*R(1,0)*b/A),((-2*Xk_(3)*A*A)+R(2,0)*Xk_(2)*R(1,0)*b/A),
//        ((2*Xk_(3)*R(0,0))*c), ((2*Xk_(2)*R(0,0))*c),((2*(Xk_(1)*R(0,0)+2*Xk_(2)*R(1,0)))*c), ((2*(Xk_(0)*R(0,0)+2*Xk_(3)*R(1,0)))*c);


    //we observe accx, accy and head, for acc R'*(0,0,1), got to move according to q* (ie R' not R which is written up)
    //since a is a fix vector and the sensor is the one moving(think about it), we change qo = qo, q1 = -q1, q2 = -q2, q3 = -q3
    Zk_(0) = 2*(Xk_(1)*Xk_(3)-Xk_(0)*Xk_(2));
    Zk_(1) = 2*(Xk_(2)*Xk_(3)+Xk_(0)*Xk_(1));
    Zk_(2) = 1-2*(Xk_(1)*Xk_(1)+Xk_(2)*Xk_(2));
    Zk_(3) = atan2(R(1,0),R(0,0));//euler formula

    if(Zk_(3)>Y_PI)Zk_(3)-=Y_2PI;
    if(Zk_(3)<-Y_PI)Zk_(3)+=Y_2PI;


    //norme acc vector to 1
    norme = sqrt(Zk_(0)*Zk_(0)+Zk_(1)*Zk_(1)+Zk_(2)*Zk_(2));

    Zk_(0) /= norme;
    Zk_(1) /= norme;
    Zk_(2) /= norme;

    Zk(0) = ax;
    Zk(1) = ay;
    Zk(2) = az;

    //isvalid_head=false;
    if(isvalid_head){
        Zk(3) = heading;
    }else{
        cerr<<"MAGS OFF!!"<<endl;
        Zk(3) = Zk_(3);
    }

    //calculate jacobian
    Hk<<-2*Xk_(2),2*Xk_(3),-2*Xk_(0),2*Xk_(1),
        2*Xk_(1),2*Xk_(0),2*Xk_(3),2*Xk_(2),
       0.0,-2*Xk_(1),-2*Xk_(2),0.0,
      ((2*Xk_(3)*R(0,0))*c), ((2*Xk_(2)*R(0,0))*c),((2*(Xk_(1)*R(0,0)+2*Xk_(2)*R(1,0)))*c), ((2*(Xk_(0)*R(0,0)+2*Xk_(3)*R(1,0)))*c);

     //cerr<<"Xk "<<Xk<<endl;

    //cerr<<"Zk"<<Zk<<endl;

    //cerr<<"Zk_"<<Zk_<<endl;

    //cerr<<"Pk_"<<Pk_<<endl;

    //calculate gain
    Yk = Zk - Zk_;

    if(Yk(3)>Y_PI)Yk(3)-=Y_2PI;
    if(Yk(3)<-Y_PI)Yk(3)+=Y_2PI;


    //cerr<<"Hk "<<Hk<<endl;
//

    ObsS = Hk.transpose();

    //cerr<<"Hkt "<<ObsS<<endl;

    Sk = Hk*Pk_*ObsS +beta*Ro;

    //cerr<<"Sk "<<Sk<<endl;

    Kk = Pk_*ObsS;

    //cerr<<"Kk "<<Kk<<endl;

    ObsS = Sk.inverse();

    //cerr<<"Skinv "<<ObsS<<endl;

    Kk *= ObsS;

   //cerr<<"Kk "<<Kk<<endl;
    //cerr<<"Yk "<<Yk<<endl;

    //update
    VectorXd Xkbis(4,1);
    Xkbis = Kk*Yk;

    //cerr<<"Xkbis "<<Xkbis<<endl;

    //cerr<<"???? Xkbis "<<Xkbis<<endl;
    if(isnan(Xkbis(0))||isnan(Xkbis(1))||isnan(Xkbis(2))||isnan(Xkbis(3))){
        Xkbis.setZero();
        cerr<<"NAN!!"<<endl;
    }else{
        Xk = Xk_ + Xkbis;
        Pk = Pk_ - Kk*Hk*Pk_;
    }

    //normalize
    norme = sqrt(Xk(0)*Xk(0)+Xk(1)*Xk(1)+Xk(2)*Xk(2)+Xk(3)*Xk(3));

    Xk(0) = Xk(0)/norme;
    Xk(1) = Xk(1)/norme;
    Xk(2) = Xk(2)/norme;
    Xk(3) = Xk(3)/norme;

}

void Orienter::correctData(){

    int i=0;
    for(i = 0; i<3; ++i){
        data.accs[i] = data_0.accs_xyz[i]*(data_0.accs_s[i]*data.accs[i]);
        data.accs[i] = data_0.accs_lp * data.accs[i] + data_0.accs_lp_ * last_data.accs[i];

        data.gyros[i] = data_0.gyros_xyz[i]*(data_0.gyros_s[i]*data.gyros[i] - data_0.gyros_0[i]);
        data.gyros[i] = data_0.gyros_lp * data.gyros[i] + data_0.gyros_lp_ * last_data.gyros[i];

        data.mags[i] = data_0.mags_xyz[i]*(data_0.mags_s[i]*data.mags[i]);
        data.mags[i] = data_0.mags_lp * data.mags[i] + data_0.mags_lp_ * last_data.mags[i];
    }

}

void Orienter::calculateRPH(){

    calculateRP();

    fixToMatrix(rph,R);

    matrixMul(R,data.mags,Vr,3,3,1);

    rph[2] = -atan2(Vr[1],Vr[0])-data_0.rph_0[2];
    if(rph[2]>Y_PI)rph[2]-=Y_2PI;
    if(rph[2]<-Y_PI)rph[2]+=Y_2PI;
    return rph[2];

}

void calculateRP(){

    rph[0] = atan2(data.accs[1],data.accs[2]);
    if(rph[0]>Y_PI)rph[0]-=Y_2PI;
    if(rph[0]<-Y_PI)rph[0]+=Y_2PI;

    rph[1] = atan2(-data.accs[0],sqrt(data.accs[1]*data.accs[1]+data.accs[2]*data.accs[2]));
    if(rph[1]>Y_PI)rph[1]-=Y_2PI;
    if(rph[1]<-Y_PI)rph[1]+=Y_2PI;

    rph[2] = 0;

}


void Orienter::checkHeading(){

    isvalid_head = true;
    norme = sqrt(data.mags[0]*data.mags[0]+data.mags[1]*data.mags[1]+data.mags[2]*data.mags[2]);
    if(norme > YP_Head_validity*data_0.mags_n || norme < YP_Head_validity_1*data_0.mags_n ){
        isvalid_head = false;
    }

}

void Orienter::checkAcc(){

    isvalid_acc = true;
    norme = sqrt(data.accs[0]*data.accs[0]+data.accs[1]*data.accs[1]+data.accs[2]*data.accs[2]);
    if(norme > YP_Acc_validity*data_0.accs_n || norme < YP_Acc_validity_1*data_0.accs_n ){
        isvalid_acc = false;
    }

}

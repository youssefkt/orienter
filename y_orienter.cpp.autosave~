#include "y_orienter.h"

double Orienter::time_receive_imu = 0;
double Orienter::prev_time_receive_imu = 0;
MESSAGE_RAWIMU Orienter::message_in_imu;
bool Orienter::isreceived_imu;

void msgHandlerImu(const MESSAGE_RAWIMU& msg){

    Orienter::isreceived_imu = true;
    Orienter::message_in_imu.a[0] = msg.a[0];
    Orienter::message_in_imu.a[1] = msg.a[1];
    Orienter::message_in_imu.a[2] = msg.a[2];
    Orienter::message_in_imu.g[0] = msg.g[0];
    Orienter::message_in_imu.g[1] = msg.g[1];
    Orienter::message_in_imu.g[2] = msg.g[2];
    Orienter::message_in_imu.m[0] = msg.m[0];
    Orienter::message_in_imu.m[1] = msg.m[1];
    Orienter::message_in_imu.m[2] = msg.m[2];
}


Orienter::Orienter(ros::NodeHandle& _n):n(&_n),Xk(4),Pk(4,4),X0(4),P0(4,4),P0bis(4,4),Xk_(4),Pk_(4,4),Fk(4,4),Fkt(4,4),Qk(4,4),Qmap(4,3),Qmapt(3,4),Qkinit(4,4),alpha(1.0),beta(1.0),Omega(4,4),Yk(4),Zk(4),Zk_(4),Sk(4,4),Hk(4,4),Ro(4,4),
    Kk(4,4),ObsS(4,4),roll(0),pitch(0),heading(0),norme(0),deltax(0),deltay(0),deltaz(0),vx_(0),vy_(0),vz_(0),x(0),y(0),z(0),ismoving(false),isaccelerated(false){

    //acc_count[0]=0.0;acc_count[1]=0.0;acc_count[2]=0.0;

    orienter_pub = n->advertise<MESSAGE_ORIENTER>(MESSAGE_NAME_ORIENTER, 3);
    orienter_sub = n->subscribe(MESSAGE_NAME_RAWIMU, 3, msgHandlerImu);

    isreceived_imu = false;
    issend_orienter = false;

    x = 0; y = 0; z = 0;
    res_roll = 0.; res_pitch = 0.;res_yaw = 0.;
    isupdated_marg = false;


    double tau = YP_filter_tau;//we want 100 ms smoothing
    lp_coeff = 1.0/(1.0 + YP_lp_freq_*tau);
    lpm_coeff = 1.0/(1.0 + YP_lp_freq_*3*tau);

}

//Orienter::Orienter():Xk(4),Pk(4,4),X0(4),P0(4,4),P0bis(4,4),Xk_(4),Pk_(4,4),Fk(4,4),Fkt(4,4),Qk(4,4),Qmap(4,3),Qmapt(3,4),Qkinit(4,4),alpha(1.0),beta(1.0),Omega(4,4),Yk(4),Zk(4),Zk_(4),Sk(4,4),Hk(4,4),Ro(4,4),
//    Kk(4,4),ObsS(4,4),roll(0),pitch(0),heading(0),norme(0),deltax(0),deltay(0),deltaz(0),vx_(0),vy_(0),vz_(0),x(0),y(0),z(0),ismoving(false),isaccelerated(false){
//    acc_count[0]=0.0;acc_count[1]=0.0;acc_count[2]=0.0;
//}

void Orienter::send(){
    if(issend_orienter){
        message_out_orienter.timestamp = time_receive_imu;
        orienter_pub.publish(message_out_orienter);
        issend_orienter = false;
    }
}

void Orienter::receive(){
    ros::spinOnce();
    if(isreceived_imu){

        prev_time_receive_imu = time_receive_imu;
        time_receive_imu = YF_get_time();
        ax = message_in_imu.a[0];ay = message_in_imu.a[1];az = message_in_imu.a[2];
        p = message_in_imu.g[0]*Y_TO_RAD;q = message_in_imu.g[1]*Y_TO_RAD;r = message_in_imu.g[2]*Y_TO_RAD;
        mx = message_in_imu.m[0];my = message_in_imu.m[1];mz = message_in_imu.m[2];
        //isreceived_imu = false;
    }

}

bool Orienter::init(){


    cerr<<"Imu Initializing, please don't move !"<<endl;

    isvalid_head = false;
    int mcount = 5;
    int total = 5;
    m0_norm = 0.;
    double m0[3];
    m0[0] = 0.; m0[1] = 0.; m0[2] = 0.;
    double head_0bis = 0.;

    fprintf(stderr, "INFO IMU :: Waiting for mag calib...\n");
    while(mcount>0&& ros::ok()){
        receive();
        if(isreceived_imu){
            m0[0]+=mx;
            m0[1]+=my;
            m0[2]+=mz;
            head_0bis+=calculateHeading();
            --mcount;
            isreceived_imu = false;
        }
        usleep(300000);
    }
    m0[0]/=total;m0[1]/=total;m0[2]/=total;
    m0_norm = sqrt(m0[0]*m0[0]+m0[1]*m0[1]+m0[2]*m0[2]);
    head_0 = head_0bis/total;

    isreceived_imu = false;

    time_receive_imu = YF_get_time();

    while(ros::ok()){
        receive();
        if(isreceived_imu){
            break;
        }
        usleep(300000);
    }


    //receive();

    calculateHeading();

    Vv<<roll,pitch,heading;
    Vector4d Xbis;
    //make X0
    RPYToQuat2(Vv,Xbis,Rx,Ry,Rz,R,norme);
    X0(0)=Xbis(0);X0(1)=Xbis(1);X0(2)=Xbis(2);X0(3)=Xbis(3);
    //make Q q' = f(q,gyros)=> mapping of covariance matrix with d(f)/d(gyros)
    Matrix3d Qinit;
    //Qinit<<p_bw,0.0,0.0,0.0,q_bw,0.0,0.0,0.0,r_bw;
    Qinit<<0.0001,0.0,0.0,0.0,0.0001,0.0,0.0,0.0,0.0001;

    Qmap<<-1,-1,-1,1,-1,1,1,1,-1,-1,1,1;
    Qmap*=0.5;
    Qmapt = Qmap.transpose();

    Qkinit = Qmap*Qinit*Qmapt;//*Qmap;
    //cerr<<"Q is "<<Qk<<endl;

    //make P0 //some degrees of error
    P0<<0.0015,0.0,0.0,0.0,0.0,0.0015,0.0,0.0,0.0,0.0,0.0015,0.0,0.0,0.0,0.0,0.0015;
    //P0bis<<0.01,0.001,0.001,0.001,0.001,0.01,0.001,0.001,0.001,0.001,0.01,0.001,0.001,0.001,0.001,0.01;

    Xk(0) = X0(0);
    Xk(1) = X0(1);
    Xk(2) = X0(2);
    Xk(3) = X0(3);
    Pk=P0;
    //error model
    //Ro<<ax_bw,0.0,0.0,0.0,0.0,ay_bw,0.0,0.0,0.0,0.0,az_bw,0.0,0.0,0.0,0.0,head_bw;
    Ro<<0.001,0.0,0.0,0.0,0.0,0.001,0.0,0.0,0.0,0.0,0.001,0.0,0.0,0.0,0.0,0.001;



    lp_ax = ax;
    lp_ay = ay;
    lp_az = az;
    lp_mx = mx;
    lp_my = my;
    lp_mz = mz;

    cerr<<"Imu Done Init."<<endl;

    return true;
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

double Orienter::calculateRollAcc(){

    roll = atan2(ay,az);
    if(roll>Y_PI)roll-=Y_2PI;
    if(roll<-Y_PI)roll+=Y_2PI;
    return roll;

}

double Orienter::calculatePitchAcc(){

    pitch = atan2(-ax,sqrt(ay*ay+az*az));
    if(pitch>Y_PI)pitch-=Y_2PI;
    if(pitch<-Y_PI)pitch+=Y_2PI;
    return pitch;
}

double Orienter::calculateHeading(){
    roll = calculateRollAcc();
    pitch = calculatePitchAcc();
    //transform to horizontal
    Vv<<mx,my,mz;
    rotateEulerZYX(0.0,pitch,roll,Vv,Vrotated,Rx,Ry,Rz);

    heading = -atan2(Vrotated(1),Vrotated(0))-head_0;
    if(heading>Y_PI)heading-=Y_2PI;
    if(heading<-Y_PI)heading+=Y_2PI;
    return heading;
}

void Orienter::checkHeading(){
    norme = sqrt(mx*mx+my*my+mz*mz);
    if(norme > YP_Head_validity*m0_norm || norme < m0_norm/YP_Head_validity){isvalid_head = false; heading=-1000;}
    else isvalid_head = true;
}

void Orienter::checkAcc(){

    norme = sqrt(ax*ax+ay*ay+az*az);
    if(norme > YP_Acc_validity*Yfix_GravField || norme < Yfix_GravField/YP_Acc_validity){isaccelerated=true;isvalid_acc = false; roll=-1000;pitch=-1000;}
    else {
        isvalid_acc = true;
        isaccelerated = false;
    }
}

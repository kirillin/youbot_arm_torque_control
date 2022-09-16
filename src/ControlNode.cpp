#include <ControlNode.h>


ControlNode::ControlNode() : nh_for_params("~"){
    ROS_INFO_STREAM("Initialization...");
    log_outfile.open("log_file_torques.text");


    status = NOTASK;

    qz << 0, 0, 0, 0, 0;

//    double LamMS215 = 0.1136*C_th5;
//    double LamMS515 = -0.1136*S_th5;
//
//    XX5
//    YY5
//    M4
//    M5
//    MZ5
//    XX4
//    YY4
//    ZZ4
//    MY4
//    MZ5
//    MY4
//    MZ5
//    YY3
//    MZ3
//    MZ4
//    M3
//    XX3
//    MX3
//    ZZ3
//    XZ3
//    MZ4
//    YY2
//    MZ2
//    M2
//    XX2
//    XZ2
//    ZZ2
//    MX2
//    YY1
//    M1
//    IA1
//    ZZ1
//
//    double XXR5 = XX5 - YY5;
//
////    double LamMS214 = 0.135*S_th4;
////    double LamMS414 = 0.27*C_th4;
////    double LamMS224 = -0.135*C_th4;
////    double LamMS424 = 0.27*S_th4;
//
//    double MR4 = M4 + M5;
//
//    double XXR4 = 0.01290496*M5 + 0.2272*MZ5 + XX4 - YY4 + YY5*(C_th5^2 + S_th5^2);
//    double ZZR4 = 0.01290496*M5 + 0.2272*MZ5 + YY5*(C_th5^2 + S_th5^2) + ZZ4;
//    double MYR4 = -0.1136*M5 + MY4 - MZ5;
//
//    double YYR3 = 0.018225*MR4 + YY3 + YY4*(C_th4^2 + S_th4^2);
//    double MZR3 = MZ3 + MZ4;
//    double MR3 = M3 + MR4;
//
//    double XXR3 = XX3 + YY4*(C_th4^2 + S_th4^2) - YYR3;
//    double XZR3 = -0.135*MZ4 + XZ3;
//    double ZZR3 = 0.018225*MR4 + ZZ3;
//    double MXR3 = 0.135*MR4 + MX3;
//
//    double YYR2 = 0.024025*MR3 + YY2 + YYR3*(C_th3^2 + S_th3^2);
//    double MZR2 = MZ2 + MZR3;
//    double MR2 = M2 + MR3;
//    double XXR2 = XX2 - YYR2 + YYR3*(C_th3^2 + S_th3^2);
//    double XZR2 = -0.155*MZR3 + XZ2;
//    double ZZR2 = IA2 + 0.024025*MR3 + ZZ2;
//    double MXR2 = 0.155*MR3 + MX2;
//
//    double YYR1 = 0.001089*MR2 + YY1;
//    double MR1 = M1 + MR2;
//    double ZZR1 = IA1 + 0.001089*MR2 + YYR2*(C_th2^2 + S_th2^2) + ZZ1;
//
//

//    double ZZR1 = 0.340305072827578;
//    double FS1 = 0.577699262401927;
//    double FV1 = 0.998476763613904;
//    double XXR2 = -0.105823697652855;
//    double XY2 = -0.085398425875565;
//    double XZR2 = 0.117633265585871;
//    double YZ2 = 0.016755465151048;
//    double ZZR2 = 0.176161306847470;
//    double MXR2 = -0.321662581040386;
//    double MY2 = -0.009328164072104;
//    double FS2 = 1.221820657399565;
//    double FV2 = 0.818839419990408;
//
//    double XXR3 = -0.170551268162398;
//    double XY3 = -0.032339820025471;
//    double XZR3 = -0.031970903923237;
//    double YZ3 = 0.071687744754188;
//    double ZZR3 = -0.008379276925227;
//    double MXR3 = -0.159825157664023;
//    double MY3 = -0.026057144128220;
//    double IA3 = 0.012209062194758;
//    double FS3 = 1.150194630214374;
//    double FV3 = 0.301075359656546;
//
//    double XXR4 = 0.086431243352941;
//    double XY4 = 0.050965676178886;
//    double XZ4 = 0.004436058500497;
//    double YZ4 = -0.014297789627228;
//    double ZZR4 = 0.035191516835694;
//    double MX4 = -0.012799239535122;
//    double MYR4 = 0.049992311112723;
//
//    double IA4 = -0.028766242838440;
//    double FS4 = 0.260104881744755;
//    double FV4 = 0.276133214174451;
//
//    double XXR5 = -0.073366761258537;
//    double XY5 = -0.002098391940529;
//    double XZ5 = -0.020273454371360;
//    double YZ5 = 0.015666435611986;
//    double ZZ5 = -0.046937762675495;
//    double MX5 = 0.005703106951981;
//    double MY5 = 0.001295977693001;
//
//    double IA5 = 0.055964673269044;
//    double FS5 = 0.080901740491206;
//    double FV5 = 0.061641125267440;



    double ZZR1 = 0.340305072827578;
    double FS1 = 0.577699262401927;
    double FV1 = 0.998476763613904;
    double XXR2 = -0.105823697652855;
    double XY2 = -0.085398425875565;
    double XZR2 = 0.117633265585871;
    double YZ2 = 0.016755465151048;
    double ZZR2 = 0.176161306847470;
    double MXR2 = -0.321662581040386;
    double MY2 = -0.009328164072104;
    double FS2 = 5.5;
    double FV2 = 1.3;

    double XXR3 = -0.170551268162398;
    double XY3 = -0.032339820025471;
    double XZR3 = -0.031970903923237;
    double YZ3 = 0.071687744754188;
    double ZZR3 = -0.008379276925227;
    double MXR3 = -0.159825157664023;
    double MY3 = -0.026057144128220;
    double IA3 = 0.09062194758;
    double FS3 = 1.150194630214374;
    double FV3 = 0.301075359656546;

    double XXR4 = 0.086431243352941;
    double XY4 = 0.050965676178886;
    double XZ4 = 0.004436058500497;
    double YZ4 = -0.014297789627228;
    double ZZR4 = 0.035191516835694;
    double MX4 = -0.012799239535122;
    double MYR4 = 0.049992311112723;

    double IA4 = 0.028766242838440;
    double FS4 = 0.260104881744755;
    double FV4 = 0.276133214174451;

    double XXR5 = -0.073366761258537;
    double XY5 = -0.002098391940529;
    double XZ5 = -0.020273454371360;
    double YZ5 = 0.015666435611986;
    double ZZ5 = -0.046937762675495;
    double MX5 = 0.005703106951981;
    double MY5 = 0.001295977693001;

    double IA5 = 0.055964673269044;
    double FS5 = 0.080901740491206;
    double FV5 = 0.061641125267440;
//
//    chi << ZZR1,
//            FS1,
//            FV1,
//            XXR2,
//            XY2,
//            XZR2,
//            YZ2,
//            ZZR2,
//            MXR2,
//            MY2,
//            FS2,
//            FV2,
//            XXR3,
//            XY3,
//            XZR3,
//            YZ3,
//            ZZR3,
//            MXR3,
//            MY3,
//            IA3,
//            FS3,
//            FV3,
//            XXR4,
//            XY4,
//            XZ4,
//            YZ4,
//            ZZR4,
//            MX4,
//            MYR4,
//            IA4,
//            FS4,
//            FV4,
//            XXR5,
//            XY5,
//            XZ5,
//            YZ5,
//            ZZ5,
//            MX5,
//            MY5,
//            IA5,
//            FS5,
//            FV5;

    chi <<    -0.5177,   -0.0324,   -0.2659,   -0.0303,    0.0019,    0.0588,   -0.0038,   -0.0018,    0.0145;

    kpp <<  10, 0, 0, 0, 0,
            0, 15, 0, 0, 0,
            0, 0, 25, 0, 0,
            0, 0, 0, 30, 0,
            0, 0, 0, 0, 8;

    kdp <<  5, 0, 0, 0, 0,
            0, 8, 0, 0, 0,
            0, 0, 13, 0, 0,
            0, 0, 0, 16, 0,
            0, 0, 0, 0, 4;



    tau_e.torques.resize(N);
    for (int i = 0; i < N; i++) {
        tau_e.torques[i].unit = "m^2 kg s^-2 rad^-1";
        tau_e.torques[i].joint_uri = "arm_joint_" + STR(i+1);
    }


    pose_sub = nh.subscribe("target_pose", 1, &ControlNode::poseCallback, this);
    js_sub = nh.subscribe("joint_states", 1, &ControlNode::jsCallback, this);
    torque_pub = nh.advertise<brics_actuator::JointTorques>("arm_1/arm_controller/torques_command", 1, true);
    ROS_INFO_STREAM("Initialization done");
}


ControlNode::~ControlNode(){
    log_outfile.close();
    torque_pub.shutdown();
    js_sub.shutdown();
}


void ControlNode::calcD(Matrix<double, N, 1> q, Matrix<double, N, 1> dq, Matrix<double, N, 1> ddq, double g){
    double G3 = g; // -9.81

    double th1 = q[0];
    double th2 = q[1];
    double th3 = q[2];
    double th4 = q[3];
    double th5 = q[4];

    double QP1 = dq[0];
    double QP2 = dq[1];
    double QP3 = dq[2];
    double QP4 = dq[3];
    double QP5 = dq[4];

    double QDP1 = ddq[0];
    double QDP2 = ddq[1];
    double QDP3 = ddq[2];
    double QDP4 = ddq[3];
    double QDP5 = ddq[4];

    double C_th1 = cos(th1 - 2.9496);
    double S_th1 = sin(th1 - 2.9496);
    double C_th2 = cos(th2 - 2.7053);
    double S_th2 = sin(th2 - 2.7053);
    double S_th3 = sin(th3 + 2.6354);
    double C_th3 = cos(th3 + 2.6354);
    double C_th4 = cos(th4 - 3.3598);
    double S_th4 = sin(th4 - 3.3598);
    double S_th5 = sin(th5 - 2.8798);
    double C_th5 = cos(th5 - 2.8798);

    double DV61 = QP1*QP1;
    double W12 = -QP1*S_th2;
    double W22 = C_th2*QP1;
    double WP12 = -QDP1*S_th2 + QP2*W22;
    double WP22 = C_th2*QDP1 - QP2*W12;
    double DV22 = W12*W22;
    double DV32 = QP2*W12;
    double DV42 = W22 * W22;
    double DV52 = QP2*W22;
    double DV62 = QP2 * QP2;
    double U112 = -DV42 - DV62;
    double U212 = DV22 + QDP2;
    double U312 = DV32 - WP22;
    double U322 = DV52 + WP12;
    double VSP12 = -0.033*DV61;
    double VSP22 = 0.033*QDP1;
    double VP12 = C_th2*VSP12 + G3*S_th2;
    double VP22 = -C_th2*G3 + S_th2*VSP12;
    double W13 = C_th3*W12 - S_th3*W22;
    double W23 = C_th3*W22 + S_th3*W12;
    double W33 = QP2 + QP3;
    double WP13 = C_th3*WP12 + QP3*W23 - S_th3*WP22;
    double WP23 = C_th3*WP22 - QP3*W13 + S_th3*WP12;
    double WP33 = QDP2 + QDP3;
    double DV13 = W13 * W13;
    double DV23 = W13*W23;
    double DV33 = W13*W33;
    double DV43 = W23 * W23;
    double DV53 = W23*W33;
    double DV63 = W33 * W33;
    double U113 = -DV43 - DV63;
    double U213 = DV23 + WP33;
    double U313 = DV33 - WP23;
    double U123 = DV23 - WP33;
    double U223 = -DV13 - DV63;
    double U323 = DV53 + WP13;
    double VSP13 = 0.155*U112 + VP12;
    double VSP23 = 0.155*U212 + VP22;
    double VSP33 = 0.155*U312 - VSP22;
    double VP13 = C_th3*VSP13 - S_th3*VSP23;
    double VP23 = C_th3*VSP23 + S_th3*VSP13;
    double W14 = C_th4*W13 - S_th4*W23;
    double W24 = C_th4*W23 + S_th4*W13;
    double W34 = QP4 + W33;
    double WP14 = C_th4*WP13 + QP4*W24 - S_th4*WP23;
    double WP24 = C_th4*WP23 - QP4*W14 + S_th4*WP13;
    double WP34 = QDP4 + WP33;
    double DV14 = W14 *W14;
    double DV24 = W14*W24;
    double DV34 = W14*W34;
    double DV44 = W24 * W24;
    double DV54 = W24*W34;
    double DV64 = W34 * W34;
    double U114 = -DV44 - DV64;
    double U214 = DV24 + WP34;
    double U314 = DV34 - WP24;
    double U124 = DV24 - WP34;
    double U224 = -DV14 - DV64;
    double U324 = DV54 + WP14;
    double VSP14 = 0.135*U113 + VP13;
    double VSP24 = 0.135*U213 + VP23;
    double VSP34 = 0.135*U313 + VSP33;
    double VP14 = C_th4*VSP14 - S_th4*VSP24;
    double VP24 = C_th4*VSP24 + S_th4*VSP14;
    double W15 = C_th5*W14 - S_th5*W34;
    double W25 = C_th5*W34 + S_th5*W14;
    double W35 = QP5 - W24;
    double WP15 = C_th5*WP14 + QP5*W25 - S_th5*WP34;
    double WP25 = C_th5*WP34 - QP5*W15 + S_th5*WP14;
    double WP35 = QDP5 - WP24;
    double DV15 = W15 * W15;
    double DV25 = W15*W25;
    double DV35 = W15*W35;
    double DV45 = W25 * W25;
    double DV55 = W25*W35;
    double DV65 = W35 * W35;
    double U115 = -DV45 - DV65;
    double U215 = DV25 + WP35;
    double U315 = DV35 - WP25;
    double U125 = DV25 - WP35;
    double U225 = -DV15 - DV65;
    double U325 = DV55 + WP15;
    double VSP15 = -0.1136*U124 + VP14;
    double VSP25 = -0.1136*U224 + VP24;
    double VSP35 = -0.1136*U324 + VSP34;
    double VP15 = C_th5*VSP15 - S_th5*VSP35;
    double VP25 = C_th5*VSP35 + S_th5*VSP15;
    double DG1ZZ1 = QDP1;
    double DG1FS1 = copysign(1.0, QP1);
    double DG1FV1 = QP1;
    double N13XX2 = C_th2*DV32 - S_th2*WP12;
    double DG1XX2 = N13XX2;
    double DG2XX2 = -DV22;
    double No21XY2 = -QP2*W12 + WP22;
    double No22XY2 = QP2*W22 + WP12;
    double No23XY2 = W12 * W12 - W22 * W22;
    double N13XY2 = C_th2*No22XY2 - No21XY2*S_th2;
    double DG1XY2 = N13XY2;
    double DG2XY2 = No23XY2;
    double No21XZ2 = QDP2 + W12*W22;
    double No22XZ2 = QP2 * QP2 - W12 *W12;
    double No23XZ2 = -QP2*W22 + WP12;
    double N13XZ2 = C_th2*No22XZ2 - No21XZ2*S_th2;
    double DG1XZ2 = N13XZ2;
    double DG2XZ2 = No23XZ2;
    double No21YZ2 = -QP2 *QP2 + W22 * W22;
    double No22YZ2 = QDP2 - W12*W22;
    double No23YZ2 = QP2*W12 + WP22;
    double N13YZ2 = C_th2*No22YZ2 - No21YZ2*S_th2;
    double DG1YZ2 = N13YZ2;
    double DG2YZ2 = No23YZ2;
    double N13ZZ2 = -C_th2*DV32 - DV52*S_th2;
    double DG1ZZ2 = N13ZZ2;
    double DG2ZZ2 = QDP2;
    double N13MX2 = C_th2*VSP22 - 0.033*U312;
    double DG1MX2 = N13MX2;
    double DG2MX2 = VP22;
    double N13MY2 = S_th2*VSP22 - 0.033*U322;
    double DG1MY2 = N13MY2;
    double DG2MY2 = -VP12;
    double DG2FS2 = copysign(1.0, QP2);
    double DG2FV2 = QP2;
    double N21XX3 = C_th3*WP13 + DV33*S_th3;
    double N22XX3 = C_th3*DV33 - S_th3*WP13;
    double N13XX3 = C_th2*N22XX3 - N21XX3*S_th2;
    double DG1XX3 = N13XX3;
    double DG2XX3 = -DV23;
    double DG3XX3 = -DV23;
    double No31XY3 = -W13*W33 + WP23;
    double No32XY3 = W23*W33 + WP13;
    double No33XY3 = W13 * W13 - W23 * W23;
    double N21XY3 = C_th3*No31XY3 + No32XY3*S_th3;
    double N22XY3 = C_th3*No32XY3 - No31XY3*S_th3;
    double N13XY3 = C_th2*N22XY3 - N21XY3*S_th2;
    double DG1XY3 = N13XY3;
    double DG2XY3 = No33XY3;
    double DG3XY3 = No33XY3;
    double No31XZ3 = W13*W23 + WP33;
    double No32XZ3 = -W13 * W13 + W33 * W33;
    double No33XZ3 = -W23*W33 + WP13;
    double N21XZ3 = C_th3*No31XZ3 + No32XZ3*S_th3;
    double N22XZ3 = C_th3*No32XZ3 - No31XZ3*S_th3;
    double N13XZ3 = C_th2*N22XZ3 - N21XZ3*S_th2;
    double DG1XZ3 = N13XZ3;
    double DG2XZ3 = No33XZ3;
    double DG3XZ3 = No33XZ3;
    double No31YZ3 = W23 * W23 - W33 * W33;
    double No32YZ3 = -W13*W23 + WP33;
    double No33YZ3 = W13*W33 + WP23;
    double N21YZ3 = C_th3*No31YZ3 + No32YZ3*S_th3;
    double N22YZ3 = C_th3*No32YZ3 - No31YZ3*S_th3;
    double N13YZ3 = C_th2*N22YZ3 - N21YZ3*S_th2;
    double DG1YZ3 = N13YZ3;
    double DG2YZ3 = No33YZ3;
    double DG3YZ3 = No33YZ3;
    double N21ZZ3 = C_th3*DV53 - DV33*S_th3;
    double N22ZZ3 = -C_th3*DV33 - DV53*S_th3;
    double N13ZZ3 = C_th2*N22ZZ3 - N21ZZ3*S_th2;
    double DG1ZZ3 = N13ZZ3;
    double DG2ZZ3 = WP33;
    double DG3ZZ3 = WP33;
    double FDI32MX3 = C_th3*U213 - S_th3*U113;
    double N21MX3 = -S_th3*VSP33;
    double N22MX3 = -C_th3*VSP33 - 0.155*U313;
    double N23MX3 = 0.155*FDI32MX3 + VP23;
    double N13MX3 = C_th2*N22MX3 - N21MX3*S_th2 - 0.033*U313;
    double DG1MX3 = N13MX3;
    double DG2MX3 = N23MX3;
    double DG3MX3 = VP23;
    double FDI32MY3 = C_th3*U223 - S_th3*U123;
    double N21MY3 = C_th3*VSP33;
    double N22MY3 = -S_th3*VSP33 - 0.155*U323;
    double N23MY3 = 0.155*FDI32MY3 - VP13;
    double N13MY3 = C_th2*N22MY3 - N21MY3*S_th2 - 0.033*U323;
    double DG1MY3 = N13MY3;
    double DG2MY3 = N23MY3;
    double DG3MY3 = -VP13;
    double DG3IA3 = QDP3;
    double DG3FS3 = copysign(1.0, QP3);
    double DG3FV3 = QP3;
    double N31XX4 = C_th4*WP14 + DV34*S_th4;
    double N32XX4 = C_th4*DV34 - S_th4*WP14;
    double N21XX4 = C_th3*N31XX4 + N32XX4*S_th3;
    double N22XX4 = C_th3*N32XX4 - N31XX4*S_th3;
    double N13XX4 = C_th2*N22XX4 - N21XX4*S_th2;
    double DG1XX4 = N13XX4;
    double DG2XX4 = -DV24;
    double DG3XX4 = -DV24;
    double DG4XX4 = -DV24;
    double No41XY4 = -W14*W34 + WP24;
    double No42XY4 = W24*W34 + WP14;
    double No43XY4 = W14 * W14 - W24 * W24;
    double N31XY4 = C_th4*No41XY4 + No42XY4*S_th4;
    double N32XY4 = C_th4*No42XY4 - No41XY4*S_th4;
    double N21XY4 = C_th3*N31XY4 + N32XY4*S_th3;
    double N22XY4 = C_th3*N32XY4 - N31XY4*S_th3;
    double N13XY4 = C_th2*N22XY4 - N21XY4*S_th2;
    double DG1XY4 = N13XY4;
    double DG2XY4 = No43XY4;
    double DG3XY4 = No43XY4;
    double DG4XY4 = No43XY4;
    double No41XZ4 = W14*W24 + WP34;
    double No42XZ4 = -W14 * W14  + W34 * W34;
    double No43XZ4 = -W24*W34 + WP14;
    double N31XZ4 = C_th4*No41XZ4 + No42XZ4*S_th4;
    double N32XZ4 = C_th4*No42XZ4 - No41XZ4*S_th4;
    double N21XZ4 = C_th3*N31XZ4 + N32XZ4*S_th3;
    double N22XZ4 = C_th3*N32XZ4 - N31XZ4*S_th3;
    double N13XZ4 = C_th2*N22XZ4 - N21XZ4*S_th2;
    double DG1XZ4 = N13XZ4;
    double DG2XZ4 = No43XZ4;
    double DG3XZ4 = No43XZ4;
    double DG4XZ4 = No43XZ4;
    double No41YZ4 = W24 * W24 - W34 * W34;
    double No42YZ4 = -W14*W24 + WP34;
    double No43YZ4 = W14*W34 + WP24;
    double N31YZ4 = C_th4*No41YZ4 + No42YZ4*S_th4;
    double N32YZ4 = C_th4*No42YZ4 - No41YZ4*S_th4;
    double N21YZ4 = C_th3*N31YZ4 + N32YZ4*S_th3;
    double N22YZ4 = C_th3*N32YZ4 - N31YZ4*S_th3;
    double N13YZ4 = C_th2*N22YZ4 - N21YZ4*S_th2;
    double DG1YZ4 = N13YZ4;
    double DG2YZ4 = No43YZ4;
    double DG3YZ4 = No43YZ4;
    double DG4YZ4 = No43YZ4;
    double N31ZZ4 = C_th4*DV54 - DV34*S_th4;
    double N32ZZ4 = -C_th4*DV34 - DV54*S_th4;
    double N21ZZ4 = C_th3*N31ZZ4 + N32ZZ4*S_th3;
    double N22ZZ4 = C_th3*N32ZZ4 - N31ZZ4*S_th3;
    double N13ZZ4 = C_th2*N22ZZ4 - N21ZZ4*S_th2;
    double DG1ZZ4 = N13ZZ4;
    double DG2ZZ4 = WP34;
    double DG3ZZ4 = WP34;
    double DG4ZZ4 = WP34;
    double FDI41MX4 = C_th4*U114 + S_th4*U214;
    double FDI42MX4 = C_th4*U214 - S_th4*U114;
    double N31MX4 = -S_th4*VSP34;
    double N32MX4 = -C_th4*VSP34 - 0.135*U314;
    double N33MX4 = 0.135*FDI42MX4 + VP24;
    double FDI32MX4 = C_th3*FDI42MX4 - FDI41MX4*S_th3;
    double N21MX4 = C_th3*N31MX4 + N32MX4*S_th3;
    double N22MX4 = C_th3*N32MX4 - N31MX4*S_th3 - 0.155*U314;
    double N23MX4 = 0.155*FDI32MX4 + N33MX4;
    double N13MX4 = C_th2*N22MX4 - N21MX4*S_th2 - 0.033*U314;
    double DG1MX4 = N13MX4;
    double DG2MX4 = N23MX4;
    double DG3MX4 = N33MX4;
    double DG4MX4 = VP24;
    double FDI41MY4 = C_th4*U124 + S_th4*U224;
    double FDI42MY4 = C_th4*U224 - S_th4*U124;
    double N31MY4 = C_th4*VSP34;
    double N32MY4 = -S_th4*VSP34 - 0.135*U324;
    double N33MY4 = 0.135*FDI42MY4 - VP14;
    double FDI32MY4 = C_th3*FDI42MY4 - FDI41MY4*S_th3;
    double N21MY4 = C_th3*N31MY4 + N32MY4*S_th3;
    double N22MY4 = C_th3*N32MY4 - N31MY4*S_th3 - 0.155*U324;
    double N23MY4 = 0.155*FDI32MY4 + N33MY4;
    double N13MY4 = C_th2*N22MY4 - N21MY4*S_th2 - 0.033*U324;
    double DG1MY4 = N13MY4;
    double DG2MY4 = N23MY4;
    double DG3MY4 = N33MY4;
    double DG4MY4 = -VP14;
    double DG4IA4 = QDP4;
    double DG4FS4 = copysign(1.0, QP4);
    double DG4FV4 = QP4;
    double N41XX5 = C_th5*WP15 + DV35*S_th5;
    double N43XX5 = C_th5*DV35 - S_th5*WP15;
    double N31XX5 = C_th4*N41XX5 + DV25*S_th4;
    double N32XX5 = C_th4*DV25 - N41XX5*S_th4;
    double N21XX5 = C_th3*N31XX5 + N32XX5*S_th3;
    double N22XX5 = C_th3*N32XX5 - N31XX5*S_th3;
    double N13XX5 = C_th2*N22XX5 - N21XX5*S_th2;
    double DG1XX5 = N13XX5;
    double DG2XX5 = N43XX5;
    double DG3XX5 = N43XX5;
    double DG4XX5 = N43XX5;
    double DG5XX5 = -DV25;
    double No51XY5 = -W15*W35 + WP25;
    double No52XY5 = W25*W35 + WP15;
    double No53XY5 = W15 * W15 - W25 * W25;
    double N41XY5 = C_th5*No51XY5 + No52XY5*S_th5;
    double N43XY5 = C_th5*No52XY5 - No51XY5*S_th5;
    double N31XY5 = C_th4*N41XY5 - No53XY5*S_th4;
    double N32XY5 = -C_th4*No53XY5 - N41XY5*S_th4;
    double N21XY5 = C_th3*N31XY5 + N32XY5*S_th3;
    double N22XY5 = C_th3*N32XY5 - N31XY5*S_th3;
    double N13XY5 = C_th2*N22XY5 - N21XY5*S_th2;
    double DG1XY5 = N13XY5;
    double DG2XY5 = N43XY5;
    double DG3XY5 = N43XY5;
    double DG4XY5 = N43XY5;
    double DG5XY5 = No53XY5;
    double No51XZ5 = W15*W25 + WP35;
    double No52XZ5 = -W15 * W15 + W35 * W35;
    double No53XZ5 = -W25*W35 + WP15;
    double N41XZ5 = C_th5*No51XZ5 + No52XZ5*S_th5;
    double N43XZ5 = C_th5*No52XZ5 - No51XZ5*S_th5;
    double N31XZ5 = C_th4*N41XZ5 - No53XZ5*S_th4;
    double N32XZ5 = -C_th4*No53XZ5 - N41XZ5*S_th4;
    double N21XZ5 = C_th3*N31XZ5 + N32XZ5*S_th3;
    double N22XZ5 = C_th3*N32XZ5 - N31XZ5*S_th3;
    double N13XZ5 = C_th2*N22XZ5 - N21XZ5*S_th2;
    double DG1XZ5 = N13XZ5;
    double DG2XZ5 = N43XZ5;
    double DG3XZ5 = N43XZ5;
    double DG4XZ5 = N43XZ5;
    double DG5XZ5 = No53XZ5;
    double No51YZ5 = W25 * W25 - W35 * W35;
    double No52YZ5 = -W15*W25 + WP35;
    double No53YZ5 = W15*W35 + WP25;
    double N41YZ5 = C_th5*No51YZ5 + No52YZ5*S_th5;
    double N43YZ5 = C_th5*No52YZ5 - No51YZ5*S_th5;
    double N31YZ5 = C_th4*N41YZ5 - No53YZ5*S_th4;
    double N32YZ5 = -C_th4*No53YZ5 - N41YZ5*S_th4;
    double N21YZ5 = C_th3*N31YZ5 + N32YZ5*S_th3;
    double N22YZ5 = C_th3*N32YZ5 - N31YZ5*S_th3;
    double N13YZ5 = C_th2*N22YZ5 - N21YZ5*S_th2;
    double DG1YZ5 = N13YZ5;
    double DG2YZ5 = N43YZ5;
    double DG3YZ5 = N43YZ5;
    double DG4YZ5 = N43YZ5;
    double DG5YZ5 = No53YZ5;
    double N41ZZ5 = C_th5*DV55 - DV35*S_th5;
    double N43ZZ5 = -C_th5*DV35 - DV55*S_th5;
    double N31ZZ5 = C_th4*N41ZZ5 - S_th4*WP35;
    double N32ZZ5 = -C_th4*WP35 - N41ZZ5*S_th4;
    double N21ZZ5 = C_th3*N31ZZ5 + N32ZZ5*S_th3;
    double N22ZZ5 = C_th3*N32ZZ5 - N31ZZ5*S_th3;
    double N13ZZ5 = C_th2*N22ZZ5 - N21ZZ5*S_th2;
    double DG1ZZ5 = N13ZZ5;
    double DG2ZZ5 = N43ZZ5;
    double DG3ZZ5 = N43ZZ5;
    double DG4ZZ5 = N43ZZ5;
    double DG5ZZ5 = WP35;
    double FDI51MX5 = C_th5*U115 + S_th5*U215;
    double FDI53MX5 = C_th5*U215 - S_th5*U115;
    double N41MX5 = -0.1136*FDI53MX5 + S_th5*VSP25;
    double N43MX5 = C_th5*VSP25 + 0.1136*FDI51MX5;
    double FDI41MX5 = C_th4*FDI51MX5 - S_th4*U315;
    double FDI42MX5 = -C_th4*U315 - FDI51MX5*S_th4;
    double N31MX5 = C_th4*N41MX5 - S_th4*VP25;
    double N32MX5 = -C_th4*VP25 - 0.135*FDI53MX5 - N41MX5*S_th4;
    double N33MX5 = 0.135*FDI42MX5 + N43MX5;
    double FDI32MX5 = C_th3*FDI42MX5 - FDI41MX5*S_th3;
    double N21MX5 = C_th3*N31MX5 + N32MX5*S_th3;
    double N22MX5 = C_th3*N32MX5 - 0.155*FDI53MX5 - N31MX5*S_th3;
    double N23MX5 = 0.155*FDI32MX5 + N33MX5;
    double N13MX5 = C_th2*N22MX5 - 0.033*FDI53MX5 - N21MX5*S_th2;
    double DG1MX5 = N13MX5;
    double DG2MX5 = N23MX5;
    double DG3MX5 = N33MX5;
    double DG4MX5 = N43MX5;
    double DG5MX5 = VP25;
    double FDI51MY5 = C_th5*U125 + S_th5*U225;
    double FDI53MY5 = C_th5*U225 - S_th5*U125;
    double N41MY5 = -C_th5*VSP25 - 0.1136*FDI53MY5;
    double N43MY5 = 0.1136*FDI51MY5 + S_th5*VSP25;
    double FDI41MY5 = C_th4*FDI51MY5 - S_th4*U325;
    double FDI42MY5 = -C_th4*U325 - FDI51MY5*S_th4;
    double N31MY5 = C_th4*N41MY5 + S_th4*VP15;
    double N32MY5 = C_th4*VP15 - 0.135*FDI53MY5 - N41MY5*S_th4;
    double N33MY5 = 0.135*FDI42MY5 + N43MY5;
    double FDI32MY5 = C_th3*FDI42MY5 - FDI41MY5*S_th3;
    double N21MY5 = C_th3*N31MY5 + N32MY5*S_th3;
    double N22MY5 = C_th3*N32MY5 - 0.155*FDI53MY5 - N31MY5*S_th3;
    double N23MY5 = 0.155*FDI32MY5 + N33MY5;
    double N13MY5 = C_th2*N22MY5 - 0.033*FDI53MY5 - N21MY5*S_th2;
    double DG1MY5 = N13MY5;
    double DG2MY5 = N23MY5;
    double DG3MY5 = N33MY5;
    double DG4MY5 = N43MY5;
    double DG5MY5 = -VP15;
    double DG5IA5 = QDP5;
    double DG5FS5 = copysign(1.0, QP5);
    double DG5FV5 = QP5;

//    D <<  DG1ZZ1,DG1FS1,DG1FV1,	                                        DG1XX2,DG1XY2,DG1XZ2,DG1YZ2,DG1ZZ2,DG1MX2,DG1MY2,0,0,	                DG1XX3,DG1XY3,DG1XZ3,DG1YZ3,DG1ZZ3,DG1MX3,DG1MY3,0,0,0,	                    DG1XX4,DG1XY4,DG1XZ4,DG1YZ4,DG1ZZ4,DG1MX4,DG1MY4,0,0,0,	                DG1XX5,DG1XY5,DG1XZ5,DG1YZ5,DG1ZZ5,DG1MX5,DG1MY5,0,0,0,
//        0,0,0,                                                          DG2XX2,DG2XY2,DG2XZ2,DG2YZ2,DG2ZZ2,DG2MX2,DG2MY2,DG2FS2,DG2FV2,	        DG2XX3,DG2XY3,DG2XZ3,DG2YZ3,DG2ZZ3,DG2MX3,DG2MY3,0,0,0,	                    DG2XX4,DG2XY4,DG2XZ4,DG2YZ4,DG2ZZ4,DG2MX4,DG2MY4,0,0,0,	                DG2XX5,DG2XY5,DG2XZ5,DG2YZ5,DG2ZZ5,DG2MX5,DG2MY5,0,0,0,
//        0,0,0,                                                          0,0,0,0,0,0,0,0,0,                                                      DG3XX3,DG3XY3,DG3XZ3,DG3YZ3,DG3ZZ3,DG3MX3,DG3MY3,DG3IA3,DG3FS3,DG3FV3,	    DG3XX4,DG3XY4,DG3XZ4,DG3YZ4,DG3ZZ4,DG3MX4,DG3MY4,0,0,0,	                DG3XX5,DG3XY5,DG3XZ5,DG3YZ5,DG3ZZ5,DG3MX5,DG3MY5,0,0,0,
//        0,0,0,                                                          0,0,0,0,0,0,0,0,0,                                                      0,0,0,0,0,0,0,0,0,0,                                                        DG4XX4,DG4XY4,DG4XZ4,DG4YZ4,DG4ZZ4,DG4MX4,DG4MY4,DG4IA4,DG4FS4,DG4FV4,	DG4XX5,DG4XY5,DG4XZ5,DG4YZ5,DG4ZZ5,DG4MX5,DG4MY5,0,0,0,
//        0,0,0,                                                          0,0,0,0,0,0,0,0,0,                                                      0,0,0,0,0,0,0,0,0,0,                                                        0,0,0,0,0,0,0,0,0,0,                                                    DG5XX5,DG5XY5,DG5XZ5,DG5YZ5,DG5ZZ5,DG5MX5,DG5MY5,DG5IA5,DG5FS5,DG5FV5;

    D <<     0,0,0,0,0,0,DG1MX5,DG1MY5, 1,
    DG2MX2,DG2MY2,DG2MX3,DG2MY3,DG2MX4,DG2MY4,DG2MX5,DG2MY5, 1,
    0,0,          DG3MX3,DG3MY3,DG3MX4,DG3MY4,DG3MX5,DG3MY5, 1,
    0,0, 0,0,                   DG4MX4,DG4MY4,DG4MX5,DG4MY5, 1,
    0,0, 0,0, 0,0,                            DG5MX5,DG5MY5, 1;

}


void ControlNode::calcMC(std::vector<double> q, std::vector<double> dq){
}


void ControlNode::calcGTau(Matrix<double, N, 1> q, Matrix<double, N, 1> dq){
    calcD(q, qz, qz, -9.81);
    G = D * chi;
}

void ControlNode::jsCallback(const sensor_msgs::JointState &msg){
    Matrix<double, N, 1> q;
    q << msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4];
    Matrix<double, N, 1> dq;
    dq << msg.velocity[0], msg.velocity[1], msg.velocity[2], msg.velocity[3],msg.velocity[4];

    Matrix<double, N, 1> tau;
    switch(status){
        case TRAJECTORY:
            calcMC(msg.position, msg.velocity);
        case POSE:
            calcGTau(q, dq);
            break;
        case GRAVITY:
            calcGTau(q, dq);
            break;
        default:
            return;
    }

    switch(status){
        case GRAVITY:
            tau = G;
            break;
        case POSE:
            tau = kpp * (qd - q) - kdp * dq + G;        
        	/*
            tau_raw = kpp * (qd - q) - kdp * dq + G;
            tau_sat = tau_raw;
            double lim[5] = {5,5,5,5,5};
            for (int k = 0; k < 5; k++) {
                if (tau_raw[i] > lim[i] ) {
                    tau_sat[i] = lim[i];
                } else if (tau_raw[i] < -lim[i] ) {
                    tau_sat[i] = -lim[i];
                }
            }
            tau = tau_raw;
			*/
            break;
        default:
            return;
    }

    for (int i = 0; i < N; i++) {

        tau_e.torques[i].value = tau(i);
        tau_e.torques[i].timeStamp = ros::Time::now();

        log_outfile << tau(i) << ' ';

    }
    
    log_outfile << '\n';

    torque_pub.publish(tau_e);

}


void ControlNode::poseCallback(const brics_actuator::JointPositions &msg){
    ROS_INFO_STREAM("New target pose received");

    status = GRAVITY;
    // candle: 3.04, 1.18, -2.63, 1.75, 2.89
    qd << 1, 1.18-0.5, -2.63, 1.7499998514010746, 2.89;
//    for (int i = 0; i < N; i++) {
//        std::cout << msg.positions[i].value << ' ';
//    }
//
//
//    char sure = 'n';
//    std::cout << "\nAre you sure? [y/N]";
//    std::cin >> sure;
//
//    if (sure == 'y') {
//        qd[0] = msg.positions[0].value;
////        for (int i = 0; i < N; i++) {
////            qd[i] = msg.positions[i].value;
////        }
//    }
}


void ControlNode::work(){
    ROS_INFO_STREAM("Start waiting for tasks");
    ros::spin();
    ROS_INFO_STREAM("Shutdowning...");
}

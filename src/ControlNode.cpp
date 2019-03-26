#include <ControlNode.h>


ControlNode::ControlNode() : nh_for_params("~"){
    ROS_INFO_STREAM("Initialization...");
    log_outfile.open("log_file_torques.text");


    status = NOTASK;

    qz << 0, 0, 0, 0;
    chi << ZZ1R,
            FV1,
            FS1,
            XX2R,
            XY2,
            XZ2R,
            YZ2,
            ZZ2R,
            MX2R,
            MY2,
            FV2,
            FS2,
            XX3R,
            XY3,
            XZ3R,
            YZ3,
            ZZ3R,
            MX3R,
            MY3,
            IA3,
            FV3,
            FS3,
            XX4R,
            XY4,
            XZ4,
            YZ4,
            ZZ4,
            MX4,
            MY4,
            IA4,
            FV4,
            FS4;

    kpp <<  0.0, 0, 0, 0,
            0, 0.0, 0, 0,
            0, 0, 0.0, 0,
            0, 0, 0, 0.0;

    kdp <<  0.0, 0, 0, 0,
            0, 0.0, 0, 0,
            0, 0, 0.0, 0,
            0, 0, 0, 0.0;



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
    double G3 = g;

    // CHECK: ??
    double th1 = q[0];
    double th2 = q[1];
    double th3 = q[2];
    double th4 = q[3];

    double QP1 = dq[0];
    double QP2 = dq[1];
    double QP3 = dq[2];
    double QP4 = dq[3];

    double QDP1 = ddq[0];
    double QDP2 = ddq[1];
    double QDP3 = ddq[2];
    double QDP4 = ddq[3];


    double S1=sin(2.9496 - th1);
    double C1=cos(2.9496 - th1);
    double S2=sin(2.7045 - th2);
    double C2=cos(2.7045 - th2);
    double S3=-sin(2.5482 + th3);
    double C3=cos(2.5482 + th3);
    double S4=sin(1.7889 - th4);
    double C4=cos(1.7889 - th4);
    double DV331=-QP1*QP1;
    double WI12=QP1*S2;
    double WI22=C2*QP1;
    double WP12=QDP1*S2 + QP2*WI22;
    double WP22=C2*QDP1 - QP2*WI12;
    double DV112=-WI12*WI12;
    double DV222=-WI22*WI22;
    double DV332=-QP2*QP2;
    double DV122=WI12*WI22;
    double DV132=QP2*WI12;
    double DV232=QP2*WI22;
    double U112=DV222 + DV332;
    double U122=DV122 - QDP2;
    double U132=DV132 + WP22;
    double U212=DV122 + QDP2;
    double U222=DV112 + DV332;
    double U232=DV232 - WP12;
    double U312=DV132 - WP22;
    double U322=DV232 + WP12;
    double U332=DV112 + DV222;
    double VSP12=0.033*DV331;
    double VSP22=0.033*QDP1;
    double VP12=-(G3*S2) + C2*VSP12;
    double VP22=-(C2*G3) - S2*VSP12;
    double WI13=C3*WI12 + S3*WI22;
    double WI23=-(S3*WI12) + C3*WI22;
    double W33=QP2 + QP3;
    double WP13=QP3*WI23 + C3*WP12 + S3*WP22;
    double WP23=-(QP3*WI13) - S3*WP12 + C3*WP22;
    double WP33=QDP2 + QDP3;
    double DV113=-WI13*WI13;
    double DV223=-WI23*WI23;
    double DV333=-W33*W33;
    double DV123=WI13*WI23;
    double DV133=W33*WI13;
    double DV233=W33*WI23;
    double U113=DV223 + DV333;
    double U123=DV123 - WP33;
    double U133=DV133 + WP23;
    double U213=DV123 + WP33;
    double U223=DV113 + DV333;
    double U233=DV233 - WP13;
    double U313=DV133 - WP23;
    double U323=DV233 + WP13;
    double U333=DV113 + DV223;
    double VSP13=0.155*U112 + VP12;
    double VSP23=0.155*U212 + VP22;
    double VSP33=0.155*U312 - VSP22;
    double VP13=C3*VSP13 + S3*VSP23;
    double VP23=-(S3*VSP13) + C3*VSP23;
    double WI14=C4*WI13 + S4*WI23;
    double WI24=-(S4*WI13) + C4*WI23;
    double W34=QP4 + W33;
    double WP14=QP4*WI24 + C4*WP13 + S4*WP23;
    double WP24=-(QP4*WI14) - S4*WP13 + C4*WP23;
    double WP34=QDP4 + WP33;
    double DV114=-WI14*WI14;
    double DV224=-WI24*WI24;
    double DV334=-W34*W34;
    double DV124=WI14*WI24;
    double DV134=W34*WI14;
    double DV234=W34*WI24;
    double U114=DV224 + DV334;
    double U124=DV124 - WP34;
    double U134=DV134 + WP24;
    double U214=DV124 + WP34;
    double U224=DV114 + DV334;
    double U234=DV234 - WP14;
    double U314=DV134 - WP24;
    double U324=DV234 + WP14;
    double U334=DV114 + DV224;
    double VSP14=0.135*U113 + VP13;
    double VSP24=0.135*U213 + VP23;
    double VSP34=0.135*U313 + VSP33;
    double VP14=C4*VSP14 + S4*VSP24;
    double VP24=-(S4*VSP14) + C4*VSP24;
    double DG1ZZ1=QDP1;
    double DEX0ZZ1=0;
    double DEY0ZZ1=0;
    double DEZ0ZZ1=0;
    double DNX0ZZ1=0.;
    double DNY0ZZ1=0.;
    double DNZ0ZZ1=QDP1;
    double DG1FV1=QP1;
    double DG1FS1=copysign(1.0, QP1);
    double DG2XX2=-DV122;
    double N1XX21=-(DV132*S2) + C2*WP12;
    double N1XX23=C2*DV132 + S2*WP12;
    double DG1XX2=N1XX23;
    double DEX0XX2=0;
    double DEY0XX2=0;
    double DEZ0XX2=0;
    double DNX0XX2=0. + C1*N1XX21 - DV122*S1;
    double DNY0XX2=0. + C1*DV122 + N1XX21*S1;
    double DNZ0XX2=N1XX23;
    double NO2XY23=-DV112 + DV222;
    double DG2XY2=NO2XY23;
    double N1XY21=-(C2*U312) - S2*U322;
    double N1XY23=-(S2*U312) + C2*U322;
    double DG1XY2=N1XY23;
    double DEX0XY2=0;
    double DEY0XY2=0;
    double DEZ0XY2=0;
    double DNX0XY2=0. + C1*N1XY21 + NO2XY23*S1;
    double DNY0XY2=0. - C1*NO2XY23 + N1XY21*S1;
    double DNZ0XY2=N1XY23;
    double NO2XZ22=DV112 - DV332;
    double DG2XZ2=-U232;
    double N1XZ21=-(NO2XZ22*S2) + C2*U212;
    double N1XZ23=C2*NO2XZ22 + S2*U212;
    double DG1XZ2=N1XZ23;
    double DEX0XZ2=0;
    double DEY0XZ2=0;
    double DEZ0XZ2=0;
    double DNX0XZ2=0. + C1*N1XZ21 - S1*U232;
    double DNY0XZ2=0. + N1XZ21*S1 + C1*U232;
    double DNZ0XZ2=N1XZ23;
    double NO2YZ21=-DV222 + DV332;
    double DG2YZ2=U132;
    double N1YZ21=C2*NO2YZ21 + S2*U122;
    double N1YZ23=NO2YZ21*S2 - C2*U122;
    double DG1YZ2=N1YZ23;
    double DEX0YZ2=0;
    double DEY0YZ2=0;
    double DEZ0YZ2=0;
    double DNX0YZ2=0. + C1*N1YZ21 + S1*U132;
    double DNY0YZ2=0. + N1YZ21*S1 - C1*U132;
    double DNZ0YZ2=N1YZ23;
    double DG2ZZ2=QDP2;
    double N1ZZ21=C2*DV232 + DV132*S2;
    double N1ZZ23=-(C2*DV132) + DV232*S2;
    double DG1ZZ2=N1ZZ23;
    double DEX0ZZ2=0;
    double DEY0ZZ2=0;
    double DEZ0ZZ2=0;
    double DNX0ZZ2=0. + C1*N1ZZ21 + QDP2*S1;
    double DNY0ZZ2=0. - C1*QDP2 + N1ZZ21*S1;
    double DNZ0ZZ2=N1ZZ23;
    double DG2MX2=VP22;
    double E1MX21=C2*U112 - S2*U212;
    double E1MX23=S2*U112 + C2*U212;
    double N1MX21=-(S2*VSP22);
    double N1MX22=-0.033*(S2*U112 + C2*U212) - VP22;
    double N1MX23=-0.033*U312 + C2*VSP22;
    double DG1MX2=N1MX23;
    double DEX0MX2=C1*E1MX21 + S1*U312;
    double DEY0MX2=E1MX21*S1 - C1*U312;
    double DEZ0MX2=E1MX23;
    double DNX0MX2=C1*N1MX21 - N1MX22*S1 - 0.147*(E1MX21*S1 - C1*U312);
    double DNY0MX2=C1*N1MX22 + N1MX21*S1 + 0.147*(C1*E1MX21 + S1*U312);
    double DNZ0MX2=N1MX23;
    double DG2MY2=-VP12;
    double E1MY21=C2*U122 - S2*U222;
    double E1MY23=S2*U122 + C2*U222;
    double N1MY21=-(C2*VSP22);
    double N1MY22=-0.033*(S2*U122 + C2*U222) + VP12;
    double N1MY23=-0.033*U322 - S2*VSP22;
    double DG1MY2=N1MY23;
    double DEX0MY2=C1*E1MY21 + S1*U322;
    double DEY0MY2=E1MY21*S1 - C1*U322;
    double DEZ0MY2=E1MY23;
    double DNX0MY2=C1*N1MY21 - N1MY22*S1 - 0.147*(E1MY21*S1 - C1*U322);
    double DNY0MY2=C1*N1MY22 + N1MY21*S1 + 0.147*(C1*E1MY21 + S1*U322);
    double DNZ0MY2=N1MY23;
    double DG2FV2=QP2;
    double DG2FS2=copysign(1.0, QP2);
    double DG3XX3=-DV123;
    double N2XX31=-(DV133*S3) + C3*WP13;
    double N2XX32=C3*DV133 + S3*WP13;
    double DG2XX3=-DV123;
    double N1XX31=C2*N2XX31 - N2XX32*S2;
    double N1XX33=C2*N2XX32 + N2XX31*S2;
    double DG1XX3=N1XX33;
    double DEX0XX3=0;
    double DEY0XX3=0;
    double DEZ0XX3=0;
    double DNX0XX3=0. + C1*N1XX31 - DV123*S1;
    double DNY0XX3=0. + C1*DV123 + N1XX31*S1;
    double DNZ0XX3=N1XX33;
    double NO3XY33=-DV113 + DV223;
    double DG3XY3=NO3XY33;
    double N2XY31=-(C3*U313) - S3*U323;
    double N2XY32=-(S3*U313) + C3*U323;
    double DG2XY3=NO3XY33;
    double N1XY31=C2*N2XY31 - N2XY32*S2;
    double N1XY33=C2*N2XY32 + N2XY31*S2;
    double DG1XY3=N1XY33;
    double DEX0XY3=0;
    double DEY0XY3=0;
    double DEZ0XY3=0;
    double DNX0XY3=0. + C1*N1XY31 + NO3XY33*S1;
    double DNY0XY3=0. - C1*NO3XY33 + N1XY31*S1;
    double DNZ0XY3=N1XY33;
    double NO3XZ32=DV113 - DV333;
    double DG3XZ3=-U233;
    double N2XZ31=-(NO3XZ32*S3) + C3*U213;
    double N2XZ32=C3*NO3XZ32 + S3*U213;
    double DG2XZ3=-U233;
    double N1XZ31=C2*N2XZ31 - N2XZ32*S2;
    double N1XZ33=C2*N2XZ32 + N2XZ31*S2;
    double DG1XZ3=N1XZ33;
    double DEX0XZ3=0;
    double DEY0XZ3=0;
    double DEZ0XZ3=0;
    double DNX0XZ3=0. + C1*N1XZ31 - S1*U233;
    double DNY0XZ3=0. + N1XZ31*S1 + C1*U233;
    double DNZ0XZ3=N1XZ33;
    double NO3YZ31=-DV223 + DV333;
    double DG3YZ3=U133;
    double N2YZ31=C3*NO3YZ31 + S3*U123;
    double N2YZ32=NO3YZ31*S3 - C3*U123;
    double DG2YZ3=U133;
    double N1YZ31=C2*N2YZ31 - N2YZ32*S2;
    double N1YZ33=C2*N2YZ32 + N2YZ31*S2;
    double DG1YZ3=N1YZ33;
    double DEX0YZ3=0;
    double DEY0YZ3=0;
    double DEZ0YZ3=0;
    double DNX0YZ3=0. + C1*N1YZ31 + S1*U133;
    double DNY0YZ3=0. + N1YZ31*S1 - C1*U133;
    double DNZ0YZ3=N1YZ33;
    double DG3ZZ3=WP33;
    double N2ZZ31=C3*DV233 + DV133*S3;
    double N2ZZ32=-(C3*DV133) + DV233*S3;
    double DG2ZZ3=WP33;
    double N1ZZ31=C2*N2ZZ31 - N2ZZ32*S2;
    double N1ZZ33=C2*N2ZZ32 + N2ZZ31*S2;
    double DG1ZZ3=N1ZZ33;
    double DEX0ZZ3=0;
    double DEY0ZZ3=0;
    double DEZ0ZZ3=0;
    double DNX0ZZ3=0. + C1*N1ZZ31 + S1*WP33;
    double DNY0ZZ3=0. + N1ZZ31*S1 - C1*WP33;
    double DNZ0ZZ3=N1ZZ33;
    double DG3MX3=VP23;
    double E2MX31=C3*U113 - S3*U213;
    double E2MX32=S3*U113 + C3*U213;
    double N2MX31=S3*VSP33;
    double N2MX32=-0.155*U313 - C3*VSP33;
    double N2MX33=0.155*(S3*U113 + C3*U213) + VP23;
    double DG2MX3=N2MX33;
    double E1MX31=C2*E2MX31 - E2MX32*S2;
    double E1MX33=C2*E2MX32 + E2MX31*S2;
    double N1MX31=C2*N2MX31 - N2MX32*S2;
    double N1MX32=-N2MX33 - 0.033*(C2*E2MX32 + E2MX31*S2);
    double N1MX33=C2*N2MX32 + N2MX31*S2 - 0.033*U313;
    double DG1MX3=N1MX33;
    double DEX0MX3=C1*E1MX31 + S1*U313;
    double DEY0MX3=E1MX31*S1 - C1*U313;
    double DEZ0MX3=E1MX33;
    double DNX0MX3=C1*N1MX31 - N1MX32*S1 - 0.147*(E1MX31*S1 - C1*U313);
    double DNY0MX3=C1*N1MX32 + N1MX31*S1 + 0.147*(C1*E1MX31 + S1*U313);
    double DNZ0MX3=N1MX33;
    double DG3MY3=-VP13;
    double E2MY31=C3*U123 - S3*U223;
    double E2MY32=S3*U123 + C3*U223;
    double N2MY31=C3*VSP33;
    double N2MY32=-0.155*U323 + S3*VSP33;
    double N2MY33=0.155*(S3*U123 + C3*U223) - VP13;
    double DG2MY3=N2MY33;
    double E1MY31=C2*E2MY31 - E2MY32*S2;
    double E1MY33=C2*E2MY32 + E2MY31*S2;
    double N1MY31=C2*N2MY31 - N2MY32*S2;
    double N1MY32=-N2MY33 - 0.033*(C2*E2MY32 + E2MY31*S2);
    double N1MY33=C2*N2MY32 + N2MY31*S2 - 0.033*U323;
    double DG1MY3=N1MY33;
    double DEX0MY3=C1*E1MY31 + S1*U323;
    double DEY0MY3=E1MY31*S1 - C1*U323;
    double DEZ0MY3=E1MY33;
    double DNX0MY3=C1*N1MY31 - N1MY32*S1 - 0.147*(E1MY31*S1 - C1*U323);
    double DNY0MY3=C1*N1MY32 + N1MY31*S1 + 0.147*(C1*E1MY31 + S1*U323);
    double DNZ0MY3=N1MY33;
    double DG3IA3=QDP3;
    double DG3FV3=QP3;
    double DG3FS3=copysign(1.0, QP3);
    double DG4XX4=-DV124;
    double N3XX41=-(DV134*S4) + C4*WP14;
    double N3XX42=C4*DV134 + S4*WP14;
    double DG3XX4=-DV124;
    double N2XX41=C3*N3XX41 - N3XX42*S3;
    double N2XX42=C3*N3XX42 + N3XX41*S3;
    double DG2XX4=-DV124;
    double N1XX41=C2*N2XX41 - N2XX42*S2;
    double N1XX43=C2*N2XX42 + N2XX41*S2;
    double DG1XX4=N1XX43;
    double DEX0XX4=0;
    double DEY0XX4=0;
    double DEZ0XX4=0;
    double DNX0XX4=0. + C1*N1XX41 - DV124*S1;
    double DNY0XX4=0. + C1*DV124 + N1XX41*S1;
    double DNZ0XX4=N1XX43;
    double NO4XY43=-DV114 + DV224;
    double DG4XY4=NO4XY43;
    double N3XY41=-(C4*U314) - S4*U324;
    double N3XY42=-(S4*U314) + C4*U324;
    double DG3XY4=NO4XY43;
    double N2XY41=C3*N3XY41 - N3XY42*S3;
    double N2XY42=C3*N3XY42 + N3XY41*S3;
    double DG2XY4=NO4XY43;
    double N1XY41=C2*N2XY41 - N2XY42*S2;
    double N1XY43=C2*N2XY42 + N2XY41*S2;
    double DG1XY4=N1XY43;
    double DEX0XY4=0;
    double DEY0XY4=0;
    double DEZ0XY4=0;
    double DNX0XY4=0. + C1*N1XY41 + NO4XY43*S1;
    double DNY0XY4=0. - C1*NO4XY43 + N1XY41*S1;
    double DNZ0XY4=N1XY43;
    double NO4XZ42=DV114 - DV334;
    double DG4XZ4=-U234;
    double N3XZ41=-(NO4XZ42*S4) + C4*U214;
    double N3XZ42=C4*NO4XZ42 + S4*U214;
    double DG3XZ4=-U234;
    double N2XZ41=C3*N3XZ41 - N3XZ42*S3;
    double N2XZ42=C3*N3XZ42 + N3XZ41*S3;
    double DG2XZ4=-U234;
    double N1XZ41=C2*N2XZ41 - N2XZ42*S2;
    double N1XZ43=C2*N2XZ42 + N2XZ41*S2;
    double DG1XZ4=N1XZ43;
    double DEX0XZ4=0;
    double DEY0XZ4=0;
    double DEZ0XZ4=0;
    double DNX0XZ4=0. + C1*N1XZ41 - S1*U234;
    double DNY0XZ4=0. + N1XZ41*S1 + C1*U234;
    double DNZ0XZ4=N1XZ43;
    double NO4YZ41=-DV224 + DV334;
    double DG4YZ4=U134;
    double N3YZ41=C4*NO4YZ41 + S4*U124;
    double N3YZ42=NO4YZ41*S4 - C4*U124;
    double DG3YZ4=U134;
    double N2YZ41=C3*N3YZ41 - N3YZ42*S3;
    double N2YZ42=C3*N3YZ42 + N3YZ41*S3;
    double DG2YZ4=U134;
    double N1YZ41=C2*N2YZ41 - N2YZ42*S2;
    double N1YZ43=C2*N2YZ42 + N2YZ41*S2;
    double DG1YZ4=N1YZ43;
    double DEX0YZ4=0;
    double DEY0YZ4=0;
    double DEZ0YZ4=0;
    double DNX0YZ4=0. + C1*N1YZ41 + S1*U134;
    double DNY0YZ4=0. + N1YZ41*S1 - C1*U134;
    double DNZ0YZ4=N1YZ43;
    double DG4ZZ4=WP34;
    double N3ZZ41=C4*DV234 + DV134*S4;
    double N3ZZ42=-(C4*DV134) + DV234*S4;
    double DG3ZZ4=WP34;
    double N2ZZ41=C3*N3ZZ41 - N3ZZ42*S3;
    double N2ZZ42=C3*N3ZZ42 + N3ZZ41*S3;
    double DG2ZZ4=WP34;
    double N1ZZ41=C2*N2ZZ41 - N2ZZ42*S2;
    double N1ZZ43=C2*N2ZZ42 + N2ZZ41*S2;
    double DG1ZZ4=N1ZZ43;
    double DEX0ZZ4=0;
    double DEY0ZZ4=0;
    double DEZ0ZZ4=0;
    double DNX0ZZ4=0. + C1*N1ZZ41 + S1*WP34;
    double DNY0ZZ4=0. + N1ZZ41*S1 - C1*WP34;
    double DNZ0ZZ4=N1ZZ43;
    double DG4MX4=VP24;
    double E3MX41=C4*U114 - S4*U214;
    double E3MX42=S4*U114 + C4*U214;
    double N3MX41=S4*VSP34;
    double N3MX42=-0.135*U314 - C4*VSP34;
    double N3MX43=0.135*(S4*U114 + C4*U214) + VP24;
    double DG3MX4=N3MX43;
    double E2MX41=C3*E3MX41 - E3MX42*S3;
    double E2MX42=C3*E3MX42 + E3MX41*S3;
    double N2MX41=C3*N3MX41 - N3MX42*S3;
    double N2MX42=C3*N3MX42 + N3MX41*S3 - 0.155*U314;
    double N2MX43=N3MX43 + 0.155*(C3*E3MX42 + E3MX41*S3);
    double DG2MX4=N2MX43;
    double E1MX41=C2*E2MX41 - E2MX42*S2;
    double E1MX43=C2*E2MX42 + E2MX41*S2;
    double N1MX41=C2*N2MX41 - N2MX42*S2;
    double N1MX42=-N2MX43 - 0.033*(C2*E2MX42 + E2MX41*S2);
    double N1MX43=C2*N2MX42 + N2MX41*S2 - 0.033*U314;
    double DG1MX4=N1MX43;
    double DEX0MX4=C1*E1MX41 + S1*U314;
    double DEY0MX4=E1MX41*S1 - C1*U314;
    double DEZ0MX4=E1MX43;
    double DNX0MX4=C1*N1MX41 - N1MX42*S1 - 0.147*(E1MX41*S1 - C1*U314);
    double DNY0MX4=C1*N1MX42 + N1MX41*S1 + 0.147*(C1*E1MX41 + S1*U314);
    double DNZ0MX4=N1MX43;
    double DG4MY4=-VP14;
    double E3MY41=C4*U124 - S4*U224;
    double E3MY42=S4*U124 + C4*U224;
    double N3MY41=C4*VSP34;
    double N3MY42=-0.135*U324 + S4*VSP34;
    double N3MY43=0.135*(S4*U124 + C4*U224) - VP14;
    double DG3MY4=N3MY43;
    double E2MY41=C3*E3MY41 - E3MY42*S3;
    double E2MY42=C3*E3MY42 + E3MY41*S3;
    double N2MY41=C3*N3MY41 - N3MY42*S3;
    double N2MY42=C3*N3MY42 + N3MY41*S3 - 0.155*U324;
    double N2MY43=N3MY43 + 0.155*(C3*E3MY42 + E3MY41*S3);
    double DG2MY4=N2MY43;
    double E1MY41=C2*E2MY41 - E2MY42*S2;
    double E1MY43=C2*E2MY42 + E2MY41*S2;
    double N1MY41=C2*N2MY41 - N2MY42*S2;
    double N1MY42=-N2MY43 - 0.033*(C2*E2MY42 + E2MY41*S2);
    double N1MY43=C2*N2MY42 + N2MY41*S2 - 0.033*U324;
    double DG1MY4=N1MY43;
    double DEX0MY4=C1*E1MY41 + S1*U324;
    double DEY0MY4=E1MY41*S1 - C1*U324;
    double DEZ0MY4=E1MY43;
    double DNX0MY4=C1*N1MY41 - N1MY42*S1 - 0.147*(E1MY41*S1 - C1*U324);
    double DNY0MY4=C1*N1MY42 + N1MY41*S1 + 0.147*(C1*E1MY41 + S1*U324);
    double DNZ0MY4=N1MY43;
    double DG4IA4=QDP4;
    double DG4FV4=QP4;
    double DG4FS4=copysign(1.0, QP4);

    D <<  DG1ZZ1,DG1FV1,DG1FS1,   DG1XX2,DG1XY2,DG1XZ2,DG1YZ2,DG1ZZ2,DG1MX2,DG1MY2, 0, 0,DG1XX3,DG1XY3,DG1XZ3,DG1YZ3,DG1ZZ3,DG1MX3,DG1MY3,0,0,0,DG1XX4,DG1XY4,DG1XZ4,DG1YZ4,DG1ZZ4,DG1MX4,DG1MY4,0,0,0,
            0, 0, 0,    DG2XX2,DG2XY2,DG2XZ2,DG2YZ2,DG2ZZ2,DG2MX2,DG2MY2,DG2FV2,DG2FS2,     DG2XX3,DG2XY3,DG2XZ3,DG2YZ3,DG2ZZ3,DG2MX3,DG2MY3,0,0,0, DG2XX4,DG2XY4,DG2XZ4,DG2YZ4,DG2ZZ4,DG2MX4,DG2MY4,0,0,0,
            0, 0, 0,    0,0,0,0,0,0,0,0,0,  DG3XX3,DG3XY3,DG3XZ3,DG3YZ3,DG3ZZ3,DG3MX3,DG3MY3,DG3IA3,DG3FV3,DG3FS3,  DG3XX4,DG3XY4,DG3XZ4,DG3YZ4,DG3ZZ4,DG3MX4,DG3MY4,0,0,0,
            0, 0, 0,    0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,    DG4XX4,DG4XY4,DG4XZ4,DG4YZ4,DG4ZZ4,DG4MX4,DG4MY4,DG4IA4,DG4FV4,DG4FS4;
}


void ControlNode::calcMC(std::vector<double> q, std::vector<double> dq){
}


void ControlNode::calcGTau(Matrix<double, N, 1> q, Matrix<double, N, 1> dq){
    calcD(q, qz, qz, 9.82);
    G = D * chi;
}


void ControlNode::jsCallback(const sensor_msgs::JointState &msg){
    Matrix<double, N, 1> q(msg.position[0], msg.position[1], msg.position[2], msg.position[3]);
    Matrix<double, N, 1> dq(msg.velocity[0], msg.velocity[1], msg.velocity[2], msg.velocity[3]);

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
    for (int i = 0; i < N; i++) {
        std::cout << tau(i) << ' ';
    }
    std::cout << '\n';
    torque_pub.publish(tau_e);

}


void ControlNode::poseCallback(const brics_actuator::JointPositions &msg){
    ROS_INFO_STREAM("New target pose received");

    status = GRAVITY;

    qd << 3.0400044219165, 1.1800003252613278, -3.00009979159634, 1.7499998514010746;
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

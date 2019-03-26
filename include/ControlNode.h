#ifndef _CONTROLNODE_
#define _CONTROLNODE_

#include <vector>
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <brics_actuator/JointValue.h>
#include <brics_actuator/JointTorques.h>
#include <brics_actuator/JointPositions.h>

#include <iostream>
#include <fstream>

#include <Eigen/Geometry>
#include "Eigen/Dense"
#include <Eigen/StdVector>

using namespace Eigen;

#define N 4
enum Status {
    NOTASK, GRAVITY, POSE, TRAJECTORY
};
#define STR(A) (static_cast<std::ostringstream*>( &(std::ostringstream() << A) )->str())


class ControlNode {

    Status status;
    ros::Publisher torque_pub;
    ros::Subscriber js_sub, pose_sub;
    ros::NodeHandle nh, nh_for_params;
    brics_actuator::JointTorques tau_e;

<<<<<<< HEAD
    Matrix<double, N, 1> q, qd;
=======
    Matrix<double, N, 1> q, qd, ddq;
>>>>>>> 0054118b6141d383f389b707e8f45092829cabfd
    Matrix<double, N, N> kpp, kdp;
    Matrix<double, N, N> M;
    Matrix<double, N, 1> C, G, tau;

    // FIXME: temporary stuff KVANT project only
    Matrix<double, 4, 32> D;
    Matrix<double, 32, 1> chi;
    Matrix<double, N, 1> qz;


    void calcD(Matrix<double, N, 1> q, Matrix<double, N, 1> dq, Matrix<double, N, 1> ddq, double g);

    void calcMC(std::vector<double> q, std::vector<double> dq);

    void calcGTau(Matrix<double, N, 1> q, Matrix<double, N, 1> dq);

    void jsCallback(const sensor_msgs::JointState &msg);

    void poseCallback(const brics_actuator::JointPositions &msg);

public:
    std::ofstream log_outfile;

    ControlNode();

    ~ControlNode();

    void work();


//    double G3 = 9.82;

//link 1
    double XX1 = 0.006 * 1.2;
    double XY1 = 0;
    double XZ1 = 0;
    double YY1 = 0.003 * 1.2;
    double YZ1 = 0;
    double ZZ1 = -0.006 * 1.2;
    double MX1 = 0.015;
    double MY1 = 0.5;
    double MZ1 = -0.115;
    double M1 = 0.091;
    double IA1 = 13.91 * 10e-6;
    double FS1 = 0;
    double FV1 = 1.0e-4 * 1.2;

//link 2
    double XX2 = 0.0006 * 1.2;
    double XY2 = 0;
    double XZ2 = 0;
    double YY2 = 0.003 * 1.2;
    double YZ2 = 0;
    double ZZ2 = 0.003 * 1.2;
    double MX2 = -0.041;
    double MY2 = 0;
    double MZ2 = -0.020;
    double M2 = 1.318 * 1.1;
    double IA2 = 13.91 * 10e-6;
    double FS2 = 0;
    double FV2 = 1.0e-4 * 1.2;


//link 3
    double XX3 = 0.0006 * 1.2;
    double XY3 = 0;
    double XZ3 = 0;
    double YY3 = 0.002 * 1.2;
    double YZ3 = 0;
    double ZZ3 = 0.002 * 1.2;
    double MX3 = -0.03;
    double MY3 = 0;
    double MZ3 = -0.01;
    double M3 = 0.821 * 1.1;
    double IA3 = 13.57 * 10e-6;
    double FS3 = 0;
    double FV3 = 1.0e-4 * 1.2;

//link 4 + 5
    double XX4 = 0.0001 * 1.2 + 0.0002 * 1.2;
    double XY4 = 0;
    double XZ4 = 0;
    double YY4 = 0.0001 * 1.2 + 0.0002 * 1.2;
    double YZ4 = 0;
    double ZZ4 = 0.0001 * 1.2 + 0.0001 * 1.2;
    double MX4 = 0;
    double MY4 = 0.015;
    double MZ4 = 0.054 - 0.113;
    double M4 = 0.769 * 1.1 + 0.091;
    double IA4 = 9.32 * 10e-6;
    double FS4 = 0;
    double FV4 = 1.0e-4 * 1.2;

// link 5
    double XX5 = 0.0002;
    double XY5 = 0;
    double XZ5 = 0;
    double YY5 = 0.0002;
    double YZ5 = 0;
    double ZZ5 = 0.0001;
    double MX5 = 0;
    double MY5 = 0;
    double MZ5 = -0.113;
    double M5 = 0.091;
    double IA5 = 3.57 * 10e-6;
    double FS5 = 0;
    double FV5 = 1.0e-4 * 1.2;

// regrouped parameters after identification
    double ZZ1R =
            IA1 + 0.018225 * M4 + 0.024025 * (M3 + M4) + 0.001089 * (M2 + M3 + M4) + YY2 + YY3 + YY4 + ZZ1;
    double XX2R = -0.024025 * (M3 + M4) + XX2 - YY2;
    double XZ2R = -0.155 * (MZ3 + MZ4) + XZ2;
    double ZZ2R = IA2 + 0.024025 * (M3 + M4) + ZZ2;
    double MX2R = 0.155 * (M3 + M4) + MX2;
    double XX3R = -0.018225 * M4 + XX3 - YY3;
    double XZ3R = -0.135 * MZ4 + XZ3;
    double ZZ3R = 0.018225 * M4 + ZZ3;
    double MX3R = 0.135 * M4 + MX3;
    double XX4R = XX4 - YY4;

    int NL = 4;
    int NJ = 4;
    int NF = 4;
    int Type = 0;

};

#endif //_CONTROLNODE_

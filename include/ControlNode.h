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

#define N 5
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

    Matrix<double, N, 1> q, qd, ddq;
    Matrix<double, N, N> kpp, kdp;
    Matrix<double, N, N> M;
    Matrix<double, N, 1> C, G, tau;

    // FIXME: temporary stuff KVANT project only
//    Matrix<double, N, 42> D;
//    Matrix<double, 42, 1> chi;

    Matrix<double, N, 9> D;
    Matrix<double, 9, 1> chi;

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


};

#endif //_CONTROLNODE_

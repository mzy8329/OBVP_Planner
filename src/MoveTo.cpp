#include <pinocchio/parsers/urdf.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <future>
#include <chrono>

#include "obvp_solver/obvp_solver.h"
#include "minimum_jerk.hpp"

ros::Publisher q_pub;

void eigen2msg(const Eigen::VectorXd& eigen, std_msgs::Float32MultiArray& msg) {
    msg.data.resize(eigen.size());
    for (int i = 0; i < eigen.size(); i++) {
        msg.data[i] = eigen(i);
    }
}

Eigen::RowVectorXd t2vec(double t) {
    Eigen::RowVectorXd vec(6);
    for (int i = 0; i < 6; i++) {
        vec(i) = std::pow(t, i);
    }
    return vec;
}

void pubCmd(Eigen::Matrix<double, 3, 7> iniState, double T, const Eigen::Matrix<double, 6, Eigen::Dynamic>& C_matrix) {
    std_msgs::Float32MultiArray q_msg;
    // eigen2msg(iniState.row(0), q_msg);
    // q_pub.publish(q_msg);
    // sleep(1);

    double dt = 0.05;
    ros::Rate loop_rate(1 / dt);
    for (double t = 0; t < T; t += dt) {
        Eigen::RowVectorXd t_vec = t2vec(t);
        Eigen::VectorXd q = (t_vec * C_matrix).transpose();
        eigen2msg(q, q_msg);
        q_pub.publish(q_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveTo");
    ros::NodeHandle nh("/moveTo");

    q_pub = nh.advertise<std_msgs::Float32MultiArray>("/mujoco/right_arm/cmd", 10);

    Eigen::Matrix<double, 3, 7> iniState, finState;
    iniState << 5, 0, 0, 0, 0, 0, 1,
        Eigen::Matrix<double, 1, 7>::Zero(),
        Eigen::Matrix<double, 1, 7>::Zero();
    finState << 1.126, -1.6, -0.88, -0.165, -3.33, -1.1, -3.14,
        Eigen::Matrix<double, 1, 7>::Zero(),
        Eigen::Matrix<double, 1, 7>::Zero();
    double T;
    Eigen::Matrix<double, 6, Eigen::Dynamic> C_matrix;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    // ObvpSolver::Plan_S3(iniState, finState, T, C_matrix);
    ObvpSolver::Plan_S3MT(iniState, finState, 0.1, T, C_matrix);
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::cout << "obvp T: " << T << std::endl;
    pubCmd(iniState, T, C_matrix);

    pinocchio::Model model;
    pinocchio::urdf::buildModel("/home/tengxun/lab/pinocchio_ws/src/mujoco_sim/description/jaka/right_jaka.urdf", model);
    MinimumJerk mini_vel_solver(model, 10, 1.0, 1.0);
    std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
    // mini_vel_solver.Solve(iniState, finState, T, C_matrix);
    mini_vel_solver.Solve(finState, iniState, T, C_matrix);
    std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();
    std::cout << "vel constant T: " << T << std::endl;
    std::cout << "calc time used: (obvp) " << (t2 - t1).count() * 1e-6 << "ms,  (min jerk) " << (t4 - t3).count() * 1e-6 << "ms" << std::endl;
    pubCmd(finState, T, C_matrix);




    return 0;
}
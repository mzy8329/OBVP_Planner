#include <pinocchio/parsers/urdf.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <future>
#include <chrono>
#include <Eigen/Dense>

#include "obvp_planner/obvp_planner.hpp"

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
    ros::init(argc, argv, "OBVP_Planner");
    ros::NodeHandle nh("/OBVP_Planner");

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


    Eigen::VectorXd max_vel(7), max_acc(7);
    max_vel << 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0;
    max_acc << 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0;

    ObvpPlanner planner(iniState, 7, max_vel, max_acc, 0.1);

    Eigen::RowVectorXd tar_state = Eigen::RowVectorXd::Random(7);
    std_msgs::Float32MultiArray q_msg;
    double dt = 0.05;
    int i = 0;
    ros::Rate loop_rate(1 / dt);
    while (ros::ok()) {
        if (i > 500) {
            break;
        }

        if (i++ % 100 == 0) {
            tar_state = Eigen::RowVectorXd::Random(7);
        }
        Eigen::VectorXd q = planner.getCurrentOutput_EPVA(tar_state, dt);
        eigen2msg(q, q_msg);
        q_pub.publish(q_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    // // ObvpSolver::Plan_S3(iniState, finState, T, C_matrix);
    // ObvpSolver::Plan_S3MT(iniState, finState, 0.1, T, C_matrix);
    // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    // std::cout << "obvp T: " << T << std::endl;
    // std::cout << "M: " << C_matrix << std::endl;
    // pubCmd(iniState, T, C_matrix);

    // pinocchio::Model model;
    // pinocchio::urdf::buildModel("/home/tengxun/lab/pinocchio_ws/src/mujoco_sim/description/jaka/right_jaka.urdf", model);
    // MinimumJerk mini_vel_solver(model, 10, 1.0, 1.0);
    // std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
    // // mini_vel_solver.Solve(iniState, finState, T, C_matrix);
    // mini_vel_solver.Solve(finState, iniState, T, C_matrix);
    // std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();
    // std::cout << "vel constant T: " << T << std::endl;
    // std::cout << "calc time used: (obvp) " << (t2 - t1).count() * 1e-6 << "ms,  (min jerk) " << (t4 - t3).count() * 1e-6 << "ms" << std::endl;
    // pubCmd(finState, T, C_matrix);
    return 0;
}
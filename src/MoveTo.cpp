#include <pinocchio/parsers/urdf.hpp>

#include "obvp_solver/obvp_solver.h"
#include "minimum_vel.hpp"

int main(int argc, char** argv)
{
    Eigen::Matrix<double, 3, 7> iniState, finState;
    iniState << 5, 0, 0, 0, 0, 0, 1,
        Eigen::Matrix<double, 1, 7>::Zero(),
        0, 0, 0, 0, 0, 0, 1;
    finState << 0.126, -1.6, -0.88, -0.165, -3.33, -1.1, -3.14,
        Eigen::Matrix<double, 1, 7>::Zero(),
        Eigen::Matrix<double, 1, 7>::Zero();
    double T;
    Eigen::Matrix<double, 6, Eigen::Dynamic> C_matrix;

    // ObvpSolver::Plan_S3(iniState, finState, T, C_matrix);
    // std::cout << T << std::endl;
    // std::cout << C_matrix << std::endl;

    pinocchio::Model model;
    pinocchio::urdf::buildModel("/home/tengxun/lab/pinocchio_ws/src/mujoco_sim/description/jaka/right_jaka.urdf", model);
    MinimumVel mini_vel_solver(model, 10, 10.0, 10.0);
    mini_vel_solver.Solve(iniState, finState, T, C_matrix);
    std::cout << T << std::endl;

    return 0;
}
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace ObvpSolver {
    inline Eigen::Matrix4d GetEigenMatrix(const Eigen::Matrix<double, 3, Eigen::Dynamic>& start_state, const Eigen::Matrix<double, 3, Eigen::Dynamic>& end_state) {
        Eigen::VectorXd q0 = start_state.col(0);
        Eigen::VectorXd v0 = start_state.col(1);
        Eigen::VectorXd a0 = start_state.col(2);
        Eigen::VectorXd qe = end_state.col(0);
        Eigen::VectorXd ve = end_state.col(1);
        Eigen::VectorXd ae = end_state.col(2);
        Eigen::Matrix4d eigen_matrix;

        eigen_matrix.setZero();
        eigen_matrix(0, 0) = 12 * (-3 * a0.transpose() * v0 - 2 * a0.transpose() * ve + 2 * ae.transpose() * v0 + 3 * ae.transpose() * ve)[0] / (3 * a0.transpose() * a0 - 2 * a0.transpose() * ae + 3 * ae.transpose() * ae)[0];
        eigen_matrix(0, 1) = 16 * (-5 * a0.transpose() * q0 + 5 * a0.transpose() * qe + 5 * ae.transpose() * q0 - 5 * ae.transpose() * qe - 8 * v0.transpose() * v0 - 14 * v0.transpose() * ve - 8 * ve.transpose() * ve)[0] / (3 * a0.transpose() * a0 - 2 * a0.transpose() * ae + 3 * ae.transpose() * ae)[0];
        eigen_matrix(0, 2) = 600 * (-q0.transpose() * v0 - q0.transpose() * ve + qe.transpose() * v0 + qe.transpose() * ve)[0] / (3 * a0.transpose() * a0 - 2 * a0.transpose() * ae + 3 * ae.transpose() * ae)[0];
        eigen_matrix(0, 3) = 720 * (-q0.transpose() * q0 + 2 * q0.transpose() * qe - qe.transpose() * qe)[0] / (3 * a0.transpose() * a0 - 2 * a0.transpose() * ae + 3 * ae.transpose() * ae)[0];
        eigen_matrix(1, 0) = 1;
        eigen_matrix(2, 1) = 1;
        eigen_matrix(3, 2) = 1;
        return eigen_matrix;
    }

    double GetT(const Eigen::Matrix<double, 3, Eigen::Dynamic>& start_state, const Eigen::Matrix<double, 3, Eigen::Dynamic>& end_state) {
        Eigen::Matrix4d eigen_matrix = GetEigenMatrix(start_state, end_state);
        Eigen::EigenSolver<Eigen::Matrix4d> solver(eigen_matrix);
        Eigen::VectorXd eigen_values = solver.eigenvalues().real();
        return eigen_values.maxCoeff();
    }

    void Plan_S3(const Eigen::Matrix<double, 3, Eigen::Dynamic>& start_state,
        const Eigen::Matrix<double, 3, Eigen::Dynamic>& end_state,
        double& T,
        Eigen::Matrix<double, 6, Eigen::Dynamic>& C) {
        Eigen::VectorXd q0 = start_state.row(0);
        Eigen::VectorXd v0 = start_state.row(1);
        Eigen::VectorXd a0 = start_state.row(2);
        Eigen::VectorXd qe = end_state.row(0);
        Eigen::VectorXd ve = end_state.row(1);
        Eigen::VectorXd ae = end_state.row(2);
        Eigen::Matrix4d eigen_matrix;

        eigen_matrix.setZero();
        eigen_matrix(0, 0) = 12.0 * (-3.0 * a0.transpose() * v0 - 2.0 * a0.transpose() * ve + 2.0 * ae.transpose() * v0 + 3.0 * ae.transpose() * ve)[0] / (3.0 * a0.transpose() * a0 - 2.0 * a0.transpose() * ae + 3.0 * ae.transpose() * ae)[0];
        eigen_matrix(0, 1) = 16.0 * (-5.0 * a0.transpose() * q0 + 5.0 * a0.transpose() * qe + 5.0 * ae.transpose() * q0 - 5.0 * ae.transpose() * qe - 8.0 * v0.transpose() * v0 - 14.0 * v0.transpose() * ve - 8.0 * ve.transpose() * ve)[0] / (3.0 * a0.transpose() * a0 - 2.0 * a0.transpose() * ae + 3.0 * ae.transpose() * ae)[0];
        eigen_matrix(0, 2) = 600.0 * (-q0.transpose() * v0 - q0.transpose() * ve + qe.transpose() * v0 + qe.transpose() * ve)[0] / (3.0 * a0.transpose() * a0 - 2.0 * a0.transpose() * ae + 3.0 * ae.transpose() * ae)[0];
        eigen_matrix(0, 3) = 720.0 * (-q0.transpose() * q0 + 2.0 * q0.transpose() * qe - qe.transpose() * qe)[0] / (3.0 * a0.transpose() * a0 - 2.0 * a0.transpose() * ae + 3.0 * ae.transpose() * ae)[0];
        eigen_matrix(1, 0) = 1.0;
        eigen_matrix(2, 1) = 1.0;
        eigen_matrix(3, 2) = 1.0;

        Eigen::EigenSolver<Eigen::Matrix4d> solver(eigen_matrix);
        Eigen::VectorXd eigen_values = solver.eigenvalues().real();

        T = eigen_values.maxCoeff();

        Eigen::VectorXd alpha = (-30.0 * pow(T, 2) * a0 + 30.0 * pow(T, 2) * ae - -180.0 * T * ve - 180.0 * T * ve - 360.0 * q0 + 360.0 * qe) / std::pow(T, 5);
        Eigen::VectorXd beta = 12.0 * (3.0 * pow(T, 2) * a0 - 2.0 * pow(T, 2) * ae + 16.0 * T * v0 + 14.0 * T * ve + 30.0 * q0 - 30.0 * qe) / std::pow(T, 4);
        Eigen::VectorXd gamma = (-9.0 * pow(T, 2) * a0 + 3.0 * pow(T, 2) * ae - 36.0 * T * v0 - 24.0 * T * ve - 60.0 * q0 + 60.0 * qe) / std::pow(T, 3);


        int dof = start_state.cols();
        Eigen::MatrixXd C_matrix(6, dof);
        C_matrix.row(5) = alpha / 60.0;
        C_matrix.row(4) = beta / 24.0;
        C_matrix.row(3) = gamma / 6.0;
        C_matrix.row(2) = a0 / 2.0;
        C_matrix.row(1) = v0;
        C_matrix.row(0) = q0;

        C = C_matrix;
    }

};
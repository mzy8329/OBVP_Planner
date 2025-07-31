#include <iostream>
#include <eigen3/Eigen/Dense>

namespace ObvpSolver {
    void Plan_S3_EPVA_FAST(const Eigen::Matrix<double, 3, Eigen::Dynamic>& _start_state,
        const Eigen::Matrix<double, 3, Eigen::Dynamic>& _end_state,
        double& _T,
        Eigen::Matrix<double, 6, Eigen::Dynamic>& _C) {
        Eigen::VectorXd q0 = _start_state.row(0);
        Eigen::VectorXd v0 = _start_state.row(1);
        Eigen::VectorXd a0 = _start_state.row(2);
        Eigen::VectorXd qe = _end_state.row(0);
        Eigen::VectorXd ve = _end_state.row(1);
        Eigen::VectorXd ae = _end_state.row(2);
        Eigen::Matrix4d eigen_matrix;

        if (a0.isZero()) {
            a0 = Eigen::VectorXd::Ones(a0.size()) * 0.1;
        }

        eigen_matrix.setZero();
        double den = (3.0 * a0.transpose() * a0 - 2.0 * a0.transpose() * ae + 3.0 * ae.transpose() * ae)[0] + 1e-4;
        eigen_matrix(0, 0) = 12.0 * (-3.0 * a0.transpose() * v0 - 2.0 * a0.transpose() * ve + 2.0 * ae.transpose() * v0 + 3.0 * ae.transpose() * ve)[0] / den;
        eigen_matrix(0, 1) = 16.0 * (-5.0 * a0.transpose() * q0 + 5.0 * a0.transpose() * qe + 5.0 * ae.transpose() * q0 - 5.0 * ae.transpose() * qe - 8.0 * v0.transpose() * v0 - 14.0 * v0.transpose() * ve - 8.0 * ve.transpose() * ve)[0] / den;
        eigen_matrix(0, 2) = 600.0 * (-q0.transpose() * v0 - q0.transpose() * ve + qe.transpose() * v0 + qe.transpose() * ve)[0] / den;
        eigen_matrix(0, 3) = 720.0 * (-q0.transpose() * q0 + 2.0 * q0.transpose() * qe - qe.transpose() * qe)[0] / den;
        eigen_matrix(1, 0) = 1.0;
        eigen_matrix(2, 1) = 1.0;
        eigen_matrix(3, 2) = 1.0;

        Eigen::EigenSolver<Eigen::Matrix4d> solver(eigen_matrix);
        Eigen::VectorXd eigen_values = solver.eigenvalues().real();

        _T = eigen_values.maxCoeff();

        Eigen::VectorXd alpha = (-30.0 * pow(_T, 2) * a0 + 30.0 * pow(_T, 2) * ae - -180.0 * _T * ve - 180.0 * _T * ve - 360.0 * q0 + 360.0 * qe) / std::pow(_T, 5);
        Eigen::VectorXd beta = 12.0 * (3.0 * pow(_T, 2) * a0 - 2.0 * pow(_T, 2) * ae + 16.0 * _T * v0 + 14.0 * _T * ve + 30.0 * q0 - 30.0 * qe) / std::pow(_T, 4);
        Eigen::VectorXd gamma = (-9.0 * pow(_T, 2) * a0 + 3.0 * pow(_T, 2) * ae - 36.0 * _T * v0 - 24.0 * _T * ve - 60.0 * q0 + 60.0 * qe) / std::pow(_T, 3);


        int dof = _start_state.cols();
        Eigen::MatrixXd C_matrix(6, dof);
        C_matrix.row(5) = alpha / 60.0;
        C_matrix.row(4) = beta / 24.0;
        C_matrix.row(3) = gamma / 6.0;
        C_matrix.row(2) = a0 / 2.0;
        C_matrix.row(1) = v0;
        C_matrix.row(0) = q0;

        _C = C_matrix;
    }

    void Plan_S3_EP(const Eigen::Matrix<double, 3, Eigen::Dynamic>& _start_state,
        const Eigen::Matrix<double, 1, Eigen::Dynamic>& _end_state,
        const double _w_t,
        double& _T,
        Eigen::Matrix<double, 6, Eigen::Dynamic>& _C) {
        Eigen::VectorXd q0 = _start_state.row(0);
        Eigen::VectorXd v0 = _start_state.row(1);
        Eigen::VectorXd a0 = _start_state.row(2);
        Eigen::VectorXd qe = _end_state.row(0);
        Eigen::Matrix<double, 15, 15> eigen_matrix;

        eigen_matrix.setZero();
        eigen_matrix(0, 0) = 0;
        eigen_matrix(0, 1) = (40.0 * a0.transpose() * a0)[0] / (3.0 * _w_t);
        eigen_matrix(0, 2) = 320.0 * (a0.transpose() * v0)[0] / _w_t - 126.0;
        eigen_matrix(0, 3) = 160.0 * (a0.transpose() * q0 - a0.transpose() * qe + v0.transpose() * v0)[0] / _w_t / 3.0;
        eigen_matrix(0, 4) = 80.0 * (-57.0 * a0.transpose() * a0 + 16.0 * q0.transpose() * v0 - 16.0 * qe.transpose() * v0)[0] / _w_t;
        eigen_matrix(0, 5) = 4.0 * (-1920.0 * a0.transpose() * v0 - 3969.0 * _w_t + 200.0 * q0.transpose() * q0 - 400.0 * q0.transpose() * qe + 200.0 * qe.transpose() * qe) / _w_t;
        eigen_matrix(0, 6) = 960.0 * (a0.transpose() * q0 - a0.transpose() * qe + v0.transpose() * v0)[0] / _w_t;
        eigen_matrix(0, 7) = 1120.0 * (33.0 * a0.transpose() * a0 + 8.0 * q0.transpose() * v0 - 8.0 * qe.transpose() * v0)[0] / _w_t;
        eigen_matrix(0, 8) = 8.0 * (29760.0 * a0.transpose() * v0 - 9261.0 * _w_t + 1000.0 * q0.transpose() * q0 - 2000.0 * q0.transpose() * qe + 1000.0 * qe.transpose() * qe) / _w_t;
        eigen_matrix(0, 9) = 328320.0 * (a0.transpose() * q0 - a0.transpose() * qe + v0.transpose() * v0)[0] / _w_t;
        eigen_matrix(0, 10) = 960.0 * (399.0 * a0.transpose() * a0 + 872.0 * q0.transpose() * v0 - 872.0 * qe.transpose() * v0)[0] / _w_t;
        eigen_matrix(0, 11) = 1920.0 * (1596.0 * a0.transpose() * v0 + 265.0 * q0.transpose() * q0 - 530.0 * q0.transpose() * qe + 265.0 * qe.trace() * qe)[0] / _w_t;
        eigen_matrix(0, 12) = 4596480.0 * (a0.transpose() * q0 - a0.transpose() * qe + v0.transpose() * v0)[0] / _w_t;
        eigen_matrix(0, 13) = 12257280.0 * (v0.transpose() * q0 - v0.transpose() * qe)[0] / _w_t;
        eigen_matrix(0, 14) = 7660800.0 * (q0.transpose() * q0 - 2.0 * q0.transpose() * qe + qe.transpose() * qe)[0] / _w_t;
        eigen_matrix(1, 0) = 1.0;
        eigen_matrix(2, 1) = 1.0;
        eigen_matrix(3, 2) = 1.0;
        eigen_matrix(4, 3) = 1.0;
        eigen_matrix(5, 4) = 1.0;
        eigen_matrix(6, 5) = 1.0;
        eigen_matrix(7, 6) = 1.0;
        eigen_matrix(8, 7) = 1.0;
        eigen_matrix(9, 8) = 1.0;
        eigen_matrix(10, 9) = 1.0;
        eigen_matrix(11, 10) = 1.0;
        eigen_matrix(12, 11) = 1.0;
        eigen_matrix(13, 12) = 1.0;
        eigen_matrix(14, 13) = 1.0;
        eigen_matrix(15, 14) = 1.0;

        Eigen::EigenSolver<Eigen::Matrix<double, 15, 15>> solver(eigen_matrix);
        Eigen::VectorXd eigen_values = solver.eigenvalues().real();
        _T = eigen_values.maxCoeff();

        Eigen::VectorXd alpha = (-60.0 * pow(_T, 5) * a0 - 120.0 * pow(_T, 4) * v0 - 120.0 * pow(_T, 3) * q0 + 120.0 * pow(_T, 3) * qe - 12.0 * pow(_T, 2) * a0 - 240.0 * _T * v0 - 240.0 * q0 + 240.0 * qe) / (pow(_T, 5) * (pow(_T, 3) + 42.0));
        Eigen::VectorXd beta = 40.0 * (pow(_T, 5) * a0 + 2.0 * pow(_T, 4) * v0 + 2.0 * pow(_T, 3) * q0 - 2.0 * pow(_T, 3) * qe + 6 * pow(_T, 2) * a0 + 12.0 * _T * v0 + 12.0 * q0 - 12.0 * qe) / (pow(_T, 4) * (pow(_T, 3) + 42.0));
        Eigen::VectorXd gamma = -10.0 * (pow(_T, 3) + 18.0) * (pow(_T, 2) * a0 + 2.0 * _T * v0 + 2.0 * q0 - 2.0 * qe) / (pow(_T, 3) * (pow(_T, 3) + 42.0));

        int dof = _start_state.cols();
        Eigen::MatrixXd C_matrix(6, dof);
        C_matrix.row(5) = alpha / 120.0;
        C_matrix.row(4) = beta / 24.0;
        C_matrix.row(3) = gamma / 6.0;
        C_matrix.row(2) = a0 / 2.0;
        C_matrix.row(1) = v0;
        C_matrix.row(0) = q0;

        _C = C_matrix;
    }

    void Plan_S3_EPV(const Eigen::Matrix<double, 3, Eigen::Dynamic>& _start_state,
        const Eigen::Matrix<double, 2, Eigen::Dynamic>& _end_state,
        const double _w_t,
        double& _T,
        Eigen::Matrix<double, 6, Eigen::Dynamic>& _C) {
        Eigen::VectorXd q0 = _start_state.row(0);
        Eigen::VectorXd v0 = _start_state.row(1);
        Eigen::VectorXd a0 = _start_state.row(2);
        Eigen::VectorXd qe = _end_state.row(0);
        Eigen::VectorXd ve = _end_state.row(1);
        Eigen::Matrix<double, 6, 6> eigen_matrix;

        eigen_matrix.setZero();
        eigen_matrix(0, 0) = 0;
        eigen_matrix(0, 1) = (8.0 * a0.transpose() * a0)[0] / _w_t;
        eigen_matrix(0, 2) = 16.0 * (7.0 * a0.transpose() * v0 + 3.0 * a0.transpose() * ve)[0] / _w_t;
        eigen_matrix(0, 3) = 48.0 * (5.0 * a0.transpose() * q0 - 5 * a0.transpose() * qe + 8.0 * v0.transpose() * v0 + 9.0 * v0.transpose() * ve + 3.0 * ve.transpose() * ve)[0] / _w_t;
        eigen_matrix(0, 4) = 320.0 * (5.0 * q0.transpose() * v0 + 3.0 * q0.transpose() * ve - 5.0 * qe.transpose() * v0 - 3.0 * qe.transpose() * ve)[0] / _w_t;
        eigen_matrix(0, 5) = 1600.0 * (q0.transpose() * q0 - 2.0 * q0.transpose() * qe + qe.transpose() * qe)[0] / _w_t;
        eigen_matrix(1, 0) = 1.0;
        eigen_matrix(2, 1) = 1.0;
        eigen_matrix(3, 2) = 1.0;
        eigen_matrix(4, 3) = 1.0;
        eigen_matrix(5, 4) = 1.0;

        Eigen::EigenSolver<Eigen::Matrix<double, 6, 6>> solver(eigen_matrix);
        Eigen::VectorXd eigen_values = solver.eigenvalues().real();
        _T = eigen_values.maxCoeff();

        Eigen::VectorXd alpha = (-40.0 * pow(_T, 2) * a0 - 200.0 * _T * v0 - 120.0 * _T * ve - 320.0 * q0 + 320.0 * qe) / std::pow(_T, 5);
        Eigen::VectorXd beta = 4.0 * (7.0 * pow(_T, 2) * a0 + 32.0 * _T * v0 + 18.0 * _T * ve + 50.0 * q0 - 50.0 * qe) / std::pow(_T, 4);
        Eigen::VectorXd gamma = (-8.0 * pow(_T, 2) * a0 - 28.0 * _T * v0 - 12.0 * _T * ve - 40.0 * q0 + 40.0 * qe) / std::pow(_T, 3);

        int dof = _start_state.cols();
        Eigen::MatrixXd C_matrix(6, dof);
        C_matrix.row(5) = alpha / 120.0;
        C_matrix.row(4) = beta / 24.0;
        C_matrix.row(3) = gamma / 6.0;
        C_matrix.row(2) = a0 / 2.0;
        C_matrix.row(1) = v0;
        C_matrix.row(0) = q0;

        _C = C_matrix;
    }

    void Plan_S3_EPVA(const Eigen::Matrix<double, 3, Eigen::Dynamic>& _start_state,
        const Eigen::Matrix<double, 3, Eigen::Dynamic>& _end_state,
        const double _w_t,
        double& _T,
        Eigen::Matrix<double, 6, Eigen::Dynamic>& _C) {
        Eigen::VectorXd q0 = _start_state.row(0);
        Eigen::VectorXd v0 = _start_state.row(1);
        Eigen::VectorXd a0 = _start_state.row(2);
        Eigen::VectorXd qe = _end_state.row(0);
        Eigen::VectorXd ve = _end_state.row(1);
        Eigen::VectorXd ae = _end_state.row(2);
        Eigen::Matrix<double, 6, 6> eigen_matrix;

        eigen_matrix.setZero();
        double den = _w_t;
        eigen_matrix(0, 0) = 0;
        eigen_matrix(0, 1) = 3.0 * (3.0 * a0.transpose() * a0 - 2.0 * a0.transpose() * ae + 3.0 * ae.transpose() * ae)[0] / den;
        eigen_matrix(0, 2) = 48.0 * (3.0 * a0.transpose() * v0 + 2.0 * a0.transpose() * ve - 2.0 * ae.transpose() * v0 - 3.0 * ae.transpose() * ve)[0] / den;
        eigen_matrix(0, 3) = 72.0 * (5.0 * a0.transpose() * q0 - 5 * a0.transpose() * qe - 5.0 * ae.transpose() * q0 + 5.0 * ae.transpose() * qe + 8.0 * v0.transpose() * v0 + 14.0 * v0.transpose() * ve + 8.0 * ve.transpose() * ve)[0] / den;
        eigen_matrix(0, 4) = 2880 * (q0.transpose() * v0 + q0.transpose() * ve - qe.transpose() * ve)[0] / den;
        eigen_matrix(0, 5) = 3600.0 * (q0.transpose() * q0 - q0.transpose() * qe + qe.transpose() * qe)[0] / den;
        eigen_matrix(1, 0) = 1.0;
        eigen_matrix(2, 1) = 1.0;
        eigen_matrix(3, 2) = 1.0;
        eigen_matrix(4, 3) = 1.0;
        eigen_matrix(5, 4) = 1.0;

        Eigen::EigenSolver<Eigen::Matrix<double, 6, 6>> solver(eigen_matrix);
        Eigen::VectorXd eigen_values = solver.eigenvalues().real();
        _T = eigen_values.maxCoeff();

        Eigen::VectorXd alpha = (-60.0 * pow(_T, 2) * a0 + 60.0 * pow(_T, 2) * ae - 360.0 * _T * ve - 360.0 * _T * ve - 720.0 * q0 + 720.0 * qe) / std::pow(_T, 5);
        Eigen::VectorXd beta = 12.0 * (3.0 * pow(_T, 2) * a0 - 2.0 * pow(_T, 2) * ae + 16.0 * _T * v0 + 14.0 * _T * ve + 30.0 * q0 - 30.0 * qe) / std::pow(_T, 4);
        Eigen::VectorXd gamma = (-9.0 * pow(_T, 2) * a0 + 3.0 * pow(_T, 2) * ae - 36.0 * _T * v0 - 24.0 * _T * ve - 60.0 * q0 + 60.0 * qe) / std::pow(_T, 3);

        int dof = _start_state.cols();
        Eigen::MatrixXd C_matrix(6, dof);
        C_matrix.row(5) = alpha / 120.0;
        C_matrix.row(4) = beta / 24.0;
        C_matrix.row(3) = gamma / 6.0;
        C_matrix.row(2) = a0 / 2.0;
        C_matrix.row(1) = v0;
        C_matrix.row(0) = q0;

        _C = C_matrix;
    }


};
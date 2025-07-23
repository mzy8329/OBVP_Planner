#include "obvp_solver/obvp_solver.hpp"

class ObvpPlanner {
public:
    ObvpPlanner(const Eigen::MatrixXd& _initial_state, int _dof, const Eigen::VectorXd& _max_vel, const Eigen::VectorXd& _max_acc, double _weight_T = 1.0)
        :dof_(_dof), max_vel_(_max_vel), max_acc_(_max_acc), weight_T_(_weight_T) {
        Pos_C_Matrix_.resize(6, _dof);
        Vel_C_Matrix_.resize(6, _dof);
        Acc_C_Matrix_.resize(6, _dof);
        Acc_C_Matrix_.row(4) = Eigen::VectorXd::Zero(dof_);
        Acc_C_Matrix_.row(5) = Eigen::VectorXd::Zero(dof_);
        Vel_C_Matrix_.row(5) = Eigen::VectorXd::Zero(dof_);
        current_state_ = _initial_state;
    };
    ~ObvpPlanner() { ; }

    Eigen::MatrixXd getCurrentState() {
        return current_state_;
    }

    double getT() {
        return T_;
    }

    Eigen::VectorXd getCurrentOutput_EP(const Eigen::MatrixXd& _target_state, double _dt) {
        if (target_state_.rows() != _target_state.rows() || target_state_ != _target_state) {
            target_state_.resize(_target_state.rows(), _target_state.cols());
            target_state_ = _target_state;

            if (_target_state.rows() == 1) {
                ObvpSolver::Plan_S3_EP(current_state_, target_state_, weight_T_, T_, Pos_C_Matrix_);
                UpdateMatrix();
            }
            else {
                std::cout << "Error: target_state.rows() must be 1" << std::endl;
                return Eigen::VectorXd::Zero(dof_);
            }
            t_ = 0;
        }
        return UpdateOutput(_dt);
    }

    Eigen::VectorXd getCurrentOutput_EPVA(const Eigen::MatrixXd& _target_state, double _dt) {
        if (target_state_.rows() != _target_state.rows() || target_state_ != _target_state) {
            target_state_.resize(_target_state.rows(), _target_state.cols());
            target_state_ = _target_state;

            if (_target_state.rows() == 3) {
                ObvpSolver::Plan_S3_EPVA(current_state_, target_state_, weight_T_, T_, Pos_C_Matrix_);
                UpdateMatrix();
            }
            else {
                std::cout << "Error: target_state.rows() must be 3" << std::endl;
                return Eigen::VectorXd::Zero(dof_);
            }
            t_ = 0;
        }
        return UpdateOutput(_dt);
    }


private:
    Eigen::RowVectorXd t2vec(double _t) {
        Eigen::RowVectorXd vec(6);
        for (int i = 0; i < 6; i++) {
            vec(i) = std::pow(_t, i);
        }
        return vec;
    }

    void UpdateMatrix() {
        Vel_C_Matrix_.row(0) = Pos_C_Matrix_.row(1);
        for (int i = 2; i < 6; i++) {
            Vel_C_Matrix_.row(i - 1) = Pos_C_Matrix_.row(i) * i;
            Acc_C_Matrix_.row(i - 2) = Pos_C_Matrix_.row(i) * i * (i - 1);
        }


        Eigen::VectorXd max_vel = Eigen::VectorXd::Zero(dof_);
        Eigen::VectorXd max_acc = Eigen::VectorXd::Zero(dof_);
        for (double t = 0; t < T_; t += 0.1) {
            Eigen::RowVectorXd t_vec = t2vec(t);
            max_vel = max_vel.cwiseMax((t_vec * Vel_C_Matrix_).transpose().cwiseAbs());
            max_acc = max_acc.cwiseMax((t_vec * Acc_C_Matrix_).transpose().cwiseAbs());
        }

        double max_radio = std::max((max_vel.array() / max_vel_.array()).maxCoeff(),
            (max_acc.array() / max_acc_.array()).maxCoeff());
        if (max_radio > 1.0) {
            t_resolution_ = 1.0 / max_radio;
        }
    }

    inline Eigen::VectorXd UpdateOutput(double _dt) {
        if (t_ < T_) {
            t_ = std::min(t_ + t_resolution_ * _dt, T_);
            Eigen::RowVectorXd t_vec = t2vec(t_);
            current_state_.row(0) = t_vec * Pos_C_Matrix_;
            current_state_.row(1) = t_vec * Vel_C_Matrix_;
            current_state_.row(2) = t_vec * Acc_C_Matrix_;
            return current_state_.row(0);
        }
        else {
            return current_state_.row(0);
        }
    }

private:
    int dof_;
    double weight_T_;
    Eigen::VectorXd max_vel_;
    Eigen::VectorXd max_acc_;

    Eigen::Matrix<double, 6, Eigen::Dynamic> Pos_C_Matrix_;
    Eigen::Matrix<double, 6, Eigen::Dynamic> Vel_C_Matrix_;
    Eigen::Matrix<double, 6, Eigen::Dynamic> Acc_C_Matrix_;
    double T_ = 0;

    Eigen::MatrixXd target_state_;
    Eigen::MatrixXd current_state_;
    double t_ = 0;
    double t_resolution_ = 1.0;

};

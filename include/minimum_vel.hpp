#include <vector>
#include <casadi/casadi.hpp>
#include <casadi/core/callback.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>


class JacobianCallback : public casadi::Callback {
public:
    JacobianCallback(const std::string& name, pinocchio::Model& model) : model_(model) {
        construct(name);
        data_ = pinocchio::Data(model_);
    }
    ~JacobianCallback() override = default;

    std::string get_name_in(casadi_int i) override { return "q"; }
    std::string get_name_out(casadi_int i) override { return "J"; }
    casadi_int get_n_in() override { return 1; }
    casadi_int get_n_out() override { return 1; }
    casadi::Sparsity get_sparsity_in(casadi_int i) override { return casadi::Sparsity::dense(model_.nq); }
    casadi::Sparsity get_sparsity_out(casadi_int i) override { return casadi::Sparsity::dense(6, model_.nq); }

    bool has_jacobian() const override { return false; }
    bool has_forward(casadi_int nfwd) const override { return false; }
    bool has_reverse(casadi_int nadj) const override { return false; }

    std::vector<casadi::DM> eval(const std::vector<casadi::DM>& arg) const override {
        Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(arg[0].ptr(), arg[0].size1());
        pinocchio::forwardKinematics(model_, data, q);
        pinocchio::updateFramePlacements(model_, data);

        Eigen::Vector3d ee_pose = data.oMf[model_.nframes - 1].translation();

        computeJointJacobians(model_, data, q);
        Eigen::MatrixXd J = data.J;
        casadi::DM J_dm = casadi::DM::zeros(J.rows(), J.cols());
        std::copy(J.data(), J.data() + J.size(), J_dm.ptr());
        return { J_dm };
    }



public:
    pinocchio::Model model_;
    pinocchio::Data data_;
    int nv_ = 6;
};


class MinimumVel {
public:
    MinimumVel(pinocchio::Model& model, int N, double max_vel, double max_acc) : model_(model) {
        casadi::SX T = casadi::SX::sym("T", 1);
        casadi::SX q0 = casadi::SX::sym("q0", model.nq);
        casadi::SX v0 = casadi::SX::sym("v0", model.nq);
        casadi::SX a0 = casadi::SX::sym("a0", model.nq);
        casadi::SX qe = casadi::SX::sym("qe", model.nq);
        casadi::SX ve = casadi::SX::sym("ve", model.nq);
        casadi::SX ae = casadi::SX::sym("ae", model.nq);
        casadi::SX params = vertcat(q0, v0, a0, qe, ve, ae);

        casadi::SX alpha = (-30.0 * T * T * a0 + 30.0 * T * T * ae - 180.0 * T * v0 - 180.0 * T * ve - 360.0 * q0 + 360.0 * qe) / casadi::SX::pow(T, 5);
        casadi::SX beta = 12.0 * (3.0 * T * T * a0 - 2.0 * T * T * ae + 16.0 * T * v0 + 14.0 * T * ve + 30.0 * q0 - 30.0 * qe) / casadi::SX::pow(T, 4);
        casadi::SX gamma = (-9.0 * T * T * a0 + 3 * T * T * ae - 36.0 * T * v0 - 24.0 * T * ve - 60.0 * q0 + 60.0 * qe) / casadi::SX::pow(T, 3);


        std::vector<casadi::SX> g;
        casadi::SX loss = 0;

        casadi::SX q = casadi::SX::sym("q", model.nq);
        jac_callback_ = std::make_shared<JacobianCallback>("jacobian_cb", model_);
        get_jacobian_ = casadi::Function("getJacobian", { q }, { (*jac_callback_)({q})[0] });

        g.push_back(T);
        lbg_.push_back(0.0);
        ubg_.push_back(1e5);

        for (int i = 1; i < N - 1; i++) {
            casadi::SX T1 = T * i / static_cast<double>(N);
            casadi::SX T2 = T1 * T1;
            casadi::SX T3 = T1 * T2;
            casadi::SX T4 = T2 * T2;
            casadi::SX T5 = T2 * T3;
            casadi::SX q = alpha / 60.0 * T5 + beta / 24.0 * T4 + gamma / 6.0 * T3 + a0 / 2.0 * T2 + v0 * T1 + q0;
            casadi::SX v = alpha / 12.0 * T4 + beta / 6.0 * T3 + gamma / 2.0 * T2 + a0 * T1 + v0;
            casadi::SX a = alpha / 3.0 * T3 + beta / 2.0 * T2 + gamma * T1 + a0;
            casadi::SX j = alpha * T2 + beta * T1 + gamma;

            g.push_back(q);
            g.push_back(v);
            g.push_back(a);
            for (int j = 0; j < model.nq; j++) {
                lbg_.push_back(model.lowerPositionLimit[j]);
                ubg_.push_back(model.upperPositionLimit[j]);
            }
            for (int j = 0; j < model.nq; j++) {
                lbg_.push_back(-max_vel);
                ubg_.push_back(max_vel);
            }
            for (int j = 0; j < model.nq; j++) {
                lbg_.push_back(-max_acc);
                ubg_.push_back(max_acc);
            }

            loss += (w_qj_ * casadi::SX::mtimes(j.T(), j));
            casadi::SX J = get_jacobian_(casadi::DM(q))[0];
            casadi::SX pv = casadi::SX::mtimes(J, v);
            loss += w_pv_ * casadi::SX::mtimes(pv.T(), pv);
        }
        loss += w_T_ * T;

        casadi::SXDict nlp = {
        {"x", T},
        {"f", loss},
        {"g", vertcat(g)},
        {"p", params}
        };

        casadi::Dict opts;
        opts["ipopt.max_iter"] = 500;
        opts["ipopt.linear_solver"] = "mumps";
        opts["ipopt.print_level"] = 0;
        opts["print_time"] = false;

        solver_ = casadi::nlpsol("solver", "ipopt", nlp, opts);
    };
    MinimumVel() { ; };

    void Solve(const Eigen::Matrix<double, 3, Eigen::Dynamic>& start_state,
        const Eigen::Matrix<double, 3, Eigen::Dynamic>& end_state,
        double& T,
        Eigen::Matrix<double, 6, Eigen::Dynamic>& C) {

        std::vector<double> param;
        for (int i = 0; i < start_state.rows(); i++) {
            for (int j = 0; j < start_state.cols(); j++) {
                param.push_back(start_state(i, j));
            }
        }
        for (int i = 0; i < end_state.rows(); i++) {
            for (int j = 0; j < end_state.cols(); j++) {
                param.push_back(end_state(i, j));
            }
        }

        double T0 = 1.0;
        casadi::DMDict args = {
            {"x0", T0},
            {"lbg", lbg_},
            {"ubg", ubg_},
            {"p", param}
        };
        casadi::DMDict res = solver_(args);
        T = static_cast<double>(res["x"](0));

        Eigen::VectorXd q0 = start_state.row(0);
        Eigen::VectorXd v0 = start_state.row(1);
        Eigen::VectorXd a0 = start_state.row(2);
        Eigen::VectorXd qe = end_state.row(0);
        Eigen::VectorXd ve = end_state.row(1);
        Eigen::VectorXd ae = end_state.row(2);
        Eigen::VectorXd alpha = (-30.0 * T * T * a0 + 30.0 * T * T * ae - 180.0 * T * v0 - 180.0 * T * ve - 360.0 * q0 + 360.0 * qe) / std::pow(T, 5);
        Eigen::VectorXd beta = 12.0 * (3.0 * T * T * a0 - 2.0 * T * T * ae + 16.0 * T * v0 + 14.0 * T * ve + 30.0 * q0 - 30.0 * qe) / std::pow(T, 4);
        Eigen::VectorXd gamma = (-9.0 * T * T * a0 + 3 * T * T * ae - 36.0 * T * v0 - 24.0 * T * ve - 60.0 * q0 + 60.0 * qe) / std::pow(T, 3);

        Eigen::MatrixXd C_matrix(6, model_.nq);
        C_matrix.row(5) = alpha / 60.0;
        C_matrix.row(4) = beta / 24.0;
        C_matrix.row(3) = gamma / 6.0;
        C_matrix.row(2) = a0 / 2.0;
        C_matrix.row(1) = v0;
        C_matrix.row(0) = q0;
        C = C_matrix;
    }

private:
    pinocchio::Model model_;
    std::shared_ptr<JacobianCallback> jac_callback_;
    casadi::Function get_jacobian_;

    double w_qj_ = 0.5;
    double w_pv_ = 0.5;
    double w_T_ = 0.5;

    // casadi::SXDict nlp_;
    std::vector<double> lbg_, ubg_;
    casadi::Function solver_;
};



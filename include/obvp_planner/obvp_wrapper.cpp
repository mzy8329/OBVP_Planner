#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "obvp_planner.hpp"
namespace py = pybind11;

PYBIND11_MODULE(obvp_planner, m) {
    m.doc() = "OBVP Planner for Python";

    py::class_<ObvpPlanner>(m, "ObvpPlanner")
        .def(py::init<
            const Eigen::MatrixXd&,
            int,
            const Eigen::VectorXd&,
            const Eigen::VectorXd&,
            double>(),
            py::arg("initial_state"),
            py::arg("dof"),
            py::arg("max_vel"),
            py::arg("max_acc"),
            py::arg("weight_T") = 1.0,
            "ObvpPlanner constructor\n"
            "Args:\n"
            "    initial_state: Initial state matrix (3 x DOF)\n"
            "    dof: Degrees of freedom\n"
            "    max_vel: Maximum velocity limits (DOF-length vector)\n"
            "    max_acc: Maximum acceleration limits (DOF-length vector)\n"
            "    weight_T: Time cost weight (default=1.0)")

        .def("getCurrentOutput_EP", &ObvpPlanner::getCurrentOutput_EP,
            py::arg("target_state"),
            py::arg("dt"),
            "Compute next output state\n"
            "Args:\n"
            "    target_state: Target state matrix (1xDOF)\n"
            "    dt: Time step\n"
            "Returns:\n"
            "    Current position vector (DOF-length)")

        .def("getCurrentOutput_EPV", &ObvpPlanner::getCurrentOutput_EPV,
            py::arg("target_state"),
            py::arg("dt"),
            "Compute next output state\n"
            "Args:\n"
            "    target_state: Target state matrix (2xDOF)\n"
            "    dt: Time step\n"
            "Returns:\n"
            "    Current position vector (DOF-length)")

        .def("getCurrentOutput_EPVA", &ObvpPlanner::getCurrentOutput_EPVA,
            py::arg("target_state"),
            py::arg("dt"),
            "Compute next output state\n"
            "Args:\n"
            "    target_state: Target state matrix (3xDOF)\n"
            "    dt: Time step\n"
            "Returns:\n"
            "    Current position vector (DOF-length)")


        .def("getCurrentState", &ObvpPlanner::getCurrentState,
            "Get full current state\n"
            "Returns:\n"
            "    State matrix (3xDOF) with positions, velocities, accelerations")

        .def("getT", &ObvpPlanner::getT,
            "Get T\n"
            "Returns:\n"
            "    T")
        .def("getResolution", &ObvpPlanner::getResolution,
            "Get Resolution\n"
            "Returns:\n"
            "    Resolution");
}

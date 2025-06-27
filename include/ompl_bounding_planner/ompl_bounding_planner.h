// Copyright 2025 Tencent. All rights reserved.
#ifndef MANIPULATION_PLANNERS_OMPL_BOUNDING_PLANNER_H_
#define MANIPULATION_PLANNERS_OMPL_BOUNDING_PLANNER_H_

#include <atomic>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <orocos_kdl/src/chain.hpp>
#include <orocos_kdl/src/chainfksolver.hpp>
#include <orocos_kdl/src/chainfksolverpos_recursive.hpp>
#include <orocos_kdl/src/tree.hpp>

#include "ros/third_party/kdl_parser/kdl_parser/include/kdl_parser/kdl_parser.hpp"
#include "third_party/fmt/include/fmt/format.h"
#include "third_party/glog/src/glog/logging.h"

#include "math/rigid_transform.h"
#include "math/rigid_transform_yaml.h"
#include "third_party/eigen/Eigen/Core"

#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"

#include "base/scoped_parameter.h"
#include "base/scoped_temporary_workspace.h"

#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/geometry.hpp"

namespace manipulation {

  // This is a path planning method for collision avoidance in the robot's joint space, where obstacles are simply
  // represented using bounding boxes.
  class OmplBoundingPlanner {
  public:
    explicit OmplBoundingPlanner(const std::string& urdf_file_path, const std::string& srdf_file_path, int dof, const std::string& chain_start,
      const std::string& chain_end, double ee_length, double expand_distance = 0.1);
    ~OmplBoundingPlanner();

    math::RigidTransformd CalcSpecificJointPose(const Eigen::VectorXd& q, int q_index) const;
    void addInwardBoundingBox(const Eigen::Vector3d& min_bounding, const Eigen::Vector3d& max_bounding);
    void addInwardBoundingBox(const Eigen::VectorXd& start_q, const Eigen::VectorXd& end_q, double expand_distance = 0.3);
    void addOutwardBoundingBox(const Eigen::Vector3d& min_bounding, const Eigen::Vector3d& max_bounding);

    void clearBoundingBox() {
      inward_bounding_boxes_.clear();
      outward_bounding_boxes_.clear();
    }

    std::vector<Eigen::VectorXd> Plan(const Eigen::VectorXd& current_q, const Eigen::VectorXd& target_q);

  private:
    std::string getTempUrdfWithEE(const std::string& origin_urdf, double length);
    std::optional<math::RigidTransformd> ComputeChainPose(const std::string& link_start, const std::string& link_end,
      const Eigen::VectorXd& q) const;
    KDL::Chain GetKdlChain(const std::string& chain_start, const std::string& chain_end) const;

    bool isPoseValid(const Eigen::Vector3d& pose) const;
    bool isStateValid(const ompl::base::State* state);

  private:
    std::string urdf_file_absolute_path_;
    int dof_ = 7;
    std::string chain_start_;
    std::string chain_end_;
    KDL::Tree kdl_tree_;
    KDL::Chain start_end_chain_;
    base::ScopedTemporaryWorkspace config_space_;

    std::shared_ptr<ompl::base::RealVectorStateSpace> space_;
    std::shared_ptr<ompl::geometric::SimpleSetup> ss_;
    std::vector<Eigen::Matrix<double, 3, 2>> inward_bounding_boxes_;
    std::vector<Eigen::Matrix<double, 3, 2>> outward_bounding_boxes_;
    double expand_distance_ = 0.5;
    double max_planning_time_ = 10.0;
    int sample_num_ = 10;

    pinocchio::Model model_;
    pinocchio::GeometryModel geometry_model_;
    pinocchio::Data model_data_;
    std::shared_ptr<pinocchio::GeometryData> model_geometry_data_ptr_;
  };
}  // namespace manipulation

#endif  // MANIPULATION_PLANNERS_OMPL_BOUNDING_PLANNER_H_

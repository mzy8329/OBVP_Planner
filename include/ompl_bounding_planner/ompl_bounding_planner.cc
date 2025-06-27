// Copyright 2025 Tencent. All rights reserved.
#include "manipulation/planners/ompl_bounding_planner.h"

#include <tinyxml2.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <tinyxml2.h>

#include "drivers/manipulators/jaka/jaka_constants.h"

#include "math/rigid_transform.h"
#include "ros/third_party/geometry/eigen_conversions/include/eigen_conversions/eigen_kdl.h"
#include "eigen/Eigen/Core"
#include "glog/src/glog/logging.h"


namespace manipulation {

  OmplBoundingPlanner::OmplBoundingPlanner(const std::string& urdf_file_path, const std::string& srdf_file_path, int dof, const std::string& chain_start,
    const std::string& chain_end, double ee_length, double expand_distance)
    : dof_(dof), chain_start_(chain_start), chain_end_(chain_end), expand_distance_(expand_distance) {
    const char* config_root = std::getenv("ROBOTICS_CONFIG_ROOT");
    CHECK(config_root != nullptr);
    urdf_file_absolute_path_ = fmt::format("{}/{}", config_root, urdf_file_path);
    std::string modified_urdf = getTempUrdfWithEE(urdf_file_absolute_path_, ee_length);
    std::string file_name = "ompl_temp_urdf.urdf";
    config_space_.CreateFile(file_name, modified_urdf);
    std::string temp_file_path = fmt::format("{}/{}", config_space_.GetRootDirectory(), file_name);
    CHECK(kdl_parser::treeFromFile(temp_file_path, kdl_tree_))
      << "Failed to parse KDL tree from " << temp_file_path;
    ompl::base::RealVectorBounds bounds(dof_);
    for (int i = 0; i < dof_; ++i) {
      bounds.setHigh(i, drivers::manipulators::jaka::kJointLimits[i].upper_position);
      bounds.setLow(i, drivers::manipulators::jaka::kJointLimits[i].lower_position);
    }

    space_ = std::make_shared<ompl::base::RealVectorStateSpace>(dof_);
    space_->setBounds(bounds);
    ss_ = std::make_shared<ompl::geometric::SimpleSetup>(space_);
    ss_->setStateValidityChecker([this](const ompl::base::State* state) { return isStateValid(state); });
    ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    pinocchio::urdf::buildModel(temp_file_path, model_);
    pinocchio::urdf::buildGeom(model_, temp_file_path, pinocchio::COLLISION, geometry_model_, { static_cast<const std::string>("") });
    geometry_model_.addAllCollisionPairs();
    if (srdf_file_path != "") {
      pinocchio::srdf::removeCollisionPairs(model_, geometry_model_, fmt::format("{}/{}", config_root, srdf_file_path));
    }
    model_data_ = pinocchio::Data(model_);
    model_geometry_data_ptr_ = std::make_shared<pinocchio::GeometryData>(geometry_model_);
  }
  OmplBoundingPlanner::~OmplBoundingPlanner() { ; }

  math::RigidTransformd OmplBoundingPlanner::CalcSpecificJointPose(const Eigen::VectorXd& q, int q_index) const {
    std::string end_link_name;
    if (q_index != dof_ - 1) {
      end_link_name = drivers::manipulators::jaka::ToString(static_cast<drivers::manipulators::jaka::Link>(q_index + 1));
    }
    else {
      end_link_name = "LINK_EE";
    }

    std::optional<math::RigidTransformd> pose = ComputeChainPose(chain_start_, end_link_name, q.head(q_index + 1));
    return pose.value();
  }

  void OmplBoundingPlanner::addInwardBoundingBox(const Eigen::Vector3d& min_bounding,
    const Eigen::Vector3d& max_bounding) {
    Eigen::Matrix<double, 3, 2> inward_box;
    inward_box.col(0) = min_bounding.array() + expand_distance_;
    inward_box.col(1) = max_bounding.array() - expand_distance_;
    inward_bounding_boxes_.emplace_back(inward_box);
  }

  void OmplBoundingPlanner::addOutwardBoundingBox(const Eigen::Vector3d& min_bounding,
    const Eigen::Vector3d& max_bounding) {
    Eigen::Matrix<double, 3, 2> outward_box;
    outward_box.col(0) = min_bounding.array() - expand_distance_;
    outward_box.col(1) = max_bounding.array() + expand_distance_;
    outward_bounding_boxes_.emplace_back(outward_box);
  }

  void OmplBoundingPlanner::addInwardBoundingBox(const Eigen::VectorXd& start_q, const Eigen::VectorXd& end_q, double expand_distance) {
    math::RigidTransformd start_pose, end_pose;
    Eigen::Vector3d min_bounding, max_bounding;
    for (int i = 0; i < dof_; i++) {
      start_pose = CalcSpecificJointPose(start_q, i);
      end_pose = CalcSpecificJointPose(end_q, i);

      min_bounding(0) = std::min(
        std::min(start_pose.translation()[0], end_pose.translation()[0]) - expand_distance, min_bounding(0));
      min_bounding(1) = std::min(
        std::min(start_pose.translation()[1], end_pose.translation()[1]) - expand_distance, min_bounding(1));
      min_bounding(2) = std::min(
        std::min(start_pose.translation()[2], end_pose.translation()[2]) - expand_distance, min_bounding(2));

      max_bounding(0) = std::max(
        std::max(start_pose.translation()[0], end_pose.translation()[0]) + expand_distance, max_bounding(0));
      max_bounding(1) = std::max(
        std::max(start_pose.translation()[1], end_pose.translation()[1]) + expand_distance, max_bounding(1));
      max_bounding(2) = std::max(
        std::max(start_pose.translation()[2], end_pose.translation()[2]) + expand_distance, max_bounding(2));
    }
    addInwardBoundingBox(min_bounding, max_bounding);
  }

  std::string OmplBoundingPlanner::getTempUrdfWithEE(const std::string& origin_urdf, double length) {
    std::ifstream file(origin_urdf);
    std::ostringstream buffer;
    buffer << file.rdbuf();
    std::string urdf_content = buffer.str();

    tinyxml2::XMLDocument doc;
    if (doc.Parse(urdf_content.c_str()) != tinyxml2::XML_SUCCESS) {
      throw std::runtime_error("Failed to parse URDF content");
    }
    tinyxml2::XMLElement* robot = doc.RootElement();

    tinyxml2::XMLElement* new_link = doc.NewElement("link");
    new_link->SetAttribute("name", "LINK_EE");
    tinyxml2::XMLElement* visual = doc.NewElement("visual");
    tinyxml2::XMLElement* geometry = doc.NewElement("geometry");
    tinyxml2::XMLElement* box = doc.NewElement("box");
    box->SetAttribute("size", "0.15 0.15 0.15");
    geometry->InsertEndChild(box);
    visual->InsertEndChild(geometry);
    new_link->InsertEndChild(visual);
    robot->InsertEndChild(new_link);

    tinyxml2::XMLElement* new_joint = doc.NewElement("joint");
    new_joint->SetAttribute("name", "J_EE");
    new_joint->SetAttribute("type", "fixed");

    tinyxml2::XMLElement* parent = doc.NewElement("parent");
    parent->SetAttribute("link", chain_end_.c_str());
    new_joint->InsertEndChild(parent);

    tinyxml2::XMLElement* child = doc.NewElement("child");
    child->SetAttribute("link", "LINK_EE");
    new_joint->InsertEndChild(child);

    tinyxml2::XMLElement* origin = doc.NewElement("origin");
    origin->SetAttribute("xyz", (std::to_string(0.0) + " "
      + std::to_string(0.0) + " "
      + std::to_string(length)).c_str());
    origin->SetAttribute("rpy", "0 0 0");
    new_joint->InsertEndChild(origin);
    robot->InsertEndChild(new_joint);

    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);
    return std::string(printer.CStr());
  }

  std::optional<math::RigidTransformd> OmplBoundingPlanner::ComputeChainPose(const std::string& link_start,
    const std::string& link_end,
    const Eigen::VectorXd& q) const {
    KDL::Chain chain;
    if (!kdl_tree_.getChain(link_start, link_end, chain)) {
      LOG(WARNING) << "Can not find chain for link_start " << link_start << " link_end " << link_end;
      return std::nullopt;
    }
    auto fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);
    KDL::Frame compute_frame;
    KDL::JntArray jnt_array;
    jnt_array.data = q;
    CHECK(fk_solver->JntToCart(jnt_array, compute_frame) == 0) << "q " << q.transpose();
    Eigen::Affine3d transform;
    tf::transformKDLToEigen(compute_frame, transform);
    math::RigidTransformd pose(transform.matrix());
    VLOG(1) << "link_start " << link_start << " link_end " << link_end << " q " << q.transpose() << " \nlocal_pose "
      << pose;
    return pose;
  }

  KDL::Chain OmplBoundingPlanner::GetKdlChain(const std::string& chain_start, const std::string& chain_end) const {
    KDL::Chain chain;
    CHECK(kdl_tree_.getChain(chain_start, chain_end, chain))
      << "Can not get the KDL chain with start=" << chain_start << " end=" << chain_end;
    return chain;
  }

  bool OmplBoundingPlanner::isPoseValid(const Eigen::Vector3d& pose) const {
    bool inBox = false;
    for (const auto& inward_box : inward_bounding_boxes_) {
      if ((pose - inward_box.col(1)).maxCoeff() < 0 && (inward_box.col(0) - pose).maxCoeff() < 0) {
        inBox = true;
        break;
      }
    }
    if (!inBox) { return false; }

    for (const auto& outward_box : outward_bounding_boxes_) {
      if ((pose - outward_box.col(1)).maxCoeff() < 0 && (outward_box.col(0) - pose).maxCoeff() < 0) {
        return false;
      }
    }
    return true;
  }

  bool OmplBoundingPlanner::isStateValid(const ompl::base::State* ompl_state) {
    if (inward_bounding_boxes_.size() + outward_bounding_boxes_.size() == 0) return true;
    auto* rv_state = ompl_state->as<ompl::base::RealVectorStateSpace::StateType>();
    Eigen::VectorXd state(dof_);
    for (int i = 0; i < dof_; i++) {
      state[i] = rv_state->values[i];
    }

    pinocchio::forwardKinematics(model_, model_data_, state);
    if (pinocchio::computeCollisions(model_, model_data_, geometry_model_, *model_geometry_data_ptr_, state, true)) {
      return false;
    }

    Eigen::Vector3d last_pose = CalcSpecificJointPose(state, 0).translation();
    if (!isPoseValid(last_pose)) {
      return false;
    }
    for (int i = 1; i < dof_; i++) {
      Eigen::Vector3d pose = CalcSpecificJointPose(state, i).translation();
      Eigen::Vector3d d_p = (pose - last_pose) / static_cast<double>(sample_num_);
      for (int item = 1; item <= sample_num_; item++) {
        Eigen::Vector3d temp_pose = last_pose + item * d_p;
        if (!isPoseValid(temp_pose)) {
          return false;
        }
      }
      last_pose = pose;
    }
    return true;
  }

  std::vector<Eigen::VectorXd> OmplBoundingPlanner::Plan(const Eigen::VectorXd& current_q,
    const Eigen::VectorXd& target_q) {
    if (inward_bounding_boxes_.empty()) {
      addInwardBoundingBox(current_q, target_q, 3 * expand_distance_);
    }

    ompl::base::ScopedState<> start_state(space_);
    ompl::base::ScopedState<> end_state(space_);
    for (int i = 0; i < dof_; i++) {
      start_state[i] = current_q[i];
      end_state[i] = target_q[i];
    }
    ss_->setStartAndGoalStates(start_state, end_state);
    ss_->setPlanner(std::make_shared<ompl::geometric::RRTConnect>(ss_->getSpaceInformation()));
    ompl::base::PlannerStatus solved = ss_->solve(max_planning_time_);
    if (solved) {
      ss_->getSolutionPath().interpolate();
      ompl::geometric::PathSimplifier simplify_path(ss_->getSpaceInformation());
      simplify_path.simplify(ss_->getSolutionPath(), true);
      ss_->getSolutionPath().interpolate();  // For multipoints interpolation
      ompl::geometric::PathGeometric& path = ss_->getSolutionPath();

      std::vector<Eigen::VectorXd> trajectory;
      for (size_t i = 0; i < path.getStateCount(); ++i) {
        const ompl::base::State* state = path.getState(i);
        const auto* rv_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
        Eigen::VectorXd point(dof_);
        for (int j = 0; j < dof_; ++j) {
          point[j] = rv_state->values[j];
        }
        trajectory.emplace_back(std::move(point));
      }
      return trajectory;
    }
    return std::vector<Eigen::VectorXd>();
  }
}  // namespace manipulation

#include <rclcpp/rclcpp.hpp>

#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/detail/common.h>

namespace py = pybind11;

/*
  * This class acts as a interface for allowing python codes to access c++ functions
  * for computing forward kinematics (KDL based) and inverse kinematics (Trac IK based)
  */
class TrackIKBindings
{
public:
  /*
    * @param node_name(string) - name to be used by logger
    * @param urdf(string) - urdf of the robot arm for which kinematics will be computed
    * @param base_link(string) - frame to be considered as base of the robot
    * @param tip_link(string) - frame to be considered as tool tip of the robot
  */
  TrackIKBindings(
    const std::string& node_name,
    const std::string& urdf,
    const std::string& base_link,
    const std::string& tip_link)
  {
    chain_ptr_ = std::make_unique<KDL::Chain>();

    node_name_ = node_name;

    urdf::Model model;
    if (!model.initString(urdf))
    {
      RCLCPP_INFO(rclcpp::get_logger(node_name_), "cannot load urdf model");
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree))
    {
      RCLCPP_INFO(rclcpp::get_logger(node_name_), "cannot create tree from urdf model");
    }

    if (!tree.getChain(base_link, tip_link, *chain_ptr_))
    {
      RCLCPP_INFO(rclcpp::get_logger(node_name_), "cannot create chain from tree");
    }

    trac_ik_solver_ptr_ = std::make_unique<TRAC_IK::TRAC_IK>(
      base_link, tip_link, urdf, 0.005, 1e-5);
  }

  /*
    * @brief method for computing forward kinematics
    * @param joints(Eigen::VectorXd) - joint angles in radians for robot arm
    * @return Eigen::VectorXd - cartesian position of the robot tool tip with respect to base
  */
  Eigen::VectorXd perform_forward_kinematics(
    const Eigen::VectorXd joints)
  {
    Eigen::VectorXd point(7);
    point(0) = 0.0;
    point(1) = 0.0;
    point(2) = 0.0;
    point(3) = 0.0;
    point(4) = 0.0;
    point(5) = 0.0;
    point(6) = 0.0;

    KDL::JntArray joint_array(chain_ptr_->getNrOfJoints());

    for (unsigned int i = 0; i < joints.size(); i++)
    {
      joint_array(i) = joints[i];
    }

    KDL::ChainFkSolverPos_recursive fk_solver(*chain_ptr_);
    KDL::Frame end_effector_pose;
    if (fk_solver.JntToCart(joint_array, end_effector_pose) >= 0)
    {
      point(0) = end_effector_pose.p.x();
      point(1) = end_effector_pose.p.y();
      point(2) = end_effector_pose.p.z();
      double rx, ry, rz, rw;
      end_effector_pose.M.GetQuaternion(rx, ry, rz, rw);
      point(3) = rx;
      point(4) = ry;
      point(5) = rz;
      point(6) = rw;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger(node_name_), "cannot perform forward kinematics");
    }

    return point;
  }

  /*
    * @brief method for computing inverse kinematics
    * @param current_joints(Eigen::VectorXd) - current joint angles in radians for robot arm (seeding)
    * @param cartesian_position(Eigen::VectorXd) - cartesian pose of tool tip of robot arm with respect to base (seeding)
    * @return Eigen::VectorXd - joint position for robot arm
  */
  Eigen::VectorXd perform_inverse_kinematics_trac_ik(
    const Eigen::VectorXd current_joints,
    const Eigen::VectorXd cartesian_position)
  {
    Eigen::VectorXd joint(chain_ptr_->getNrOfJoints());
    joint(0) = 0.0;
    joint(1) = 0.0;
    joint(2) = 0.0;
    joint(3) = 0.0;
    joint(4) = 0.0;
    joint(5) = 0.0;

    double x = cartesian_position(0);
    double y = cartesian_position(1);
    double z = cartesian_position(2);
    double rx = cartesian_position(3);
    double ry = cartesian_position(4);
    double rz = cartesian_position(5);

    KDL::Vector kdl_vector(
      cartesian_position(0),
      cartesian_position(1),
      cartesian_position(2));
  
    double angle = std::sqrt(rx*rx + ry*ry + rz*rz);
    KDL::Rotation kdl_rotation;

    if (angle < 1e-9)
    {
      kdl_rotation = KDL::Rotation::Identity();
    }
    else
    {
      kdl_rotation = KDL::Rotation::Rot(
        KDL::Vector(rx/angle, ry/angle, rz/angle), angle);
    }
    
    KDL::Frame desired_pose(kdl_rotation, kdl_vector);

    KDL::JntArray q_init(chain_ptr_->getNrOfJoints());
    for (unsigned int i = 0; i < chain_ptr_->getNrOfJoints(); i++)
    {
      q_init(i) = current_joints(i);
    }

    KDL::JntArray q_result;
    int rc = trac_ik_solver_ptr_->CartToJnt(q_init, desired_pose, q_result);

    if (rc >= 0)
    {
      for (unsigned int i = 0; i < q_result.rows(); ++i)
      {
        joint[i] = q_result(i);
      }
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger(node_name_), "cannot perform inverse kinematics trac ik");
    }
    
    return joint;
  }

  /*
   * @brief function for getting joints limits pre-set via urdf
   * @return std::vector<Eigen::VectorXd> - vector of joint limits in radians (lower bound, upper bound)
  */
  std::vector<Eigen::VectorXd> get_joint_limits()
  {
    std::vector<Eigen::VectorXd> joint_limits;

    KDL::JntArray 
      current_lb(chain_ptr_->getNrOfJoints()),
      current_ub(chain_ptr_->getNrOfJoints());
    if (!trac_ik_solver_ptr_->getKDLLimits(current_lb, current_ub))
    {
      RCLCPP_ERROR(rclcpp::get_logger(node_name_), "failed to get joint limits");
    }
    else
    {
      Eigen::VectorXd lb(chain_ptr_->getNrOfJoints());
      Eigen::VectorXd ub(chain_ptr_->getNrOfJoints());

      for (unsigned int i = 0; i < chain_ptr_->getNrOfJoints(); i++)
      {
        lb(i) = current_lb(i);
        ub(i) = current_ub(i);
      }
      joint_limits.push_back(lb);
      joint_limits.push_back(ub);
    }

    return joint_limits;
  }

  /*
   * @brief function for setting limits for joints
   * @param upper_boundary(Eigen::VectorXd) - upper limit for the joints
   * @param lower boundary(Eigen::VectorXd) - lower limit for the joints  
   */
  bool set_joint_limits(
    Eigen::VectorXd lower_boundary,
    Eigen::VectorXd upper_boundary)
  {
    KDL::JntArray 
      current_lb(chain_ptr_->getNrOfJoints()),
      current_ub(chain_ptr_->getNrOfJoints());
    
    for (unsigned int i = 0; i < lower_boundary.rows(); i++)
    {
      current_lb(i) = lower_boundary(i);
      current_ub(i) = upper_boundary(i);
    }

    if (!trac_ik_solver_ptr_->setKDLLimits(current_lb, current_ub))
    {
      RCLCPP_ERROR(rclcpp::get_logger(node_name_), "failed to set joint limits");
      return false;
    }
    return true;
  }

private:
  std::unique_ptr<KDL::Chain> chain_ptr_;
  std::unique_ptr<TRAC_IK::TRAC_IK> trac_ik_solver_ptr_;
  std::string node_name_;
};


PYBIND11_MODULE(trac_ik_python_module, m)
{
  m.doc() = "Track IK Solver access to python";

  py::class_<TrackIKBindings>(m, "TrackIKBindings")
    .def(py::init<const std::string&, const std::string&, const std::string&, const std::string&>(), 
      py::arg("node_name"), py::arg("urdf"), py::arg("base_link"), py::arg("tip_link"), "initialise the KDL chain")
    .def("perform_forward_kinematics", &TrackIKBindings::perform_forward_kinematics,
      py::arg("joints"), "return cartesian poses")
    .def("perform_inverse_kinematics_trac_ik", &TrackIKBindings::perform_inverse_kinematics_trac_ik,
      py::arg("current_joints"), py::arg("cartesian_position"), "return joint values")
    .def("get_joint_limits", &TrackIKBindings::get_joint_limits, "return limits for joints")
    .def("set_joint_limits", &TrackIKBindings::set_joint_limits,
      py::arg("lower_boundary"), py::arg("upper_boundary"), "set joint limits");
}

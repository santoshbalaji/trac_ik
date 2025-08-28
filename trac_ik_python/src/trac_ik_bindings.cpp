#include <rclcpp/rclcpp.hpp>

#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>

#include <Eigen/Dense>
#include <memory>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/detail/common.h>

namespace py = pybind11;

class TrackIKBindings
{
public:
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

    KDL::Vector kdl_vector(
      cartesian_position(0),
      cartesian_position(1),
      cartesian_position(2));

    KDL::Rotation kdl_rotation = KDL::Rotation::Quaternion(
      cartesian_position(3),
      cartesian_position(4),
      cartesian_position(5),
      cartesian_position(6));
    
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
      py::arg("current_joints"), py::arg("cartesian_position"), "return joint values");
}

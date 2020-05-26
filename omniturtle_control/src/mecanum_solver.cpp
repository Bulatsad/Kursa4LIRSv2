#include"../include/mecanum_solver.h"

size_t OmniTurtle::MecanumSolver::find_joint(const sensor_msgs::JointState &jsm,
                                             std::string joint_name) const
{
  size_t index = 0;

  auto it = std::find(jsm.name.begin(), jsm.name.end(), joint_name);
  if (it != jsm.name.end())
    index = std::distance(jsm.name.begin(), it);
  else
    throw std::invalid_argument("MecanumSolverError: " + joint_name + "isn't found");

  return index;
}


const sensor_msgs::JointState
  OmniTurtle::MecanumSolver::moveForward(const sensor_msgs::JointState &jsm, double delta) const
{
  if(jsm.name.size() != JOINT_NUM)
    throw std::length_error("MecanumSolver::moveForwardError: invalid message" +
                            std::to_string(jsm.name.size()));

  sensor_msgs::JointState result = jsm;

  size_t index = find_joint(result, "mecanum_rf_joint");
  result.position.at(index) += 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_lf_joint");
  result.position.at(index) += 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_rb_joint");
  result.position.at(index) += 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_lb_joint");
  result.position.at(index) += 2*delta/WHEEL_DIAMETER;

  return result;
}

const sensor_msgs::JointState
  OmniTurtle::MecanumSolver::moveBackward(const sensor_msgs::JointState &jsm, double delta) const
{
  if(jsm.name.size() != JOINT_NUM)
    throw std::length_error("MecanumSolver::moveForwardError: invalid message" +
                            std::to_string(jsm.name.size()));
  sensor_msgs::JointState result = jsm;

  size_t index = find_joint(result, "mecanum_rf_joint");
  result.position.at(index) -= 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_lf_joint");
  result.position.at(index) -= 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_rb_joint");
  result.position.at(index) -= 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_lb_joint");
  result.position.at(index) -= 2*delta/WHEEL_DIAMETER;

  return result;
}

const sensor_msgs::JointState
  OmniTurtle::MecanumSolver::moveRight(const sensor_msgs::JointState &jsm, double delta) const
{
  if(jsm.name.size() != JOINT_NUM)
    throw std::length_error("MecanumSolver::moveForwardError: invalid message" +
                            std::to_string(jsm.name.size()));

  sensor_msgs::JointState result = jsm;

  size_t index = find_joint(result, "mecanum_rf_joint");
  result.position.at(index) -= 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_lf_joint");
  result.position.at(index) += 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_rb_joint");
  result.position.at(index) += 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_lb_joint");
  result.position.at(index) -= 2*delta/WHEEL_DIAMETER;

  return result;
}

const sensor_msgs::JointState
  OmniTurtle::MecanumSolver::moveLeft(const sensor_msgs::JointState &jsm, double delta) const
{
  if(jsm.name.size() != JOINT_NUM)
    throw std::length_error("MecanumSolver::moveForwardError: invalid message" +
                            std::to_string(jsm.name.size()));

  sensor_msgs::JointState result = jsm;

  size_t index = find_joint(result, "mecanum_rf_joint");
  result.position.at(index) += 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_lf_joint");
  result.position.at(index) -= 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_rb_joint");
  result.position.at(index) -= 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_lb_joint");
  result.position.at(index) += 2*delta/WHEEL_DIAMETER;

  return result;
}

const sensor_msgs::JointState
  OmniTurtle::MecanumSolver::moveDiagRightUp(const sensor_msgs::JointState &jsm, double delta) const
{
  if(jsm.name.size() != JOINT_NUM)
    throw std::length_error("MecanumSolver::moveForwardError: invalid message" +
                            std::to_string(jsm.name.size()));

  sensor_msgs::JointState result = jsm;

  size_t index = find_joint(result, "mecanum_lf_joint");
  result.position.at(index) += 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_rb_joint");
  result.position.at(index) += 2*delta/WHEEL_DIAMETER;

  return result;
}

const sensor_msgs::JointState
  OmniTurtle::MecanumSolver::moveDiagLeftUp(const sensor_msgs::JointState &jsm, double delta) const
{
  if(jsm.name.size() != JOINT_NUM)
    throw std::length_error("MecanumSolver::moveForwardError: invalid message" +
                            std::to_string(jsm.name.size()));

  sensor_msgs::JointState result = jsm;

  size_t index = find_joint(result, "mecanum_rf_joint");
  result.position.at(index) += 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_lb_joint");
  result.position.at(index) += 2*delta/WHEEL_DIAMETER;

  return result;
}

const sensor_msgs::JointState
  OmniTurtle::MecanumSolver::moveDiagRightDown(const sensor_msgs::JointState &jsm, double delta) const
{
  if(jsm.name.size() != JOINT_NUM)
    throw std::length_error("MecanumSolver::moveForwardError: invalid message" +
                            std::to_string(jsm.name.size()));

  sensor_msgs::JointState result = jsm;

  size_t index = find_joint(result, "mecanum_rf_joint");
  result.position.at(index) -= 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_lb_joint");
  result.position.at(index) -= 2*delta/WHEEL_DIAMETER;

  return result;
}

const sensor_msgs::JointState
  OmniTurtle::MecanumSolver::moveDiagLeftDown(const sensor_msgs::JointState &jsm, double delta) const
{
  if(jsm.name.size() != JOINT_NUM)
    throw std::length_error("MecanumSolver::moveForwardError: invalid message" +
                            std::to_string(jsm.name.size()));

  sensor_msgs::JointState result = jsm;

  size_t index = find_joint(result, "mecanum_lf_joint");
  result.position.at(index) -= 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_rb_joint");
  result.position.at(index) -= 2*delta/WHEEL_DIAMETER;

  return result;
}

const sensor_msgs::JointState
  OmniTurtle::MecanumSolver::turnAroundClockwise(const sensor_msgs::JointState &jsm, double delta) const
{
  if(jsm.name.size() != JOINT_NUM)
    throw std::length_error("MecanumSolver::moveForwardError: invalid message" +
                            std::to_string(jsm.name.size()));

  sensor_msgs::JointState result = jsm;

  size_t index = find_joint(result, "mecanum_rf_joint");
  result.position.at(index) -= 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_lf_joint");
  result.position.at(index) += 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_rb_joint");
  result.position.at(index) -= 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_lb_joint");
  result.position.at(index) += 2*delta/WHEEL_DIAMETER;

  return result;
}

const sensor_msgs::JointState
  OmniTurtle::MecanumSolver::turnAroundAnticlockwise(const sensor_msgs::JointState &jsm, double delta) const
{
  if(jsm.name.size() != JOINT_NUM)
    throw std::length_error("MecanumSolver::moveForwardError: invalid message" +
                            std::to_string(jsm.name.size()));

  sensor_msgs::JointState result = jsm;

  size_t index = find_joint(result, "mecanum_rf_joint");
  result.position.at(index) += 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_lf_joint");
  result.position.at(index) -= 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_rb_joint");
  result.position.at(index) += 2*delta/WHEEL_DIAMETER;

  index = find_joint(result, "mecanum_lb_joint");
  result.position.at(index) -= 2*delta/WHEEL_DIAMETER;

  return result;
}

const sensor_msgs::JointState
  OmniTurtle::MecanumSolver::stop(const sensor_msgs::JointState &jsm, double delta) const
{
  return jsm;
}

#ifndef MECANUM_SOLVER_H
#define MECANUM_SOLVER_H

#include"solver.h"
#include<string>

namespace OmniTurtle
{
  class MecanumSolver : public Solver
  {
  private:
    static constexpr double WHEEL_DIAMETER = 60.0; // in mm
    size_t find_joint(const sensor_msgs::JointState &jsm, std::string joint_name) const; // mecanum_rf_joint
                                                                                         // mecanum_lf_joint
                                                                                         // mecanum_rb_joint
                                                                                         // mecanum_lb_joint
  public:
    static const int JOINT_NUM = 4;

    const sensor_msgs::JointState
    moveForward(const sensor_msgs::JointState& jsm, double delta) const override;

    const sensor_msgs::JointState
    moveBackward(const sensor_msgs::JointState& jsm, double delta) const override;

    const sensor_msgs::JointState
    moveRight(const sensor_msgs::JointState& jsm, double delta) const override;

    const sensor_msgs::JointState
    moveLeft(const sensor_msgs::JointState& jsm, double delta) const override;

    const sensor_msgs::JointState
    moveDiagRightUp(const sensor_msgs::JointState& jsm, double delta) const override;

    const sensor_msgs::JointState
    moveDiagLeftUp(const sensor_msgs::JointState& jsm, double delta) const override;

    const sensor_msgs::JointState
    moveDiagRightDown(const sensor_msgs::JointState& jsm, double delta) const override;

    const sensor_msgs::JointState
    moveDiagLeftDown(const sensor_msgs::JointState& jsm, double delta) const override;

    const sensor_msgs::JointState
    turnAroundClockwise(const sensor_msgs::JointState& jsm, double delta) const override;

    const sensor_msgs::JointState
    turnAroundAnticlockwise(const sensor_msgs::JointState& jsm, double delta) const override;

    const sensor_msgs::JointState
    stop(const sensor_msgs::JointState& jsm, double delta) const override;

  };

}

#endif // MECANUM_SOLVER_H

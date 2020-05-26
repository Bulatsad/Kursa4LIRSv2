#ifndef SOLVER_H
#define SOLVER_H

/// Solver interface class

#include"sensor_msgs/JointState.h"

namespace OmniTurtle
{
  class Solver
  {
  public:
    virtual const sensor_msgs::JointState
    moveForward(const sensor_msgs::JointState& jsm, double delta) const =0; // delta - translation length in mm

    virtual const sensor_msgs::JointState
    moveBackward(const sensor_msgs::JointState& jsm, double delta) const =0;

    virtual const sensor_msgs::JointState
    moveRight(const sensor_msgs::JointState& jsm, double delta) const =0;

    virtual const sensor_msgs::JointState
    moveLeft(const sensor_msgs::JointState& jsm, double delta) const =0;

    virtual const sensor_msgs::JointState
    moveDiagRightUp(const sensor_msgs::JointState& jsm, double delta) const =0;

    virtual const sensor_msgs::JointState
    moveDiagLeftUp(const sensor_msgs::JointState& jsm, double delta) const =0;

    virtual const sensor_msgs::JointState
    moveDiagRightDown(const sensor_msgs::JointState& jsm, double delta) const =0;

    virtual const sensor_msgs::JointState
    moveDiagLeftDown(const sensor_msgs::JointState& jsm, double delta) const =0;

    virtual const sensor_msgs::JointState
    turnAroundClockwise(const sensor_msgs::JointState& jsm, double delta) const =0;

    virtual const sensor_msgs::JointState
    turnAroundAnticlockwise(const sensor_msgs::JointState& jsm, double delta) const =0;

    virtual const sensor_msgs::JointState
    stop(const sensor_msgs::JointState& jsm, double delta) const =0;

  };

}

#endif // SOLVER_H

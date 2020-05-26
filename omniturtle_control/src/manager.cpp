#include"../include/manager.h"

OmniTurtle::OmniTurtleManager::OmniTurtleManager()
  : is_running(true), freq(10.0)
{
}

void OmniTurtle::OmniTurtleManager::launch(ros::NodeHandle &node,
                                      const std::string &sub_topic_name,
                                      const MecanumSolver &ms)
{
  n = node;
  msp = &ms;

  ctrl_sub = n.subscribe(sub_topic_name, 1, &OmniTurtle::OmniTurtleManager::controlCallback, this);
  mecanum_rf_jnt_sub = n.subscribe("/omniturtle/mecanum_rf_joint_position_controller/state", 1,
                                    &OmniTurtle::OmniTurtleManager::mecanumRFCallback, this);

  mecanum_lf_jnt_sub = n.subscribe("/omniturtle/mecanum_lf_joint_position_controller/state", 1,
                                    &OmniTurtle::OmniTurtleManager::mecanumLFCallback, this);

  mecanum_rb_jnt_sub = n.subscribe("/omniturtle/mecanum_rb_joint_position_controller/state", 1,
                                    &OmniTurtle::OmniTurtleManager::mecanumRBCallback, this);

  mecanum_lb_jnt_sub = n.subscribe("/omniturtle/mecanum_lb_joint_position_controller/state", 1,
                                    &OmniTurtle::OmniTurtleManager::mecanumLBCallback, this);

  cur_jnt_state.name.push_back("mecanum_rf_joint");
  cur_jnt_state.name.push_back("mecanum_lf_joint");
  cur_jnt_state.name.push_back("mecanum_rb_joint");
  cur_jnt_state.name.push_back("mecanum_lb_joint");
  cur_jnt_state.position.resize(4, 0.0);

  mecanum_rf_jnt_pub = n.advertise<std_msgs::Float64>("/omniturtle/mecanum_rf_joint_position_controller/command", 1);
  mecanum_lf_jnt_pub = n.advertise<std_msgs::Float64>("/omniturtle/mecanum_lf_joint_position_controller/command", 1);
  mecanum_rb_jnt_pub = n.advertise<std_msgs::Float64>("/omniturtle/mecanum_rb_joint_position_controller/command", 1);
  mecanum_lb_jnt_pub = n.advertise<std_msgs::Float64>("/omniturtle/mecanum_lb_joint_position_controller/command", 1);
}

void OmniTurtle::OmniTurtleManager::controlCallback(const geometry_msgs::Twist &msg)
{
  if(cur_jnt_state.name.size() != msp->JOINT_NUM)
    throw std::runtime_error("OmniTurtleManager::controlCallbackError: invalid message length " +
                                  std::to_string(cur_jnt_state.name.size()));

  if(msg.linear.x>0 && msg.linear.y==0 && msg.angular.z==0)
    moveForward(msg.linear.x / freq * 1000.0); // velocity [m/s] -> distance [mm]

  if(msg.linear.x<0 && msg.linear.y==0 && msg.angular.z==0)
    moveBackward(-msg.linear.x / freq * 1000.0);

  if(msg.linear.x==0 && msg.linear.y>0 && msg.angular.z==0)
    moveLeft(msg.linear.y / freq * 1000.0);

  if(msg.linear.x==0 && msg.linear.y<0 && msg.angular.z==0)
    moveRight(-msg.linear.y / freq * 1000.0);

  if(msg.linear.x>0 && msg.linear.y<0 && msg.angular.z==0)
    moveDiagRightUp(msg.linear.x / freq * 1000.0);

  if(msg.linear.x>0 && msg.linear.y>0 && msg.angular.z==0)
    moveDiagLeftUp(msg.linear.x / freq * 1000.0);

  if(msg.linear.x<0 && msg.linear.y<0 && msg.angular.z==0)
    moveDiagRightDown(-msg.linear.x / freq * 1000.0);

  if(msg.linear.x<0 && msg.linear.y>0 && msg.angular.z==0)
    moveDiagLeftDown(-msg.linear.x / freq * 1000.0);

  if(msg.linear.x==0 && msg.linear.y==0 && msg.angular.z>0)
    turnAroundAnticlockwise(msg.angular.z / freq * 1000.0);

  if(msg.linear.x==0 && msg.linear.y==0 && msg.angular.z<0)
    turnAroundClockwise(-msg.angular.z / freq * 1000.0);

  if(msg.linear.x==0 && msg.linear.y==0 && msg.angular.z==0)
    stop();
}

size_t OmniTurtle::OmniTurtleManager::find_joint(const sensor_msgs::JointState &jsm,
                                             std::string joint_name) const
{
  size_t index = 0;

  auto it = std::find(jsm.name.begin(), jsm.name.end(), joint_name);
  if (it != jsm.name.end())
    index = std::distance(jsm.name.begin(), it);
  else
    throw std::invalid_argument("OmniTurtleManager: " + joint_name + "isn't found");

  return index;
}

void OmniTurtle::OmniTurtleManager::mecanumRFCallback(const control_msgs::JointControllerState& msg)
{
  size_t index = find_joint(cur_jnt_state, "mecanum_rf_joint");
  cur_jnt_state.position.at(index) = msg.process_value;
}
void OmniTurtle::OmniTurtleManager::mecanumLFCallback(const control_msgs::JointControllerState& msg)
{
  size_t index = find_joint(cur_jnt_state, "mecanum_lf_joint");
  cur_jnt_state.position.at(index) = msg.process_value;
}
void OmniTurtle::OmniTurtleManager::mecanumRBCallback(const control_msgs::JointControllerState& msg)
{
  size_t index = find_joint(cur_jnt_state, "mecanum_rb_joint");
  cur_jnt_state.position.at(index) = msg.process_value;
}
void OmniTurtle::OmniTurtleManager::mecanumLBCallback(const control_msgs::JointControllerState& msg)
{
  size_t index = find_joint(cur_jnt_state, "mecanum_lb_joint");
  cur_jnt_state.position.at(index) = msg.process_value;
}

void OmniTurtle::OmniTurtleManager::sendMessage(const sensor_msgs::JointState &jsm)
{
  std_msgs::Float64 ctrl_msg;

  size_t index = find_joint(jsm, "mecanum_rf_joint");
  ctrl_msg.data = jsm.position.at(index);
  mecanum_rf_jnt_pub.publish(ctrl_msg);

  index = find_joint(jsm, "mecanum_lf_joint");
  ctrl_msg.data = jsm.position.at(index);
  mecanum_lf_jnt_pub.publish(ctrl_msg);

  index = find_joint(jsm, "mecanum_rb_joint");
  ctrl_msg.data = jsm.position.at(index);
  mecanum_rb_jnt_pub.publish(ctrl_msg);

  index = find_joint(jsm, "mecanum_lb_joint");
  ctrl_msg.data = jsm.position.at(index);
  mecanum_lb_jnt_pub.publish(ctrl_msg);

}

void OmniTurtle::OmniTurtleManager::moveForward(double delta)
{
  sendMessage(msp->moveForward(cur_jnt_state, delta));
}

void OmniTurtle::OmniTurtleManager::moveBackward(double delta)
{
  sendMessage(msp->moveBackward(cur_jnt_state, delta));
}

void OmniTurtle::OmniTurtleManager::moveRight(double delta)
{
  sendMessage(msp->moveRight(cur_jnt_state, delta));
}

void OmniTurtle::OmniTurtleManager::moveLeft(double delta)
{
  sendMessage(msp->moveLeft(cur_jnt_state, delta));
}

void OmniTurtle::OmniTurtleManager::moveDiagRightUp(double delta)
{
  sendMessage(msp->moveDiagRightUp(cur_jnt_state, delta));
}

void OmniTurtle::OmniTurtleManager::moveDiagLeftUp(double delta)
{
  sendMessage(msp->moveDiagLeftUp(cur_jnt_state, delta));
}

void OmniTurtle::OmniTurtleManager::moveDiagRightDown(double delta)
{
  sendMessage(msp->moveDiagRightDown(cur_jnt_state, delta));
}

void OmniTurtle::OmniTurtleManager::moveDiagLeftDown(double delta)
{
  sendMessage(msp->moveDiagLeftDown(cur_jnt_state, delta));
}

void OmniTurtle::OmniTurtleManager::turnAroundClockwise(double delta)
{
  sendMessage(msp->turnAroundClockwise(cur_jnt_state, delta));
}

void OmniTurtle::OmniTurtleManager::turnAroundAnticlockwise(double delta)
{
  sendMessage(msp->turnAroundAnticlockwise(cur_jnt_state, delta));
}

void OmniTurtle::OmniTurtleManager::stop()
{
  sendMessage(msp->stop(cur_jnt_state, 0.0));
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"omniturtle_manager");
  ros::NodeHandle n;

  OmniTurtle::OmniTurtleManager otm;
  otm.launch(n, "/omniturtle_control", OmniTurtle::MecanumSolver());

  ros::spin();


  return 0;
}



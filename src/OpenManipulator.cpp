/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Limm, Hye-Jong KIM */

#include "robotis_manipulator/OpenManipulator.h"

using namespace OPEN_MANIPULATOR;
using namespace Eigen;


////////////////////////////////////////////////////////////////////////////
////////////////////////////////Basic Function//////////////////////////////
////////////////////////////////////////////////////////////////////////////

OpenManipulator::OpenManipulator() : move_time_(1.0f),
                                     control_time_(ACTUATOR_CONTROL_TIME),
                                     moving_(false),
                                     platform_(true),
                                     processing_(false)
{
  manager_ = new Manager();


}

OpenManipulator::~OpenManipulator()
{
}

void OpenManipulator::initKinematics(Kinematics *kinematics)
{
  kinematics_ = kinematics;
}

void OpenManipulator::initActuator(Actuator *actuator)
{
  actuator_ = actuator;
  platform_ = true;
}

void OpenManipulator::addDraw(Name name, Draw *draw)
{
  Draw *temp_draw = draw;

  draw_.insert(std::make_pair(name, temp_draw));
}

void OpenManipulator::initTrajectory(std::vector<double> angle_vector)
{
  joint_trajectory_ = new RM_TRAJECTORY::JointTrajectory(manipulator_.getDOF());
  task_trajectory_ = new RM_TRAJECTORY::TaskTrajectory();

  previous_goal_.position = angle_vector;
  previous_goal_.velocity.resize(manipulator_.getDOF());
  previous_goal_.acceleration.resize(manipulator_.getDOF());
  previous_goal_.pose.position.resize(3);
  previous_goal_.pose_vel.position.resize(3);
  previous_goal_.pose_acc.position.resize(3);

  start_joint_trajectory_.reserve(manipulator_.getDOF());
  goal_joint_trajectory_.reserve(manipulator_.getDOF());

  start_task_trajectory_.reserve(3);
  goal_task_trajectory_.reserve(3);

}

void OpenManipulator::addWorld(Name world_name,
                               Name child_name,
                               Vector3f world_position,
                               Matrix3f world_orientation)
{
  manipulator_.addWorld(world_name, child_name, world_position, world_orientation);
}

void OpenManipulator::addComponent(Name my_name,
                                   Name parent_name,
                                   Name child_name,
                                   Vector3f relative_position,
                                   Matrix3f relative_orientation,
                                   Vector3f axis_of_rotation,
                                   int8_t actuator_id,
                                   double coefficient,
                                   double mass,
                                   Matrix3f inertia_tensor,
                                   Vector3f center_of_mass)
{
  manipulator_.addComponent(my_name, parent_name, child_name, relative_position, relative_orientation, axis_of_rotation, actuator_id, coefficient, mass, inertia_tensor, center_of_mass);
}

void OpenManipulator::addTool(Name my_name,
                              Name parent_name,
                              Vector3f relative_position,
                              Matrix3f relative_orientation,
                              int8_t tool_id,
                              double coefficient,
                              double mass,
                              Matrix3f inertia_tensor,
                              Vector3f center_of_mass)
{
  manipulator_.addTool(my_name, parent_name, relative_position, relative_orientation, tool_id, coefficient, mass, inertia_tensor, center_of_mass);
}

void OpenManipulator::addComponentChild(Name my_name, Name child_name)
{
  manipulator_.addComponentChild(my_name, child_name);
}

void OpenManipulator::checkManipulatorSetting()
{
  manipulator_.checkManipulatorSetting();
}

void OpenManipulator::setWorldPose(Pose world_pose)
{
  manipulator_.setWorldPose(world_pose);
}

void OpenManipulator::setWorldPosition(Vector3f world_position)
{
  manipulator_.setWorldPosition(world_position);
}

void OpenManipulator::setWorldOrientation(Matrix3f world_orientation)
{
  manipulator_.setWorldOrientation(world_orientation);
}

void OpenManipulator::setWorldState(State world_state)
{
  manipulator_.setWorldState(world_state);
}

void OpenManipulator::setWorldVelocity(VectorXf world_velocity)
{
  manipulator_.setWorldVelocity(world_velocity);
}

void OpenManipulator::setWorldAcceleration(VectorXf world_acceleration)
{
  manipulator_.setWorldAcceleration(world_acceleration);
}

void OpenManipulator::setComponent(Name name, Component component)
{
  manipulator_.setComponent(name, component);
}

void OpenManipulator::setComponentPoseToWorld(Name name, Pose pose_to_world)
{
  manipulator_.setComponentPoseToWorld(name, pose_to_world);
}

void OpenManipulator::setComponentPositionToWorld(Name name, Vector3f position_to_world)
{
  manipulator_.setComponentPositionToWorld(name, position_to_world);
}

void OpenManipulator::setComponentOrientationToWorld(Name name, Matrix3f orientation_to_wolrd)
{
  manipulator_.setComponentOrientationToWorld(name, orientation_to_wolrd);
}

void OpenManipulator::setComponentStateToWorld(Name name, State state_to_world)
{
  manipulator_.setComponentStateToWorld(name, state_to_world);
}

void OpenManipulator::setComponentVelocityToWorld(Name name, VectorXf velocity)
{
  manipulator_.setComponentVelocityToWorld(name, velocity);
}

void OpenManipulator::setComponentAccelerationToWorld(Name name, VectorXf accelaration)
{
  manipulator_.setComponentAccelerationToWorld(name, accelaration);
}

void OpenManipulator::setComponentJointAngle(Name name, double angle)
{
  manipulator_.setComponentJointAngle(name, angle);
}

void OpenManipulator::setComponentJointVelocity(Name name, double angular_velocity)
{
  manipulator_.setComponentJointVelocity(name, angular_velocity);
}

void OpenManipulator::setComponentJointAcceleration(Name name, double angular_acceleration)
{
  manipulator_.setComponentJointAcceleration(name, angular_acceleration);
}

void OpenManipulator::setComponentToolOnOff(Name name, bool on_off)
{
  manipulator_.setComponentToolOnOff(name, on_off);
}

void OpenManipulator::setComponentToolValue(Name name, double actuator_value)
{
  manipulator_.setComponentToolValue(name, actuator_value);
}

void OpenManipulator::setAllActiveJointAngle(std::vector<double> angle_vector)
{
  manipulator_.setAllActiveJointAngle(angle_vector);
}

int8_t OpenManipulator::getDOF()
{
  return manipulator_.getDOF();
}

int8_t OpenManipulator::getComponentSize()
{
  return manipulator_.getComponentSize();
}

Name OpenManipulator::getWorldName()
{
  return manipulator_.getWorldName();
}

Name OpenManipulator::getWorldChildName()
{
  return manipulator_.getWorldChildName();
}

Pose OpenManipulator::getWorldPose()
{
  return manipulator_.getWorldPose();
}

Vector3f OpenManipulator::getWorldPosition()
{
  return manipulator_.getWorldPosition();
}

Matrix3f OpenManipulator::getWorldOrientation()
{
  return manipulator_.getWorldOrientation();
}

State OpenManipulator::getWorldState()
{
  return manipulator_.getWorldState();
}

VectorXf OpenManipulator::getWorldVelocity()
{
  return manipulator_.getWorldVelocity();
}

VectorXf OpenManipulator::getWorldAcceleration()
{
  return manipulator_.getWorldAcceleration();
}

std::map<Name, Component> OpenManipulator::getAllComponent()
{
  return manipulator_.getAllComponent();
}

std::map<Name, Component>::iterator OpenManipulator::getIteratorBegin()
{
  return manipulator_.getIteratorBegin();
}

std::map<Name, Component>::iterator OpenManipulator::getIteratorEnd()
{
  return manipulator_.getIteratorEnd();
}

Component OpenManipulator::getComponent(Name name)
{
  return manipulator_.getComponent(name);
}

Name OpenManipulator::getComponentParentName(Name name)
{
  return manipulator_.getComponentParentName(name);
}

std::vector<Name> OpenManipulator::getComponentChildName(Name name)
{
  return manipulator_.getComponentChildName(name);
}

Pose OpenManipulator::getComponentPoseToWorld(Name name)
{
  return manipulator_.getComponentPoseToWorld(name);
}

Vector3f OpenManipulator::getComponentPositionToWorld(Name name)
{
  return manipulator_.getComponentPositionToWorld(name);
}

Matrix3f OpenManipulator::getComponentOrientationToWorld(Name name)
{
  return manipulator_.getComponentOrientationToWorld(name);
}

State OpenManipulator::getComponentStateToWorld(Name name)
{
  return manipulator_.getComponentStateToWorld(name);
}

VectorXf OpenManipulator::getComponentVelocityToWorld(Name name)
{
  return manipulator_.getComponentVelocityToWorld(name);
}

VectorXf OpenManipulator::getComponentAccelerationToWorld(Name name)
{
  return manipulator_.getComponentAccelerationToWorld(name);
}

Pose OpenManipulator::getComponentRelativePoseToParent(Name name)
{
  return manipulator_.getComponentRelativePoseToParent(name);
}

Vector3f OpenManipulator::getComponentRelativePositionToParent(Name name)
{
  return manipulator_.getComponentRelativePositionToParent(name);
}

Matrix3f OpenManipulator::getComponentRelativeOrientationToParent(Name name)
{
  return manipulator_.getComponentRelativeOrientationToParent(name);
}

Joint OpenManipulator::getComponentJoint(Name name)
{
  return manipulator_.getComponentJoint(name);
}

int8_t OpenManipulator::getComponentJointId(Name name)
{
  return manipulator_.getComponentJointId(name);
}

double OpenManipulator::getComponentJointCoefficient(Name name)
{
  return manipulator_.getComponentJointCoefficient(name);
}

Vector3f OpenManipulator::getComponentJointAxis(Name name)
{
  return manipulator_.getComponentJointAxis(name);
}

double OpenManipulator::getComponentJointAngle(Name name)
{
  return manipulator_.getComponentJointAngle(name);
}

double OpenManipulator::getComponentJointVelocity(Name name)
{
  return manipulator_.getComponentJointVelocity(name);
}

double OpenManipulator::getComponentJointAcceleration(Name name)
{
  return manipulator_.getComponentJointAcceleration(name);
}

Tool OpenManipulator::getComponentTool(Name name)
{
  return manipulator_.getComponentTool(name);
}

int8_t OpenManipulator::getComponentToolId(Name name)
{
  return manipulator_.getComponentToolId(name);
}

double OpenManipulator::getComponentToolCoefficient(Name name)
{
  return manipulator_.getComponentToolCoefficient(name);
}

bool OpenManipulator::getComponentToolOnOff(Name name)
{
  return manipulator_.getComponentToolOnOff(name);
}

double OpenManipulator::getComponentToolValue(Name name)
{
  return manipulator_.getComponentToolValue(name);
}

double OpenManipulator::getComponentMass(Name name)
{
  return manipulator_.getComponentMass(name);
}

Matrix3f OpenManipulator::getComponentInertiaTensor(Name name)
{
  return manipulator_.getComponentInertiaTensor(name);
}

Vector3f OpenManipulator::getComponentCenterOfMass(Name name)
{
  return manipulator_.getComponentCenterOfMass(name);
}

std::vector<double> OpenManipulator::getAllJointAngle()
{
  return manipulator_.getAllJointAngle();
}

std::vector<double> OpenManipulator::getAllActiveJointAngle()
{
  return manipulator_.getAllActiveJointAngle();
}

std::vector<uint8_t> OpenManipulator::getAllActiveJointID()
{
  return manipulator_.getAllActiveJointID();
}

// KINEMATICS

MatrixXf OpenManipulator::jacobian(Name tool_name)
{
  return kinematics_->jacobian(&manipulator_, tool_name);
}

void OpenManipulator::forward()
{
  return kinematics_->forward(&manipulator_);
}

void OpenManipulator::forward(Name first_component_name)
{
  return kinematics_->forward(&manipulator_, first_component_name);
}

std::vector<double> OpenManipulator::inverse(Name tool_name, Pose goal_pose)
{
  return kinematics_->inverse(&manipulator_, tool_name, goal_pose);
}

// ACTUATOR
std::vector<double> OpenManipulator::sendAllActuatorAngle(std::vector<double> radian_vector)
{
  std::vector<double> calc_angle;
  std::map<Name, Component>::iterator it;

  uint8_t index = 0;
  for (it = manipulator_.getIteratorBegin(); it != manipulator_.getIteratorEnd(); it++)
  {
    if (manipulator_.getComponentJointId(it->first) != -1)
    {
      calc_angle.push_back(radian_vector.at(index++) * manipulator_.getComponentJointCoefficient(it->first));
    }
  }

  return calc_angle;
}

std::vector<double> OpenManipulator::sendMultipleActuatorAngle(std::vector<uint8_t> active_joint_id, std::vector<double> radian_vector)
{
  std::vector<double> calc_angle;
  std::map<Name, Component>::iterator it;

  for (uint8_t index = 0; index < active_joint_id.size(); index++)
  {
    for (it = manipulator_.getIteratorBegin(); it != manipulator_.getIteratorEnd(); it++)
    {
      if (active_joint_id.at(index) == manipulator_.getComponentJointId(it->first))
      {
        calc_angle.push_back(radian_vector.at(index) * manipulator_.getComponentJointCoefficient(it->first));
        break;
      }
    }
  }
  return calc_angle;
}

double OpenManipulator::sendActuatorAngle(uint8_t active_joint_id, double radian)
{
  double calc_angle;
  std::map<Name, Component>::iterator it;

  for (it = manipulator_.getIteratorBegin(); it != manipulator_.getIteratorEnd(); it++)
  {
    if (manipulator_.getComponentJointId(it->first) == active_joint_id)
    {
      calc_angle = radian * manipulator_.getComponentJointCoefficient(it->first);
    }
  }

  return calc_angle;
}

bool OpenManipulator::sendActuatorSignal(uint8_t active_joint_id, bool onoff)
{
  return actuator_->sendActuatorSignal(active_joint_id, onoff);
}

std::vector<double> OpenManipulator::receiveAllActuatorAngle()
{
  std::vector<double> angles = actuator_->receiveAllActuatorAngle();
  std::vector<uint8_t> active_joint_id = manipulator_.getAllActiveJointID();
  std::vector<uint8_t> sorted_id = active_joint_id;

  std::vector<double> sorted_angle_vector;
  sorted_angle_vector.reserve(active_joint_id.size());

  double sorted_angle_array[angles.size()];

  std::sort(sorted_id.begin(), sorted_id.end());
  for (uint8_t i = 0; i < sorted_id.size(); i++)
  {
    for (uint8_t j = 0; j < active_joint_id.size(); j++)
    {
      if (sorted_id.at(i) == active_joint_id.at(j))
      {
        sorted_angle_array[j] = angles.at(i);
        break;
      }
    }
  }

  for (uint8_t index = 0; index < active_joint_id.size(); index++)
    sorted_angle_vector.push_back(sorted_angle_array[index]);

  std::vector<double> calc_sorted_angle;
  std::map<Name, Component>::iterator it;

  uint8_t index = 0;
  for (it = manipulator_.getIteratorBegin(); it != manipulator_.getIteratorEnd(); it++)
  {
    if (manipulator_.getComponentJointId(it->first) != -1)
    {
      calc_sorted_angle.push_back(sorted_angle_vector.at(index++) / manipulator_.getComponentJointCoefficient(it->first));
    }
  }

  return calc_sorted_angle;
}

void OpenManipulator::actuatorInit(const void *arg)
{
  return actuator_->initActuator(arg);
}

void OpenManipulator::setActuatorControlMode()
{
  actuator_->setActuatorControlMode();
}

void OpenManipulator::actuatorEnable()
{
  return actuator_->Enable();
}

void OpenManipulator::actuatorDisable()
{
  return actuator_->Disable();
}

// DRAW
void OpenManipulator::drawInit(Name name, double move_time, const void *arg)
{
  move_time_ = move_time;
  draw_.at(name)->initDraw(arg);
}

void OpenManipulator::setRadiusForDrawing(Name name, double radius)
{
  draw_.at(name)->setRadius(radius);
}

void OpenManipulator::setStartAngularPositionForDrawing(Name name, double start_angular_position)
{
  draw_.at(name)->setAngularStartPosition(start_angular_position);
}

Pose OpenManipulator::getPoseForDrawing(Name name, double tick)
{
  return draw_.at(name)->getPose(tick);
}

// JOINT TRAJECTORY

void OpenManipulator::setMoveTime(double move_time)
{
  move_time_ = move_time;
}
void OpenManipulator::setPresentTime(double present_time)
{
  present_time_ = present_time;
}

void OpenManipulator::setControlTime(double control_time)
{
  control_time_ = control_time;
}

double OpenManipulator::getMoveTime()
{
  return move_time_;
}

double OpenManipulator::getControlTime()
{
  return control_time_;
}

void OpenManipulator::startMoving()
{
  moving_ = true;
  start_time_ = present_time_;
}

bool OpenManipulator::isMoving()
{
  return moving_;
}

void OpenManipulator::makeTrajectory(std::vector<Trajectory> start,std::vector<Trajectory> goal)
{
  if(trajectory_type_ == JOINT_TRAJECTORY)
    joint_trajectory_->init(start, goal, move_time_, control_time_);
  else if(trajectory_type_ == TASK_TRAJECTORY)
    task_trajectory_->init(start, goal, move_time_, control_time_);
}

void OpenManipulator::setStartTrajectory(Trajectory trajectory)
{
  if(trajectory_type_ == JOINT_TRAJECTORY)
    start_joint_trajectory_.push_back(trajectory);
  else if(trajectory_type_ == TASK_TRAJECTORY)
    start_task_trajectory_.push_back(trajectory);
}

void OpenManipulator::clearStartTrajectory()
{
  if(trajectory_type_ == JOINT_TRAJECTORY)
    start_joint_trajectory_.clear();
  else if(trajectory_type_ == TASK_TRAJECTORY)
    start_task_trajectory_.clear();
}

std::vector<Trajectory> OpenManipulator::getStartTrajectory()
{
  if(trajectory_type_ == JOINT_TRAJECTORY)
    return start_joint_trajectory_;
  else if(trajectory_type_ == TASK_TRAJECTORY)
    return start_task_trajectory_;
}

void OpenManipulator::setGoalTrajectory(Trajectory trajectory)
{
  if(trajectory_type_ == JOINT_TRAJECTORY)
    goal_joint_trajectory_.push_back(trajectory);
  else if(trajectory_type_ == TASK_TRAJECTORY)
    goal_task_trajectory_.push_back(trajectory);
}

void OpenManipulator::clearGoalTrajectory()
{
  if(trajectory_type_ == JOINT_TRAJECTORY)
    goal_joint_trajectory_.clear();
  else if(trajectory_type_ == TASK_TRAJECTORY)
    goal_task_trajectory_.clear();
}

std::vector<Trajectory> OpenManipulator::getGoalTrajectory()
{
  if(trajectory_type_ == JOINT_TRAJECTORY)
    return goal_joint_trajectory_;
  else if(trajectory_type_ == TASK_TRAJECTORY)
    return goal_task_trajectory_;
}

double OpenManipulator::toolMove(Name tool_name, double tool_value)
{
  double calc_value = tool_value * manipulator_.getComponentToolCoefficient(tool_name);
  manipulator_.setComponentToolValue(tool_name, calc_value);
  return sendActuatorAngle(manipulator_.getComponentToolId(tool_name), calc_value);
}

void OpenManipulator::wait(double wait_time)
{
  Trajectory start;
  Trajectory goal;

  std::vector<double> present_position = previous_goal_.position;
  std::vector<double> present_velocity = previous_goal_.velocity;
  std::vector<double> present_acceleration = previous_goal_.acceleration;

  start_joint_trajectory_.clear();
  goal_joint_trajectory_.clear();

  for (uint8_t index = 0; index < manipulator_.getDOF(); index++)
  {
    start.position = present_position.at(index);
    start.velocity = present_velocity.at(index);
    start.acceleration = present_acceleration.at(index);

    start_joint_trajectory_.push_back(start);

    goal.position = present_position.at(index);
    goal.velocity = 0.0f;
    goal.acceleration = 0.0f;

    goal_joint_trajectory_.push_back(goal);
  }

  setMoveTime(wait_time);
  makeTrajectory(start_joint_trajectory_, goal_joint_trajectory_);
  startMoving();
}


void OpenManipulator::setPreviousGoalPosition(std::vector<double> data)
{
  previous_goal_.position = data;
  return;
}

std::vector<double> OpenManipulator::getPreviousGoalPosition()
{
  return previous_goal_.position;
}
void OpenManipulator::setStartPoseForDrawing(Name name, Pose start_pose)
{
  draw_.at(name)->setStartPose(start_pose);
}
void OpenManipulator::setEndPoseForDrawing(Name name, Pose end_pose)
{
  draw_.at(name)->setEndPose(end_pose);
}


std::vector<double> OpenManipulator::controlLoop(double present_time, Name tool_name)
{
  setPresentTime(present_time);
  setAllActiveJointAngle(getPreviousGoalPosition());
  forward(getWorldChildName());

  if(moving_)
  {
    Goal joint_goal_states;

    switch(trajectory_type_)
    {
    case JOINT_TRAJECTORY:
      joint_goal_states = getJointAngleFromJointTraj();
      break;
    case TASK_TRAJECTORY:
      joint_goal_states = getJointAngleFromTaskTraj(tool_name);
      break;
    case DRAWING:
      joint_goal_states = getJointAngleFromDrawing(tool_name);
      break;
    }
    ///////////////////send target angle////////////////////////////////
    previous_goal_ = joint_goal_states;
    return sendMultipleActuatorAngle(manipulator_.getAllActiveJointID(), joint_goal_states.position);
    /////////////////////////////////////////////////////////////////////
  }

  return {};
}

Goal OpenManipulator::getJointAngleFromJointTraj()
{
  double tick_time = present_time_ - start_time_;
  Goal joint_goal_states;

  if(tick_time < move_time_)
  {
    joint_goal_states.position = joint_trajectory_->getPosition(tick_time);
    joint_goal_states.velocity = joint_trajectory_->getVelocity(tick_time);
    joint_goal_states.acceleration = joint_trajectory_->getAcceleration(tick_time);
  }
  else
  {
    joint_goal_states.position = joint_trajectory_->getPosition(move_time_);
    joint_goal_states.velocity= joint_trajectory_->getVelocity(move_time_);
    joint_goal_states.acceleration = joint_trajectory_->getAcceleration(move_time_);
    moving_   = false;
    start_time_ = present_time_;
  }
  return joint_goal_states;
}

Goal OpenManipulator::getJointAngleFromTaskTraj(Name tool_name)
{
  double tick_time = present_time_ - start_time_;
  Goal joint_goal_states;
  joint_goal_states.velocity.resize(4);
  joint_goal_states.acceleration.resize(4);

  if(tick_time < move_time_)
  {
    std::vector<double> temp = task_trajectory_->getPosition(tick_time);
    Pose goal_pose;
    goal_pose.position(0) = temp.at(0); goal_pose.position(1) = temp.at(1); goal_pose.position(2) = temp.at(2);
    goal_pose.orientation = previous_goal_.pose.orientation;
    joint_goal_states.pose = goal_pose;

    temp = task_trajectory_->getVelocity(tick_time);
    Pose goal_pose_vel;
    goal_pose_vel.position(0) = temp.at(0); goal_pose_vel.position(1) = temp.at(1); goal_pose_vel.position(2) = temp.at(2);
    joint_goal_states.pose_vel = goal_pose_vel;

    temp = task_trajectory_->getAcceleration(tick_time);
    Pose goal_pose_acc;
    goal_pose_acc.position(0) = temp.at(0); goal_pose_acc.position(1) = temp.at(1); goal_pose_acc.position(2) = temp.at(2);
    joint_goal_states.pose_acc = goal_pose_acc;

    joint_goal_states.position = kinematics_->inverse(&manipulator_, tool_name, goal_pose);
  }
  else
  {
    std::vector<double> temp = task_trajectory_->getPosition(move_time_);
    Pose goal_pose;
    goal_pose.position(0) = temp.at(0); goal_pose.position(1) = temp.at(1); goal_pose.position(2) = temp.at(2);
    goal_pose.orientation = previous_goal_.pose.orientation;
    joint_goal_states.pose = goal_pose;

    temp = task_trajectory_->getVelocity(move_time_);
    Pose goal_pose_vel;
    goal_pose_vel.position(0) = temp.at(0); goal_pose_vel.position(1) = temp.at(1); goal_pose_vel.position(2) = temp.at(2);
    joint_goal_states.pose_vel = goal_pose_vel;

    temp = task_trajectory_->getAcceleration(move_time_);
    Pose goal_pose_acc;
    goal_pose_acc.position(0) = temp.at(0); goal_pose_acc.position(1) = temp.at(1); goal_pose_acc.position(2) = temp.at(2);
    joint_goal_states.pose_acc = goal_pose_acc;

    joint_goal_states.position = kinematics_->inverse(&manipulator_, tool_name, goal_pose);
    moving_   = false;
    start_time_ = present_time_;
  }
  return joint_goal_states;

}
Goal OpenManipulator::getJointAngleFromDrawing(Name tool_name)
{
  double tick_time = present_time_ - start_time_;
  Goal joint_goal_states;
  joint_goal_states.velocity.resize(4);
  joint_goal_states.acceleration.resize(4);

  if(tick_time < move_time_)
  {
    joint_goal_states.position = kinematics_->inverse(&manipulator_, tool_name, getPoseForDrawing(object_, tick_time));
  }
  else
  {
    joint_goal_states.position = kinematics_->inverse(&manipulator_, tool_name, getPoseForDrawing(object_, move_time_));
    moving_   = false;
    start_time_ = present_time_;
  }

  return joint_goal_states;
}

void OpenManipulator::setJointTrajectory(std::vector<double> joint_angle, double move_time)
{
  trajectory_type_ = JOINT_TRAJECTORY;

  Trajectory start;
  Trajectory goal;

  start_joint_trajectory_.clear();
  goal_joint_trajectory_.clear();

  for (uint8_t index = 0; index < manipulator_.getDOF(); index++)
  {
    start.position = previous_goal_.position.at(index);
    start.velocity = previous_goal_.velocity.at(index);
    start.acceleration = previous_goal_.acceleration.at(index);
    start_joint_trajectory_.push_back(start);

    goal.position = joint_angle.at(index);
    goal.velocity = 0.0f;
    goal.acceleration = 0.0f;

    goal_joint_trajectory_.push_back(goal);
  }

  setMoveTime(move_time);
  makeTrajectory(start_joint_trajectory_, goal_joint_trajectory_);
  startMoving();
}

void OpenManipulator::setJointTrajectory(Name tool_name, Pose goal_pose, double move_time)
{
  trajectory_type_ = JOINT_TRAJECTORY;
  std::vector<double> goal_position = kinematics_->inverse(&manipulator_, tool_name, goal_pose);
  setJointTrajectory(goal_position, move_time);
}


void OpenManipulator::setTaskTrajectory(Name tool_name, Pose goal_pose, double move_time)
{
  trajectory_type_ = TASK_TRAJECTORY;
  move_time_ = move_time;

  setAllActiveJointAngle(previous_goal_.position);
  forward(manipulator_.getWorldChildName());

  previous_goal_.pose = manipulator_.getComponentPoseToWorld(tool_name);

  Vector3f goal_position_to_world = goal_pose.position;

  Trajectory start;
  Trajectory goal;

  start_task_trajectory_.clear();
  goal_task_trajectory_.clear();

  for (uint8_t index = 0; index < 3; index++)
  {
    start.position = previous_goal_.pose.position[index];
    start.velocity = 0.0;//previous_goal_.pose_vel.position[index];
    start.acceleration = 0.0;//previous_goal_.pose_acc.position[index];
    start_task_trajectory_.push_back(start);

    goal.position = goal_position_to_world[index];
    goal.velocity = 0.0f;
    goal.acceleration = 0.0f;

    goal_task_trajectory_.push_back(goal);
  }

  setMoveTime(move_time);
  makeTrajectory(start_task_trajectory_, goal_task_trajectory_);
  startMoving();

}

void OpenManipulator::setDrawing(Name tool_name, int object, double move_time, double option)
{
  trajectory_type_ = DRAWING;
  move_time_ = move_time;

  setAllActiveJointAngle(previous_goal_.position);
  forward(manipulator_.getWorldChildName());

  previous_goal_.pose = manipulator_.getComponentPoseToWorld(tool_name);

  double init_arg[2] = {move_time, ACTUATOR_CONTROL_TIME};
  void *p_init_arg = init_arg;
  drawInit(object, move_time, p_init_arg);
  setRadiusForDrawing(object, option);
  setStartPoseForDrawing(object, previous_goal_.pose);
  setStartAngularPositionForDrawing(object, 0.0);

  object_ = object;
  startMoving();
}

void OpenManipulator::setDrawing(Name tool_name, int object, double move_time, Vector3f meter)
{
  trajectory_type_ = DRAWING;
  move_time_ = move_time;

  setAllActiveJointAngle(previous_goal_.position);
  forward(manipulator_.getWorldChildName());

  previous_goal_.pose = manipulator_.getComponentPoseToWorld(tool_name);

  Vector3f present_position_to_world = previous_goal_.pose.position;
  Matrix3f present_orientation_to_world = previous_goal_.pose.orientation;

  Vector3f goal_position_to_world = present_position_to_world + meter;

  Pose start, end;
  start.position = present_position_to_world;
  start.orientation = present_orientation_to_world;

  end.position = goal_position_to_world;
  end.orientation = present_orientation_to_world;

  double init_arg[2] = {move_time, ACTUATOR_CONTROL_TIME};
  void *p_init_arg = init_arg;

  setStartPoseForDrawing(object, start);
  setEndPoseForDrawing(object, end);
  drawInit(object, move_time, p_init_arg);

  object_ = object;
  startMoving();
}
































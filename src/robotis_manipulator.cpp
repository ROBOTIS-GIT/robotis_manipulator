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

#include "robotis_manipulator/robotis_manipulator.h"

using namespace ROBOTIS_MANIPULATOR;


////////////////////////////////////////////////////////////////////////////
////////////////////////////////Basic Function//////////////////////////////
////////////////////////////////////////////////////////////////////////////

RobotisManipulator::RobotisManipulator() :
                                     moving_(false)
{

}

RobotisManipulator::~RobotisManipulator()
{
}

///////////////////////////*initialize function*/////////////////////////////

void RobotisManipulator::addWorld(Name world_name,
                           Name child_name,
                           Eigen::Vector3d world_position,
                           Eigen::Quaterniond world_orientation)
{
  manipulator_.addWorld(world_name, child_name, world_position, world_orientation);
}

void RobotisManipulator::addComponent(Name my_name,
                               Name parent_name,
                               Name child_name,
                               Eigen::Vector3d relative_position,
                               Eigen::Quaterniond relative_orientation,
                               Eigen::Vector3d axis_of_rotation,
                               int8_t joint_actuator_id,
                               double coefficient,
                               double mass,
                               Eigen::Matrix3d inertia_tensor,
                               Eigen::Vector3d center_of_mass)
{
  manipulator_.addComponent(my_name, parent_name, child_name, relative_position, relative_orientation, axis_of_rotation,
                            joint_actuator_id, coefficient, mass, inertia_tensor, center_of_mass);
}

void RobotisManipulator::addComponentChild(Name my_name, Name child_name)
{
  manipulator_.addComponentChild(my_name, child_name);
}

void RobotisManipulator::addTool(Name my_name,
                          Name parent_name,
                          Eigen::Vector3d relative_position,
                          Eigen::Quaterniond relative_orientation,
                          int8_t tool_id,
                          double coefficient,
                          double mass,
                          Eigen::Matrix3d inertia_tensor,
                          Eigen::Vector3d center_of_mass)
{

  manipulator_.addTool(my_name, parent_name, relative_position, relative_orientation, tool_id, coefficient, mass,
                       inertia_tensor, center_of_mass);
}

void RobotisManipulator::checkManipulatorSetting()
{
  //use debug
}

void RobotisManipulator::addKinematics(Kinematics *kinematics)
{
  kinematics_= kinematics;
}

void RobotisManipulator::addJointActuator(Name actuator_name, JointActuator *joint_actuator)
{
  joint_actuator_.insert(std::make_pair(actuator_name, joint_actuator));
}

void RobotisManipulator::addToolActuator(Name tool_name, ToolActuator *tool_actuator)
{
  tool_actuator_.insert(std::make_pair(tool_name, tool_actuator));
}

void RobotisManipulator::addDrawingTrajectory(Name name, DrawingTrajectory *drawing)
{
  trajectory_.drawing_.insert(std::make_pair(name, drawing));
}

/////////////////////////////////////////////////////////////////////////////

// MANIPULATOR
Manipulator RobotisManipulator::getManipulator()
{
  return manipulator_;
}



// KINEMATICS

void RobotisManipulator::updatePassiveJointValue()
{
  return kinematics_->updatePassiveJointValue(&manipulator_);
}

Eigen::MatrixXd RobotisManipulator::jacobian(Name tool_name)
{
  return kinematics_->jacobian(&manipulator_, tool_name);
}

void RobotisManipulator::forward()
{
  return kinematics_->forward(&manipulator_);
}

void RobotisManipulator::forward(Name first_component_name)
{
  return kinematics_->forward(&manipulator_, first_component_name);
}

std::vector<double> RobotisManipulator::inverse(Name tool_name, Pose goal_pose)
{
  return kinematics_->inverse(&manipulator_, tool_name, goal_pose);
}




// ACTUATOR
void RobotisManipulator::JointActuatorInit(Name actuator_name, std::vector<uint8_t> id_array, const void *arg)
{
  if(joint_actuator_.find(actuator_name) != joint_actuator_.end())
  {
    joint_actuator_.at(actuator_name)->init(id_array, arg);
  }
  else
  {
    //error
  }
}

void RobotisManipulator::toolActuatorInit(Name actuator_name, uint8_t id, const void *arg)
{
  if(tool_actuator_.find(actuator_name) != tool_actuator_.end())
  {
    tool_actuator_.at(actuator_name)->init(id, arg);
  }
  else
  {
    //error
  }
}

void RobotisManipulator::JointActuatorSetMode(Name actuator_name, std::vector<uint8_t> id_array, const void *arg)
{
  if(joint_actuator_.find(actuator_name) != joint_actuator_.end())
  {
    joint_actuator_.at(actuator_name)->setMode(id_array, arg);
  }
  else
  {
    //error
  }
}

void RobotisManipulator::toolActuatorSetMode(Name actuator_name, const void *arg)
{
  if(tool_actuator_.find(actuator_name) != tool_actuator_.end())
  {
    tool_actuator_.at(actuator_name)->setMode(arg);
  }
  else
  {
    //error
  }
}

std::vector<uint8_t> RobotisManipulator::getJointActuatorId(Name actuator_name)
{
  std::vector<uint8_t> result_vector;

  if(joint_actuator_.find(actuator_name) != joint_actuator_.end())
  {
    return result_vector = joint_actuator_.at(actuator_name)->getId();
  }
  else
  {
    //error
  }
}

uint8_t RobotisManipulator::getToolActuatorId(Name actuator_name)
{
  uint8_t result;

  if(tool_actuator_.find(actuator_name) != tool_actuator_.end())
  {
    return result = tool_actuator_.at(actuator_name)->getId();
  }
  else
  {
    //error
  }
}

void RobotisManipulator::actuatorEnable(Name actuator_name)
{
  if(joint_actuator_.find(actuator_name) != joint_actuator_.end())
  {
    joint_actuator_.at(actuator_name)->enable();
  }
  else if(tool_actuator_.find(actuator_name) != tool_actuator_.end())
  {
    tool_actuator_.at(actuator_name)->enable();
  }
  else
  {
    //error
  }
}

void RobotisManipulator::actuatorDisable(Name actuator_name)
{
  if(joint_actuator_.find(actuator_name) != joint_actuator_.end())
  {
    joint_actuator_.at(actuator_name)->disable();
  }
  else if(tool_actuator_.find(actuator_name) != tool_actuator_.end())
  {
    tool_actuator_.at(actuator_name)->disable();
  }
  else
  {
    //error
  }
}

void RobotisManipulator::allJointActuatorEnable()
{
  std::map<Name, JointActuator *>::iterator it;
  for(it = joint_actuator_.begin(); it != joint_actuator_.end(); it++)
  {
    joint_actuator_.at(it->first)->enable();
  }
}

void RobotisManipulator::allJointActuatorDisable()
{
  std::map<Name, JointActuator *>::iterator it;
  for(it = joint_actuator_.begin(); it != joint_actuator_.end(); it++)
  {
    joint_actuator_.at(it->first)->disable();
  }
}

void RobotisManipulator::allToolActuatorEnable()
{
  std::map<Name, ToolActuator *>::iterator it;
  for(it = tool_actuator_.begin(); it != tool_actuator_.end(); it++)
  {
    tool_actuator_.at(it->first)->enable();
  }
}

void RobotisManipulator::allToolActuatorDisable()
{
  std::map<Name, ToolActuator *>::iterator it;
  for(it = tool_actuator_.begin(); it != tool_actuator_.end(); it++)
  {
    tool_actuator_.at(it->first)->disable();
  }
}

void RobotisManipulator::allActuatorEnable()
{
  std::map<Name, JointActuator *>::iterator it_joint;
  std::map<Name, ToolActuator *>::iterator it_tool;
  for(it_joint = joint_actuator_.begin(); it_joint != joint_actuator_.end(); it_joint++)
  {
    joint_actuator_.at(it_joint->first)->enable();
  }
  for(it_tool = tool_actuator_.begin(); it_tool != tool_actuator_.end(); it_tool++)
  {
    tool_actuator_.at(it_tool->first)->enable();
  }
}

void RobotisManipulator::allActuatorDisable()
{
  std::map<Name, JointActuator *>::iterator it_joint;
  std::map<Name, ToolActuator *>::iterator it_tool;
  for(it_joint = joint_actuator_.begin(); it_joint != joint_actuator_.end(); it_joint++)
  {
    joint_actuator_.at(it_joint->first)->disable();
  }
  for(it_tool = tool_actuator_.begin(); it_tool != tool_actuator_.end(); it_tool++)
  {
    tool_actuator_.at(it_tool->first)->disable();
  }
}

bool RobotisManipulator::sendJointActuatorValue(Name actuator_name, uint8_t actuator_id, Actuator value)
{
  return joint_actuator_.at(actuator_name)->sendJointActuatorValue(actuator_id, value);
}

bool RobotisManipulator::sendMultipleJointActuatorValue(Name actuator_name, std::vector<uint8_t> actuator_id, std::vector<Actuator> value_vector)
{
  return joint_actuator_.at(actuator_name)->sendMultipleJointActuatorValue(actuator_id, value_vector);
}

Actuator RobotisManipulator::receiveJointActuatorValue(Name actuator_name, uint8_t actuator_id)
{
  return joint_actuator_.at(actuator_name)->receiveJointActuatorValue(actuator_id);
}

std::vector<Actuator> RobotisManipulator::receiveMultipleJointActuatorValue(Name actuator_name, std::vector<uint8_t> actuator_id)
{
    return joint_actuator_.at(actuator_name)->receiveMultipleJointActuatorValue(actuator_id);
}

bool RobotisManipulator::sendToolActuatorValue(uint8_t actuator_id, double value)
{
  std::map<Name, ToolActuator *>::iterator it_tool;
  for(it_tool = tool_actuator_.begin(); it_tool != tool_actuator_.end(); it_tool++)
  {
    if(tool_actuator_.at(it_tool->first)->getId() == actuator_id)
    {
      return tool_actuator_.at(it_tool->first)->sendToolActuatorValue(actuator_id, value);
    }
  }
}

double RobotisManipulator::receiveToolActuatorValue(Name actuator_name, uint8_t actuator_id)
{
  return tool_actuator_.at(actuator_name)->receiveToolActuatorValue(actuator_id);
}

bool RobotisManipulator::sendAllJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<Actuator> value_vector)
{
  std::map<Name, JointActuator *>::iterator it_joint;
  std::vector<uint8_t> single_joint_id;
  std::vector<Actuator> single_joint_value_vector;
  for(it_joint = joint_actuator_.begin(); it_joint != joint_actuator_.end(); it_joint++)
  {
    single_joint_id = joint_actuator_.at(it_joint->first)->getId();
    for(int index; index < single_joint_id.size(); index++)
    {
      for(int index2; index2 < actuator_id.size(); index2++)
      {
        if(single_joint_id.at(index) == actuator_id.at(index2))
        {
          single_joint_value_vector.push_back(value_vector.at(index2));
        }
      }
    }
    joint_actuator_.at(it_joint->first)->sendMultipleJointActuatorValue(single_joint_id, single_joint_value_vector);
    single_joint_id.clear();
    single_joint_value_vector.clear();
  }
}

std::vector<Actuator> RobotisManipulator::receiveAllJointActuatorValue(std::vector<uint8_t> actuator_id)
{
  std::map<Name, JointActuator *>::iterator it_joint;
  std::vector<uint8_t> copy_joint_id;
  std::vector<Actuator> copy_joint_value_vector;

  std::vector<uint8_t> single_joint_id;
  std::vector<Actuator> single_joint_value_vector;
  for(it_joint = joint_actuator_.begin(); it_joint != joint_actuator_.end(); it_joint++)
  {
    single_joint_id = joint_actuator_.at(it_joint->first)->getId();
    single_joint_value_vector = joint_actuator_.at(it_joint->first)->receiveMultipleJointActuatorValue(single_joint_id);
    for(int index; index < single_joint_id.size(); index++)
    {
      copy_joint_id.push_back(single_joint_id.at(index));
      copy_joint_value_vector.push_back(single_joint_value_vector.at(index));
    }
    single_joint_id.clear();
    single_joint_value_vector.clear();
  }

  std::vector<Actuator> result_vector;
  for(int index; index < actuator_id.size(); index++)
  {
    for(int index2; index2 < copy_joint_id.size(); index2++)
    {
      if(actuator_id.at(index) == copy_joint_id.at(index2))
        result_vector.push_back(copy_joint_value_vector.at(index2));
    }
  }
  return result_vector;
}


bool RobotisManipulator::sendAllToolActuatorValue(std::vector<uint8_t> actuator_id, std::vector<double> value_vector)
{
  std::map<Name, ToolActuator *>::iterator it_tool;
  uint8_t single_tool_id;
  double single_tool_value;
  for(it_tool = tool_actuator_.begin(); it_tool != tool_actuator_.end(); it_tool++)
  {
    single_tool_id = tool_actuator_.at(it_tool->first)->getId();
    for(int index; index < actuator_id.size(); index++)
    {
      if(single_tool_id == actuator_id.at(index))
      {
        tool_actuator_.at(it_tool->first)->sendToolActuatorValue(actuator_id.at(index), value_vector.at(index));
      }
    }
  }
}

std::vector<double> RobotisManipulator::receiveAllToolActuatorValue(std::vector<uint8_t> actuator_id)
{
  std::map<Name, ToolActuator *>::iterator it_tool;
  std::vector<uint8_t> copy_tool_id;
  std::vector<double> copy_tool_value_vector;

  for(it_tool = tool_actuator_.begin(); it_tool != tool_actuator_.end(); it_tool++)
  {
    copy_tool_id.push_back(tool_actuator_.at(it_tool->first)->getId());
    copy_tool_value_vector.push_back(tool_actuator_.at(it_tool->first)->receiveToolActuatorValue(tool_actuator_.at(it_tool->first)->getId()));
  }

  std::vector<double> result_vector;
  for(int index; index < actuator_id.size(); index++)
  {
    for(int index2; index2 < copy_tool_id.size(); index2++)
    {
      if(actuator_id.at(index) == copy_tool_id.at(index2))
        result_vector.push_back(copy_tool_value_vector.at(index2));
    }
  }
  return result_vector;
}


////////
// TIME

void RobotisManipulator::setMoveTime(double move_time)
{
  manipulation_time_.move_time = move_time;
}
void RobotisManipulator::setPresentTime(double present_time)
{
  manipulation_time_.present_time = present_time;
}

void RobotisManipulator::setControlTime(double control_time)
{
  manipulation_time_.control_time = control_time;
}

double RobotisManipulator::getMoveTime()
{
  return manipulation_time_.move_time;
}

double RobotisManipulator::getControlTime()
{
  return manipulation_time_.control_time;
}

void RobotisManipulator::startMoving()
{
  moving_ = true;
  manipulation_time_.start_time = manipulation_time_.present_time;
}

bool RobotisManipulator::isMoving()
{
  return moving_;
}


// Way Point

void RobotisManipulator::initWayPoint(std::vector<double> joint_value_vector)
{
  trajectory_.manipulator_ = manipulator_;
  std::vector<WayPoint> joint_way_point_vector;
  WayPoint joint_way_point;
  for(int index; index < joint_value_vector.size(); index++)
  {
    joint_way_point.value = joint_value_vector.at(index);
    joint_way_point.velocity = 0.0;
    joint_way_point.acceleration = 0.0;
    joint_way_point_vector.push_back(joint_way_point);
  }

  setPresentJointWayPoint(joint_way_point_vector);
  UpdatePresentWayPoint();

  trajectory_.start_way_point_.reserve(manipulator_.getDOF());
  trajectory_.goal_way_point_.reserve(manipulator_.getDOF());
}


void RobotisManipulator::UpdatePresentWayPoint()
{
  //kinematics (position)
  kinematics_->updatePassiveJointValue(&trajectory_.manipulator_);
  kinematics_->forward(&trajectory_.manipulator_);

  //dynamics (velocity)
  std::map<Name, Component>::iterator it;
  Eigen::VectorXd joint_velocity(trajectory_.manipulator_.getDOF());
  Eigen::VectorXd pose_velocity(6);
  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d angular_velocity;

  int8_t index = 0;
  for (it = trajectory_.manipulator_.getIteratorBegin(); it != trajectory_.manipulator_.getIteratorEnd(); it++)
  {
    if (trajectory_.manipulator_.getJointId(it->first) != -1) // Check whether Active or Passive
    {
      // Active
      joint_velocity[index] = trajectory_.manipulator_.getJointVelocity(it->first);
      index++;
    }
  }
  for (it = trajectory_.manipulator_.getIteratorBegin(); it != trajectory_.manipulator_.getIteratorEnd(); it++)
  {
    if (trajectory_.manipulator_.getToolId(it->first) != -1)
    {
      for(int index2 = 0; index2 < trajectory_.manipulator_.getDOF(); index2++)
      {
        joint_velocity[index2] =trajectory_.manipulator_.getJointVelocity(it->first);
      }
      pose_velocity = kinematics_->jacobian(&trajectory_.manipulator_, it->first)*joint_velocity;
      linear_velocity[0] = pose_velocity[0];
      linear_velocity[1] = pose_velocity[1];
      linear_velocity[2] = pose_velocity[2];
      angular_velocity[0] = pose_velocity[3];
      angular_velocity[1] = pose_velocity[4];
      angular_velocity[2] = pose_velocity[5];
      Dynamicpose dynamic_pose;
      dynamic_pose.linear.velocity = linear_velocity;
      dynamic_pose.angular.velocity = angular_velocity;
      dynamic_pose.linear.accelation = Eigen::Vector3d::Zero();
      dynamic_pose.angular.accelation = Eigen::Vector3d::Zero();

      trajectory_.manipulator_.setComponentDynamicPoseToWorld(it->first, dynamic_pose);
    }
  }
}

void RobotisManipulator::setPresentJointWayPoint(std::vector<WayPoint> joint_value_vector)
{
  std::map<Name, Component>::iterator it;
  int8_t index = 0;

  for (it = trajectory_.manipulator_.getIteratorBegin(); it != trajectory_.manipulator_.getIteratorEnd(); it++)
  {
    if (trajectory_.manipulator_.getJointId(it->first) != -1)
    {
      trajectory_.manipulator_.setJointValue(it->first, joint_value_vector.at(index).value);
      trajectory_.manipulator_.setJointVelocity(it->first, joint_value_vector.at(index).velocity);
      trajectory_.manipulator_.setJointAcceleration(it->first, joint_value_vector.at(index).acceleration);
    }
    index++;
  }
}

void RobotisManipulator::setPresentTaskWayPoint(Name tool_name, std::vector<WayPoint> tool_value_vector)
{
  Pose pose_to_world;
  Dynamicpose dynamic_pose;
  for(int pos_count = 0; pos_count < 3; pos_count++)
  {
    pose_to_world.position[pos_count] = tool_value_vector.at(pos_count).value;
    dynamic_pose.linear.velocity[pos_count] = tool_value_vector.at(pos_count).velocity;
    dynamic_pose.linear.accelation[pos_count] = tool_value_vector.at(pos_count).acceleration;
  }

  Eigen::Vector3d orientation_value_vector;
  Eigen::Vector3d orientation_velocity_vector;
  Eigen::Vector3d orientation_acceleration_vector;
  for(int ori_count = 0; ori_count < 3; ori_count++)
  {
    orientation_value_vector[ori_count] = tool_value_vector.at(ori_count+3).value;
    orientation_velocity_vector[ori_count] = tool_value_vector.at(ori_count+3).velocity;
    orientation_acceleration_vector[ori_count] = tool_value_vector.at(ori_count+3).acceleration;
  }
  Eigen::Quaterniond orientation_Quat;
  orientation_Quat = RM_MATH::convertRPYToQuaternion(orientation_value_vector[0], orientation_value_vector[1], orientation_value_vector[2]);
  pose_to_world.orientation = orientation_Quat;
  dynamic_pose.angular.velocity = orientation_velocity_vector;
  dynamic_pose.angular.accelation = orientation_acceleration_vector;

  trajectory_.manipulator_.setComponentPoseToWorld(tool_name, pose_to_world);
  trajectory_.manipulator_.setComponentDynamicPoseToWorld(tool_name, dynamic_pose);
}


std::vector<WayPoint> RobotisManipulator::getPresentJointWayPoint()
{
  std::map<Name, Component>::iterator it;
  WayPoint result;
  std::vector<WayPoint> result_vector;

  for (it = trajectory_.manipulator_.getIteratorBegin(); it != trajectory_.manipulator_.getIteratorEnd(); it++)
  {
    if (trajectory_.manipulator_.getJointId(it->first) != -1) // Check whether Active or Passive
    {
      // Active
      result.value = trajectory_.manipulator_.getJointValue(it->first);
      result.velocity = trajectory_.manipulator_.getJointVelocity(it->first);
      result.acceleration = trajectory_.manipulator_.getJointAcceleration(it->first);

      result_vector.push_back(result);
    }
  }
  return result_vector;
}

std::vector<WayPoint> RobotisManipulator::getPresentTaskWayPoint(Name tool_name)
{
  std::vector<WayPoint> result_vector;
  WayPoint result;
  for(int pos_count = 0; pos_count < 3; pos_count++)
  {
    result.value = trajectory_.manipulator_.getComponentPoseToWorld(tool_name).position[pos_count];
    result.velocity = trajectory_.manipulator_.getComponentDynamicPoseToWorld(tool_name).linear.velocity[pos_count];
    result.acceleration = trajectory_.manipulator_.getComponentDynamicPoseToWorld(tool_name).linear.accelation[pos_count];

    result_vector.push_back(result);
  }

  Eigen::Vector3d orientation_vector =  RM_MATH::convertQuaternionToRPY(trajectory_.manipulator_.getComponentPoseToWorld(tool_name).orientation);
  for(int ori_count = 0; ori_count < 3; ori_count++)
  {
    result.value = orientation_vector[ori_count];
    result.velocity = trajectory_.manipulator_.getComponentDynamicPoseToWorld(tool_name).angular.velocity[ori_count];
    result.acceleration = trajectory_.manipulator_.getComponentDynamicPoseToWorld(tool_name).angular.accelation[ori_count];

    result_vector.push_back(result);
  }

  return result_vector;
}

void RobotisManipulator::setStartWayPoint(std::vector<WayPoint> start_way_point)
{
  trajectory_.start_way_point_ = start_way_point;
}

void RobotisManipulator::setGoalWayPoint(std::vector<WayPoint> goal_way_point)
{
  trajectory_.goal_way_point_ = goal_way_point;
}

void RobotisManipulator::clearStartWayPoint()
{
  trajectory_.start_way_point_.clear();
}

void RobotisManipulator::clearGoalWayPoint()
{
  trajectory_.goal_way_point_.clear();
}

std::vector<WayPoint> RobotisManipulator::getStartWayPoint()
{
  return trajectory_.start_way_point_;
}

std::vector<WayPoint> RobotisManipulator::getGoalWayPoint()
{
  return trajectory_.goal_way_point_;
}


//Trajectory

void RobotisManipulator::makeJointTrajectory()
{
  trajectory_.joint_.init(manipulation_time_.move_time, manipulation_time_.control_time, trajectory_.start_way_point_, trajectory_.goal_way_point_);
}

void RobotisManipulator::makeTaskTrajectory()
{
  trajectory_.task_.init(manipulation_time_.move_time, manipulation_time_.control_time, trajectory_.start_way_point_, trajectory_.goal_way_point_);
}

void RobotisManipulator::makeDrawingTrajectory(Name drawing_name, const void *arg)
{
  trajectory_.drawing_.at(drawing_name)->init(manipulation_time_.move_time, manipulation_time_.control_time, trajectory_.start_way_point_, arg);
}



//Trajectory Control Fuction


void RobotisManipulator::jointTrajectoryMove(std::vector<double> goal_joint_angle, double move_time)
{
  trajectory_.trajectory_type_ = JOINT_TRAJECTORY;

  clearStartWayPoint();
  clearGoalWayPoint();

  setMoveTime(move_time);

  setStartWayPoint(getPresentJointWayPoint());

  WayPoint goal_way_point;
  std::vector<WayPoint> goal_way_point_vector;
  for (uint8_t index = 0; index < trajectory_.manipulator_.getDOF(); index++)
  {
    goal_way_point.value = goal_joint_angle.at(index);
    goal_way_point.velocity = 0.0;
    goal_way_point.acceleration = 0.0;

    goal_way_point_vector.push_back(goal_way_point);
  }
  setGoalWayPoint(goal_way_point_vector);

  makeJointTrajectory();
  startMoving();
}

void RobotisManipulator::jointTrajectoryMove(Name tool_name, Pose goal_pose, double move_time)
{
  trajectory_.trajectory_type_ = JOINT_TRAJECTORY;

  clearStartWayPoint();
  clearGoalWayPoint();

  setMoveTime(move_time);

  setStartWayPoint(getPresentJointWayPoint());

  std::vector<double> goal_joint_angle;
  goal_joint_angle = kinematics_->inverse(&trajectory_.manipulator_, tool_name, goal_pose);

  WayPoint goal_way_point;
  std::vector<WayPoint> goal_way_point_vector;
  for (uint8_t index = 0; index < trajectory_.manipulator_.getDOF(); index++)
  {
    goal_way_point.value = goal_joint_angle.at(index);
    goal_way_point.velocity = 0.0;
    goal_way_point.acceleration = 0.0;

    goal_way_point_vector.push_back(goal_way_point);
  }
  setGoalWayPoint(goal_way_point_vector);

  makeJointTrajectory();
  startMoving();
}

void RobotisManipulator::taskTrajectoryMove(Name tool_name, Pose goal_pose, double move_time)
{
  trajectory_.trajectory_type_ = TASK_TRAJECTORY;
  trajectory_.present_controled_tool_name_ = tool_name;

  clearStartWayPoint();
  clearGoalWayPoint();

  setMoveTime(move_time);

  setStartWayPoint(getPresentTaskWayPoint(tool_name));

  Eigen::Vector3d goal_position_to_world = goal_pose.position;
  Eigen::Vector3d goal_orientation_to_world = RM_MATH::convertQuaternionToRPY(goal_pose.orientation);

  WayPoint goal_way_point;
  std::vector<WayPoint> goal_way_point_vector;

  for (uint8_t index = 0; index < 3; index++)
  {
    goal_way_point.value = goal_position_to_world[index];
    goal_way_point.velocity = 0.0;
    goal_way_point.acceleration =0.0;
    goal_way_point_vector.push_back(goal_way_point);
  }
  for (uint8_t index = 0; index < 3; index++)
  {
    goal_way_point.value = goal_orientation_to_world[index];
    goal_way_point.velocity = 0.0;
    goal_way_point.acceleration =0.0;
    goal_way_point_vector.push_back(goal_way_point);
  }
  setGoalWayPoint(goal_way_point_vector);

  makeTaskTrajectory();
  startMoving();
}

void RobotisManipulator::drawingTrajectoryMove(Name drawing_name, Name tool_name, double *arg, double move_time)
{
  trajectory_.trajectory_type_ = DRAWING_TRAJECTORY;
  trajectory_.present_controled_tool_name_ = tool_name;

  trajectory_.drawing_.at(drawing_name)->output_way_point_type_ = TASK;
  trajectory_.present_drawing_object_name_ = drawing_name;

  clearStartWayPoint();
  clearGoalWayPoint();

  setMoveTime(move_time);

  setStartWayPoint(getPresentTaskWayPoint(tool_name));

  makeDrawingTrajectory(drawing_name, arg);

  startMoving();
}

void RobotisManipulator::drawingTrajectoryMove(Name drawing_name, double *arg, double move_time)
{
  trajectory_.trajectory_type_ = DRAWING_TRAJECTORY;

  trajectory_.drawing_.at(drawing_name)->output_way_point_type_ = JOINT;
  trajectory_.present_drawing_object_name_ = drawing_name;

  clearStartWayPoint();
  clearGoalWayPoint();

  setMoveTime(move_time);

  setStartWayPoint(getPresentJointWayPoint());

  makeDrawingTrajectory(drawing_name, arg);

  startMoving();
}

void RobotisManipulator::TrajectoryWait(double wait_time)
{
  trajectory_.trajectory_type_ = JOINT_TRAJECTORY;

  clearStartWayPoint();
  clearGoalWayPoint();

  setMoveTime(wait_time);

  setStartWayPoint(getPresentJointWayPoint());

  std::vector<WayPoint> goal_way_point_vector;
  goal_way_point_vector = getPresentJointWayPoint();

  for (uint8_t index = 0; index < goal_way_point_vector.size(); index++)
  {
    goal_way_point_vector.at(index).velocity = 0.0;
    goal_way_point_vector.at(index).acceleration = 0.0;
  }
  setGoalWayPoint(goal_way_point_vector);

  makeJointTrajectory();
  startMoving();
}



std::vector<Actuator> RobotisManipulator::getTrajectoryJointValue(double tick_time)
{
  std::vector<Actuator> result_joint_actuator_value;
  std::vector<Actuator> result;

  if(trajectory_.trajectory_type_ == JOINT_TRAJECTORY)
  {
    std::vector<WayPoint> joint_way_point_value;
    joint_way_point_value = trajectory_.joint_.getJointWayPoint(tick_time);
    setPresentJointWayPoint(joint_way_point_value);
    UpdatePresentWayPoint();
    for(int index = 0; index < joint_way_point_value.size(); index++)
    {
      result_joint_actuator_value.at(index).value = joint_way_point_value.at(index).value;
      result_joint_actuator_value.at(index).velocity = joint_way_point_value.at(index).velocity;
      result_joint_actuator_value.at(index).acceleration = joint_way_point_value.at(index).acceleration;
    }
  }
  else if(trajectory_.trajectory_type_ == TASK_TRAJECTORY)
  {
    std::vector<WayPoint> task_way_point_value;
    Pose goal_pose;
    std::vector<double> joint_value;
    task_way_point_value = trajectory_.task_.getTaskWayPoint(tick_time);
    setPresentTaskWayPoint(trajectory_.present_controled_tool_name_, task_way_point_value);

    goal_pose.position[0] = task_way_point_value.at(0).value;
    goal_pose.position[1] = task_way_point_value.at(1).value;
    goal_pose.position[2] = task_way_point_value.at(2).value;
    goal_pose.orientation = RM_MATH::convertRPYToQuaternion(task_way_point_value.at(3).value, task_way_point_value.at(4).value, task_way_point_value.at(5).value);
    joint_value = kinematics_->inverse(&trajectory_.manipulator_, trajectory_.present_controled_tool_name_, goal_pose);

    for(int index = 0; index < joint_value.size(); index++)
    {
      result_joint_actuator_value.at(index).value = joint_value.at(index);
      result_joint_actuator_value.at(index).velocity = 0.0;
      result_joint_actuator_value.at(index).acceleration = 0.0;
    }
  }
  else if(trajectory_.trajectory_type_ == DRAWING_TRAJECTORY)
  {
    if(trajectory_.drawing_.at(trajectory_.present_drawing_object_name_)->output_way_point_type_==TASK)
    {
      std::vector<WayPoint> task_way_point_value;
      Pose goal_pose;
      std::vector<double> joint_value;
      task_way_point_value = trajectory_.drawing_.at(trajectory_.present_drawing_object_name_)->getTaskWayPoint(tick_time);
      setPresentTaskWayPoint(trajectory_.present_controled_tool_name_, task_way_point_value);

      goal_pose.position[0] = task_way_point_value.at(0).value;
      goal_pose.position[1] = task_way_point_value.at(1).value;
      goal_pose.position[2] = task_way_point_value.at(2).value;
      goal_pose.orientation = RM_MATH::convertRPYToQuaternion(task_way_point_value.at(3).value, task_way_point_value.at(4).value, task_way_point_value.at(5).value);
      joint_value = kinematics_->inverse(&trajectory_.manipulator_, trajectory_.present_controled_tool_name_, goal_pose);

      for(int index = 0; index < joint_value.size(); index++)
      {
        result_joint_actuator_value.at(index).value = joint_value.at(index);
        result_joint_actuator_value.at(index).velocity = 0.0;
        result_joint_actuator_value.at(index).acceleration = 0.0;
      }
    }
    else if(trajectory_.drawing_.at(trajectory_.present_drawing_object_name_)->output_way_point_type_==JOINT)
    {
      std::vector<WayPoint> joint_way_point_value;
      joint_way_point_value = trajectory_.drawing_.at(trajectory_.present_drawing_object_name_)->getJointWayPoint(tick_time);
      setPresentJointWayPoint(joint_way_point_value);
      UpdatePresentWayPoint();
      for(int index = 0; index < joint_way_point_value.size(); index++)
      {
        result_joint_actuator_value.at(index).value = joint_way_point_value.at(index).value;
        result_joint_actuator_value.at(index).velocity = joint_way_point_value.at(index).velocity;
        result_joint_actuator_value.at(index).acceleration = joint_way_point_value.at(index).acceleration;
      }
    }
  }

  std::map<Name, Component>::iterator it;
  int index_it;
  for (it = trajectory_.manipulator_.getIteratorBegin(); it != trajectory_.manipulator_.getIteratorEnd(); it++)
  {
    if (trajectory_.manipulator_.getJointId(it->first) != -1) // Check whether Active or Passive
    {
      // Active
      result.at(index_it).value = result_joint_actuator_value.at(index_it).value / trajectory_.manipulator_.getJointCoefficient(it->first);
      result.at(index_it).velocity = result_joint_actuator_value.at(index_it).velocity / trajectory_.manipulator_.getJointCoefficient(it->first);
      result.at(index_it).acceleration = result_joint_actuator_value.at(index_it).acceleration / trajectory_.manipulator_.getJointCoefficient(it->first);
    }
    index_it++;
  }

  return result;
}


std::vector<Actuator> RobotisManipulator::TrajectoryTimeCounter()
{
  double tick_time = manipulation_time_.present_time - manipulation_time_.start_time;

  if(tick_time < manipulation_time_.move_time)
  {
    moving_ = true;
    return getTrajectoryJointValue(tick_time);
  }
  else
  {
    moving_   = false;
    return getTrajectoryJointValue(manipulation_time_.move_time);
  }
}


void RobotisManipulator::trajectoryControllerLoop(double present_time)
{
  setPresentTime(present_time);

  if(moving_)
  {
    std::vector<Actuator> joint_goal_way_point;
    joint_goal_way_point = TrajectoryTimeCounter();

    ///////////////////send target angle////////////////////////////////
    sendAllJointActuatorValue(manipulator_.getAllActiveJointID(), joint_goal_way_point);
    /////////////////////////////////////////////////////////////////////
  }

}

double RobotisManipulator::toolMove(Name tool_name, double tool_value)
{
  manipulator_.setToolValue(tool_name, tool_value);
  sendToolActuatorValue(manipulator_.getToolId(tool_name), manipulator_.getToolActuatorValue(tool_name));
}








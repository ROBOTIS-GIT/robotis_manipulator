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
                                     moving_(false),
                                     using_platform_(false),
                                     step_moving_(false)
{

}

RobotisManipulator::~RobotisManipulator()
{
}

///////////////////////////*initialize function*/////////////////////////////

void RobotisManipulator::addWorld(Name world_name,
                           Name child_name,
                           Eigen::Vector3d world_position,
                           Eigen::Matrix3d world_orientation)
{
  manipulator_.addWorld(world_name, child_name, world_position, world_orientation);
}

void RobotisManipulator::addComponent(Name my_name,
                               Name parent_name,
                               Name child_name,
                               Eigen::Vector3d relative_position,
                               Eigen::Matrix3d relative_orientation,
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
                          Eigen::Matrix3d relative_orientation,
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

void RobotisManipulator::addJointActuator(Name actuator_name, JointActuator *joint_actuator, std::vector<uint8_t> id_array, const void *arg)
{
  joint_actuator_.insert(std::make_pair(actuator_name, joint_actuator));
  if(joint_actuator_.find(actuator_name) != joint_actuator_.end())
  {
    joint_actuator_.at(actuator_name)->init(id_array, arg);
  }
  else
  {
    //error
  }
  for(int index = 0; index < id_array.size(); index++)
  {
    manipulator_.setComponentActuatorName(manipulator_.findJointComponentNameFromId(id_array.at(index)),actuator_name);
  }
  using_platform_ = true;
}

void RobotisManipulator::addToolActuator(Name actuator_name, ToolActuator *tool_actuator, uint8_t id, const void *arg)
{
  tool_actuator_.insert(std::make_pair(actuator_name, tool_actuator));
  if(tool_actuator_.find(actuator_name) != tool_actuator_.end())
  {
    tool_actuator_.at(actuator_name)->init(id, arg);
  }
  else
  {
    //error
  }
  manipulator_.setComponentActuatorName(manipulator_.findToolComponentNameFromId(id),actuator_name);
  using_platform_ = true;
}

void RobotisManipulator::addDrawingTrajectory(Name name, DrawingTrajectory *drawing)
{
  trajectory_.drawing_.insert(std::make_pair(name, drawing));
}

/////////////////////////////////////////////////////////////////////////////

// MANIPULATOR
Manipulator *RobotisManipulator::getManipulator()
{
  return &manipulator_;
}

void RobotisManipulator::setAllActiveJointValue(std::vector<WayPoint> joint_value_vector)
{
  manipulator_.setAllActiveJointValue(joint_value_vector);
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

void RobotisManipulator::jointActuatorSetMode(Name actuator_name, std::vector<uint8_t> id_array, const void *arg)
{
  if(using_platform_)
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
}

void RobotisManipulator::toolActuatorSetMode(Name actuator_name, const void *arg)
{
  if(using_platform_)
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
}

std::vector<uint8_t> RobotisManipulator::getJointActuatorId(Name actuator_name)
{
  std::vector<uint8_t> result_vector;

  if(using_platform_)
  {
    if(joint_actuator_.find(actuator_name) != joint_actuator_.end())
    {
      return result_vector = joint_actuator_.at(actuator_name)->getId();
    }
    else
    {
      //error
    }
  }
}

uint8_t RobotisManipulator::getToolActuatorId(Name actuator_name)
{
  uint8_t result;

  if(using_platform_)
  {
    if(tool_actuator_.find(actuator_name) != tool_actuator_.end())
    {
      return result = tool_actuator_.at(actuator_name)->getId();
    }
    else
    {
      //error
    }
  }
}

void RobotisManipulator::actuatorEnable(Name actuator_name)
{
  if(using_platform_)
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
}

void RobotisManipulator::actuatorDisable(Name actuator_name)
{
  if(using_platform_)
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
}

void RobotisManipulator::allJointActuatorEnable()
{
  if(using_platform_)
  {
    for(it_joint_actuator_ = joint_actuator_.begin(); it_joint_actuator_ != joint_actuator_.end(); it_joint_actuator_++)
    {
      joint_actuator_.at(it_joint_actuator_->first)->enable();
    }
  }
}

void RobotisManipulator::allJointActuatorDisable()
{
  if(using_platform_)
  {
    for(it_joint_actuator_ = joint_actuator_.begin(); it_joint_actuator_ != joint_actuator_.end(); it_joint_actuator_++)
    {
      joint_actuator_.at(it_joint_actuator_->first)->disable();
    }
  }
}

void RobotisManipulator::allToolActuatorEnable()
{
  if(using_platform_)
  {
    for(it_tool_actuator_ = tool_actuator_.begin(); it_tool_actuator_ != tool_actuator_.end(); it_tool_actuator_++)
    {
      tool_actuator_.at(it_tool_actuator_->first)->enable();
    }
  }
}

void RobotisManipulator::allToolActuatorDisable()
{
  if(using_platform_)
  {
    for(it_tool_actuator_ = tool_actuator_.begin(); it_tool_actuator_ != tool_actuator_.end(); it_tool_actuator_++)
    {
      tool_actuator_.at(it_tool_actuator_->first)->disable();
    }
  }
}

void RobotisManipulator::allActuatorEnable()
{
  if(using_platform_)
  {
    for(it_joint_actuator_ = joint_actuator_.begin(); it_joint_actuator_ != joint_actuator_.end(); it_joint_actuator_++)
    {
      joint_actuator_.at(it_joint_actuator_->first)->enable();
    }
    for(it_tool_actuator_ = tool_actuator_.begin(); it_tool_actuator_ != tool_actuator_.end(); it_tool_actuator_++)
    {
      tool_actuator_.at(it_tool_actuator_->first)->enable();
    }
  }
}

void RobotisManipulator::allActuatorDisable()
{
  if(using_platform_)
  {
    for(it_joint_actuator_ = joint_actuator_.begin(); it_joint_actuator_ != joint_actuator_.end(); it_joint_actuator_++)
    {
      joint_actuator_.at(it_joint_actuator_->first)->disable();
    }
    for(it_tool_actuator_ = tool_actuator_.begin(); it_tool_actuator_ != tool_actuator_.end(); it_tool_actuator_++)
    {
      tool_actuator_.at(it_tool_actuator_->first)->disable();
    }
  }
}

////send

bool RobotisManipulator::sendJointActuatorValue(Name joint_component_name, WayPoint value)
{
  if(using_platform_)
  {
    double coefficient;
    coefficient = manipulator_.getJointCoefficient(joint_component_name);
    value.value = value.value / coefficient;
    value.velocity = value.velocity / coefficient;
    value.effort = value.effort / coefficient;

    std::vector<uint8_t> id;
    std::vector<WayPoint> value_vector;
    id.push_back(manipulator_.getJointId(joint_component_name));
    value_vector.push_back(value);
    return joint_actuator_.at(manipulator_.getComponentActuatorName(joint_component_name))->sendJointActuatorValue(id, value_vector);
  }
}

bool RobotisManipulator::sendMultipleJointActuatorValue(std::vector<Name> joint_component_name, std::vector<WayPoint> value_vector)
{
  if(using_platform_)
  {
    if(joint_component_name.size() != value_vector.size())
      return false; //error;

    std::vector<int8_t> joint_id;
    for(int index = 0; index < value_vector.size(); index++)
    {
      value_vector.at(index).value = value_vector.at(index).value / manipulator_.getJointCoefficient(joint_component_name.at(index));
      value_vector.at(index).velocity = value_vector.at(index).velocity / manipulator_.getJointCoefficient(joint_component_name.at(index));
      value_vector.at(index).effort = value_vector.at(index).effort / manipulator_.getJointCoefficient(joint_component_name.at(index));
      joint_id.push_back(manipulator_.getJointId(joint_component_name.at(index)));
    }

    std::vector<uint8_t> single_actuator_id;
    std::vector<WayPoint> single_value_vector;
    for(it_joint_actuator_ = joint_actuator_.begin(); it_joint_actuator_ != joint_actuator_.end(); it_joint_actuator_++)
    {
      single_actuator_id = joint_actuator_.at(it_joint_actuator_->first)->getId();
      for(int index = 0; index < single_actuator_id.size(); index++)
      {
        for(int index2=0; index2 < joint_id.size(); index2++)
        {
           if(single_actuator_id.at(index) == joint_id.at(index2))
           {
             single_value_vector.push_back(value_vector.at(index2));
           }
        }
      }
      joint_actuator_.at(it_joint_actuator_->first)->sendJointActuatorValue(single_actuator_id, single_value_vector);
    }
    return true;
  }
}

bool RobotisManipulator::sendAllJointActuatorValue(std::vector<WayPoint> value_vector)
{
  if(using_platform_)
  {
    std::map<Name, Component>::iterator it;
    std::vector<int8_t> joint_id;
    int index = 0;
    for (it = manipulator_.getIteratorBegin(); it != manipulator_.getIteratorEnd(); it++)
    {
      if(manipulator_.getJointId(it->first) != -1)
      {
        value_vector.at(index).value = value_vector.at(index).value / manipulator_.getJointCoefficient(it->first);
        value_vector.at(index).velocity = value_vector.at(index).velocity / manipulator_.getJointCoefficient(it->first);
        value_vector.at(index).effort = value_vector.at(index).effort / manipulator_.getJointCoefficient(it->first);
        joint_id.push_back(manipulator_.getJointId(it->first));
        index++;
      }
    }

    std::vector<uint8_t> single_actuator_id;
    std::vector<WayPoint> single_value_vector;
    for(it_joint_actuator_ = joint_actuator_.begin(); it_joint_actuator_ != joint_actuator_.end(); it_joint_actuator_++)
    {
      single_actuator_id = joint_actuator_.at(it_joint_actuator_->first)->getId();
      for(int index = 0; index < single_actuator_id.size(); index++)
      {
        for(int index2=0; index2 < joint_id.size(); index2++)
        {
           if(single_actuator_id.at(index) == joint_id.at(index2))
           {
             single_value_vector.push_back(value_vector.at(index2));
           }
        }
      }
      joint_actuator_.at(it_joint_actuator_->first)->sendJointActuatorValue(single_actuator_id, single_value_vector);
    }
    return true;
  }
}


WayPoint RobotisManipulator::receiveJointActuatorValue(Name joint_component_name)
{
  if(using_platform_)
  {
    std::vector<uint8_t> actuator_id;
    std::vector<WayPoint> result;

    actuator_id.push_back(manipulator_.getJointId(joint_component_name));

    result = joint_actuator_.at(manipulator_.getComponentActuatorName(joint_component_name))->receiveJointActuatorValue(actuator_id);

    result.at(0).value = result.at(0).value * manipulator_.getJointCoefficient(joint_component_name);
    result.at(0).velocity = result.at(0).velocity * manipulator_.getJointCoefficient(joint_component_name);
    result.at(0).effort = result.at(0).velocity * manipulator_.getJointCoefficient(joint_component_name);

    manipulator_.setJointValue(joint_component_name, result.at(0));
    return result.at(0);
  }
}

std::vector<WayPoint> RobotisManipulator::receiveMultipleJointActuatorValue(std::vector<Name> joint_component_name)
{
  if(using_platform_)
  {
    std::vector<Actuator> get_value_vector;
    std::vector<uint8_t> get_actuator_id;

    std::vector<Actuator> single_value_vector;
    std::vector<uint8_t> single_actuator_id;
    for(it_joint_actuator_ = joint_actuator_.begin(); it_joint_actuator_ != joint_actuator_.end(); it_joint_actuator_++)
    {
      single_actuator_id = joint_actuator_.at(it_joint_actuator_->first)->getId();
      single_value_vector = joint_actuator_.at(it_joint_actuator_->first)->receiveJointActuatorValue(single_actuator_id);
      for(int index=0; index < single_actuator_id.size(); index++)
      {
        get_actuator_id.push_back(single_actuator_id.at(index));
        get_value_vector.push_back(single_value_vector.at(index));
      }
    }

    std::vector<WayPoint> result_vector;
    WayPoint result;

    for(int index = 0; index < joint_component_name.size(); index++)
    {
      for(int index2 = 0; index2 < get_actuator_id.size(); index2++)
      {
        if(manipulator_.getJointId(joint_component_name.at(index)) == get_actuator_id.at(index2))
        {
          result.value = get_value_vector.at(index2).value * manipulator_.getJointCoefficient(joint_component_name.at(index));
          result.velocity = get_value_vector.at(index2).velocity * manipulator_.getJointCoefficient(joint_component_name.at(index));
          result.effort = get_value_vector.at(index2).effort * manipulator_.getJointCoefficient(joint_component_name.at(index));
          manipulator_.setJointValue(joint_component_name.at(index), result);
          result_vector.push_back(result);
        }
      }
    }

    return result_vector;
  }
}

std::vector<WayPoint> RobotisManipulator::receiveAllJointActuatorValue()
{
  if(using_platform_)
  {
    std::vector<Actuator> get_value_vector;
    std::vector<uint8_t> get_actuator_id;

    std::vector<Actuator> single_value_vector;
    std::vector<uint8_t> single_actuator_id;
    for(it_joint_actuator_ = joint_actuator_.begin(); it_joint_actuator_ != joint_actuator_.end(); it_joint_actuator_++)
    {
      single_actuator_id = joint_actuator_.at(it_joint_actuator_->first)->getId();
      single_value_vector = joint_actuator_.at(it_joint_actuator_->first)->receiveJointActuatorValue(single_actuator_id);
      for(int index=0; index < single_actuator_id.size(); index++)
      {
        get_actuator_id.push_back(single_actuator_id.at(index));
        get_value_vector.push_back(single_value_vector.at(index));
      }
    }

    std::map<Name, Component>::iterator it;
    std::vector<WayPoint> result_vector;
    WayPoint result;

    for (it = manipulator_.getIteratorBegin(); it != manipulator_.getIteratorEnd(); it++)
    {
      for(int index2 = 0; index2 < get_actuator_id.size(); index2++)
      {
        if(manipulator_.getJointId(it->first) == get_actuator_id.at(index2))
        {
          result.value = get_value_vector.at(index2).value * manipulator_.getJointCoefficient(it->first);
          result.velocity = get_value_vector.at(index2).velocity * manipulator_.getJointCoefficient(it->first);
          result.effort = get_value_vector.at(index2).effort * manipulator_.getJointCoefficient(it->first);
          manipulator_.setJointValue(it->first, result);
          result_vector.push_back(result);
        }
      }
    }

    return result_vector;
  }
}
/////////////////////////////////////////

bool RobotisManipulator::sendToolActuatorValue(Name tool_component_name, double value)
{
  manipulator_.setToolGoalValue(tool_component_name, value / manipulator_.getToolCoefficient(tool_component_name));
  if(using_platform_)
  {
    return tool_actuator_.at(manipulator_.getComponentActuatorName(tool_component_name))
        ->sendToolActuatorValue(value / manipulator_.getToolCoefficient(tool_component_name));
  }
}

double RobotisManipulator::receiveToolActuatorValue(Name tool_component_name)
{
  if(using_platform_)
  {
    double result;
    result =  manipulator_.getToolCoefficient(tool_component_name) * tool_actuator_.at(manipulator_.getComponentActuatorName(tool_component_name))
          ->receiveToolActuatorValue();

    manipulator_.setToolValue(tool_component_name, result);
    return result;
  }
}

bool RobotisManipulator::sendAllToolActuatorValue(std::vector<Name> tool_component_name, std::vector<double> value_vector)
{
  for (int index = 0; index < tool_component_name.size(); index++)
    manipulator_.setToolGoalValue(tool_component_name.at(index), value_vector.at(index) / manipulator_.getToolCoefficient(tool_component_name.at(index)));

  if(using_platform_)
  {
    for (int index = 0; index < tool_component_name.size(); index++)
    {
      manipulator_.setToolGoalValue(tool_component_name.at(index), value_vector.at(index));
      tool_actuator_.at(manipulator_.getComponentActuatorName(tool_component_name.at(index)))->sendToolActuatorValue(value_vector.at(index)/manipulator_.getToolCoefficient(tool_component_name.at(index)));
    }
    return true;
  }
}

std::vector<double> RobotisManipulator::receiveAllToolActuatorValue(std::vector<Name> tool_component_name)
{
  if(using_platform_)
  {
    std::vector<double> result_vector;
    double result;
    for (int index = 0; index < tool_component_name.size(); index++)
    {
      result = tool_actuator_.at(manipulator_.getComponentActuatorName(tool_component_name.at(index)))->receiveToolActuatorValue() * manipulator_.getToolCoefficient(tool_component_name.at(index));
      manipulator_.setToolValue(tool_component_name.at(index), result);
      result_vector.push_back(result);
    }
    return result_vector;
  }
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

void RobotisManipulator::initTrajectoryWayPoint()
{
  trajectory_.manipulator_ = manipulator_;
  std::vector<WayPoint> joint_way_point_vector;
  WayPoint joint_way_point;
  std::vector<double> joint_value_vector;
  joint_value_vector = manipulator_.getAllActiveJointValue();

  for(int index=0; index < joint_value_vector.size(); index++)
  {
    joint_way_point.value = joint_value_vector.at(index);
    joint_way_point.velocity = 0.0;
    joint_way_point.effort = 0.0;
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

//  //dynamics (velocity)
//  std::map<Name, Component>::iterator it;
//  Eigen::VectorXd joint_velocity(trajectory_.manipulator_.getDOF());

//  Eigen::VectorXd pose_velocity(6);
//  Eigen::Vector3d linear_velocity;
//  Eigen::Vector3d angular_velocity;

//  int8_t index = 0;
//  for (it = trajectory_.manipulator_.getIteratorBegin(); it != trajectory_.manipulator_.getIteratorEnd(); it++)
//  {
//    if (trajectory_.manipulator_.getJointId(it->first) != -1) // Check whether Active or Passive
//    {
//      // Active
//      joint_velocity[index] = trajectory_.manipulator_.getJointVelocity(it->first);
//      index++;
//    }
//  }

//  for (it = trajectory_.manipulator_.getIteratorBegin(); it != trajectory_.manipulator_.getIteratorEnd(); it++)
//  {
//    if (trajectory_.manipulator_.getJointId(it->first) != -1)
//    {
//      for(int index2 = 0; index2 < trajectory_.manipulator_.getDOF(); index2++)
//      {
//        joint_velocity[index2] =trajectory_.manipulator_.getJointVelocity(it->first);
//      }
//      pose_velocity = kinematics_->jacobian(&trajectory_.manipulator_, it->first)*joint_velocity;
//      linear_velocity[0] = pose_velocity[0];
//      linear_velocity[1] = pose_velocity[1];
//      linear_velocity[2] = pose_velocity[2];
//      angular_velocity[0] = pose_velocity[3];
//      angular_velocity[1] = pose_velocity[4];
//      angular_velocity[2] = pose_velocity[5];
//      Dynamicpose dynamic_pose;
//      dynamic_pose.linear.velocity = linear_velocity;
//      dynamic_pose.angular.velocity = angular_velocity;
//      dynamic_pose.linear.effort = Eigen::Vector3d::Zero();
//      dynamic_pose.angular.effort = Eigen::Vector3d::Zero();

//      trajectory_.manipulator_.setComponentDynamicPoseToWorld(it->first, dynamic_pose);
//    }
//  }
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
      trajectory_.manipulator_.setJointeffort(it->first, joint_value_vector.at(index).effort);
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
    dynamic_pose.linear.effort[pos_count] = tool_value_vector.at(pos_count).effort;
  }

  Eigen::Vector3d orientation_value_vector;
  Eigen::Vector3d orientation_velocity_vector;
  Eigen::Vector3d orientation_effort_vector;
  for(int ori_count = 0; ori_count < 3; ori_count++)
  {
    orientation_value_vector[ori_count] = tool_value_vector.at(ori_count+3).value;
    orientation_velocity_vector[ori_count] = tool_value_vector.at(ori_count+3).velocity;
    orientation_effort_vector[ori_count] = tool_value_vector.at(ori_count+3).effort;
  }
  Eigen::Matrix3d orientation;
  orientation = RM_MATH::convertRPYToRotation(orientation_value_vector[0], orientation_value_vector[1], orientation_value_vector[2]);
  pose_to_world.orientation = orientation;
  dynamic_pose.angular.velocity = orientation_velocity_vector;
  dynamic_pose.angular.effort = orientation_effort_vector;

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
      result.effort = trajectory_.manipulator_.getJointeffort(it->first);

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
    result.effort = trajectory_.manipulator_.getComponentDynamicPoseToWorld(tool_name).linear.effort[pos_count];

    result_vector.push_back(result);
  }

  Eigen::Vector3d orientation_vector =  RM_MATH::convertRotationToRPY(trajectory_.manipulator_.getComponentPoseToWorld(tool_name).orientation);
  for(int ori_count = 0; ori_count < 3; ori_count++)
  {
    result.value = orientation_vector[ori_count];
    result.velocity = trajectory_.manipulator_.getComponentDynamicPoseToWorld(tool_name).angular.velocity[ori_count];
    result.effort = trajectory_.manipulator_.getComponentDynamicPoseToWorld(tool_name).angular.effort[ori_count];

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
  trajectory_.joint_.setJointNum(manipulator_.getDOF());
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

void RobotisManipulator::jointTrajectoryMoveToPresentValue(std::vector<double> goal_joint_angle, double move_time)
{
  std::vector<WayPoint> temp = getPresentJointWayPoint();
  std::vector<double> goal_joint_value;
  for(int i = 0; i < manipulator_.getDOF(); i ++)
    goal_joint_value.push_back(temp.at(i).value + goal_joint_angle.at(i));

  jointTrajectoryMove(goal_joint_value, move_time);
}

void RobotisManipulator::jointTrajectoryMove(std::vector<double> goal_joint_angle, double move_time)
{
  trajectory_.trajectory_type_ = JOINT_TRAJECTORY;

  clearStartWayPoint();
  clearGoalWayPoint();

  setMoveTime(move_time);

  std::vector<WayPoint> temp = getPresentJointWayPoint();
  setStartWayPoint(getPresentJointWayPoint());

  WayPoint goal_way_point;
  std::vector<WayPoint> goal_way_point_vector;
  for (uint8_t index = 0; index < trajectory_.manipulator_.getDOF(); index++)
  {
    goal_way_point.value = goal_joint_angle.at(index);
    goal_way_point.velocity = 0.0;
    goal_way_point.effort = 0.0;

    goal_way_point_vector.push_back(goal_way_point);
  }
  setGoalWayPoint(goal_way_point_vector);

  if(isMoving())
  {
    moving_=false;
    while(!step_moving_) ;
  }
  makeJointTrajectory();
  startMoving();
}

void RobotisManipulator::jointTrajectoryMove(Name tool_name, Eigen::Vector3d goal_position, double move_time)
{
  Pose goal_pose;

  goal_pose.position = goal_position;
  goal_pose.orientation = trajectory_.manipulator_.getComponentOrientationToWorld(tool_name);
  jointTrajectoryMove(tool_name, goal_pose, move_time);
}

void RobotisManipulator::jointTrajectoryMove(Name tool_name, Eigen::Matrix3d goal_orientation, double move_time)
{
  Pose goal_pose;

  goal_pose.position = trajectory_.manipulator_.getComponentPositionToWorld(tool_name);
  goal_pose.orientation = goal_orientation;
  jointTrajectoryMove(tool_name, goal_pose, move_time);
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
    goal_way_point.effort = 0.0;

    goal_way_point_vector.push_back(goal_way_point);
  }
  setGoalWayPoint(goal_way_point_vector);

  if(isMoving())
  {
    moving_=false;
    while(!step_moving_) ;
  }
  makeJointTrajectory();
  startMoving();
}

void RobotisManipulator::taskTrajectoryMoveToPresentPosition(Name tool_name, Eigen::Vector3d meter, double move_time)
{
  Pose goal_pose;

  goal_pose.position = trajectory_.manipulator_.getComponentPositionToWorld(tool_name) + meter;
  goal_pose.orientation = trajectory_.manipulator_.getComponentOrientationToWorld(tool_name);
  taskTrajectoryMove(tool_name, goal_pose, move_time);
}

void RobotisManipulator::taskTrajectoryMove(Name tool_name, Eigen::Vector3d goal_position, double move_time)
{
  Pose goal_pose;

  goal_pose.position = goal_position;
  goal_pose.orientation = trajectory_.manipulator_.getComponentOrientationToWorld(tool_name);
  taskTrajectoryMove(tool_name, goal_pose, move_time);
}

void RobotisManipulator::taskTrajectoryMove(Name tool_name, Eigen::Matrix3d goal_orientation, double move_time)
{
  Pose goal_pose;

  goal_pose.position = trajectory_.manipulator_.getComponentPositionToWorld(tool_name);
  goal_pose.orientation = goal_orientation;
  taskTrajectoryMove(tool_name, goal_pose, move_time);
}

void RobotisManipulator::taskTrajectoryMove(Name tool_name, Pose goal_pose, double move_time)
{
  trajectory_.trajectory_type_ = TASK_TRAJECTORY;
  trajectory_.present_controled_tool_name_ = tool_name;

  clearStartWayPoint();
  clearGoalWayPoint();

  setMoveTime(move_time);
  std::vector<WayPoint> temp = getPresentTaskWayPoint(tool_name);
  setStartWayPoint(getPresentTaskWayPoint(tool_name));

  Eigen::Vector3d goal_position_to_world = goal_pose.position;
  Eigen::Vector3d goal_orientation_to_world = RM_MATH::convertRotationToRPY(goal_pose.orientation);

  WayPoint goal_way_point;
  std::vector<WayPoint> goal_way_point_vector;

  for (uint8_t index = 0; index < 3; index++)
  {
    goal_way_point.value = goal_position_to_world[index];
    goal_way_point.velocity = 0.0;
    goal_way_point.effort =0.0;
    goal_way_point_vector.push_back(goal_way_point);
  }
  for (uint8_t index = 0; index < 3; index++)
  {
    goal_way_point.value = goal_orientation_to_world[index];
    goal_way_point.velocity = 0.0;
    goal_way_point.effort =0.0;
    goal_way_point_vector.push_back(goal_way_point);
  }
  setGoalWayPoint(goal_way_point_vector);

  if(isMoving())
  {
    moving_=false;
    while(!step_moving_) ;
  }
  makeTaskTrajectory();
  startMoving();
}

void RobotisManipulator::drawingTrajectoryMove(Name drawing_name, Name tool_name, const void *arg, double move_time)
{
  trajectory_.trajectory_type_ = DRAWING_TRAJECTORY;
  trajectory_.present_controled_tool_name_ = tool_name;

  trajectory_.drawing_.at(drawing_name)->output_way_point_type_ = TASK;
  trajectory_.present_drawing_object_name_ = drawing_name;

  clearStartWayPoint();
  clearGoalWayPoint();

  setMoveTime(move_time);

  setStartWayPoint(getPresentTaskWayPoint(tool_name));

  if(isMoving())
  {
    moving_=false;
    while(!step_moving_) ;
  }
  makeDrawingTrajectory(drawing_name, arg);
  startMoving();
}

void RobotisManipulator::drawingTrajectoryMove(Name drawing_name, const void *arg, double move_time)
{
  trajectory_.trajectory_type_ = DRAWING_TRAJECTORY;

  trajectory_.drawing_.at(drawing_name)->output_way_point_type_ = JOINT;
  trajectory_.present_drawing_object_name_ = drawing_name;

  clearStartWayPoint();
  clearGoalWayPoint();

  setMoveTime(move_time);

  setStartWayPoint(getPresentJointWayPoint());

  if(isMoving())
  {
    moving_=false;
    while(!step_moving_) ;
  }
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
    goal_way_point_vector.at(index).effort = 0.0;
  }
  setGoalWayPoint(goal_way_point_vector);

  if(isMoving())
  {
    moving_= false;
    while(!step_moving_) ;
  }
  makeJointTrajectory();
  startMoving();
}

std::vector<Actuator> RobotisManipulator::getTrajectoryJointValue(double tick_time)
{
  std::vector<WayPoint> joint_way_point_value;

  if(trajectory_.trajectory_type_ == JOINT_TRAJECTORY)
  {
    joint_way_point_value = trajectory_.joint_.getJointWayPoint(tick_time);
    setPresentJointWayPoint(joint_way_point_value);
    UpdatePresentWayPoint();
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
    goal_pose.orientation = RM_MATH::convertRPYToRotation(task_way_point_value.at(3).value, task_way_point_value.at(4).value, task_way_point_value.at(5).value);
    joint_value = kinematics_->inverse(&trajectory_.manipulator_, trajectory_.present_controled_tool_name_, goal_pose);
    joint_way_point_value.resize(joint_value.size());

    for(int index = 0; index < joint_value.size(); index++)
    {
      joint_way_point_value.at(index).value = joint_value.at(index);
      joint_way_point_value.at(index).velocity = 0.0;
      joint_way_point_value.at(index).effort = 0.0;
    }
    setPresentJointWayPoint(joint_way_point_value);
  }
  else if(trajectory_.trajectory_type_ == DRAWING_TRAJECTORY)
  {
    if(trajectory_.drawing_.at(trajectory_.present_drawing_object_name_)->output_way_point_type_==JOINT)
    {
      joint_way_point_value = trajectory_.drawing_.at(trajectory_.present_drawing_object_name_)->getJointWayPoint(tick_time);
      setPresentJointWayPoint(joint_way_point_value);
      UpdatePresentWayPoint();
    }
    else if(trajectory_.drawing_.at(trajectory_.present_drawing_object_name_)->output_way_point_type_==TASK)
    {
      std::vector<WayPoint> task_way_point_value;
      Pose goal_pose;
      std::vector<double> joint_value;
      task_way_point_value = trajectory_.drawing_.at(trajectory_.present_drawing_object_name_)->getTaskWayPoint(tick_time);
      setPresentTaskWayPoint(trajectory_.present_controled_tool_name_, task_way_point_value);

      goal_pose.position[0] = task_way_point_value.at(0).value;
      goal_pose.position[1] = task_way_point_value.at(1).value;
      goal_pose.position[2] = task_way_point_value.at(2).value;
      goal_pose.orientation = RM_MATH::convertRPYToRotation(task_way_point_value.at(3).value, task_way_point_value.at(4).value, task_way_point_value.at(5).value);

      joint_value = kinematics_->inverse(&trajectory_.manipulator_, trajectory_.present_controled_tool_name_, goal_pose);
      joint_way_point_value.resize(joint_value.size());

      for(int index = 0; index < joint_value.size(); index++)
      {
        joint_way_point_value.at(index).value = joint_value.at(index);
        joint_way_point_value.at(index).velocity = 0.0;
        joint_way_point_value.at(index).effort = 0.0;
      }
      setPresentJointWayPoint(joint_way_point_value);
    }
  }

  return joint_way_point_value;
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


std::vector<WayPoint> RobotisManipulator::trajectoryControllerLoop(double present_time)
{
  setPresentTime(present_time);

  if(moving_)
  {
    step_moving_ = false;
    std::vector<WayPoint> joint_goal_way_point;
    joint_goal_way_point = TrajectoryTimeCounter();
    step_moving_ = true;
    return joint_goal_way_point;
  }
  return {};
}

void RobotisManipulator::toolMove(Name tool_name, double tool_value)
{
  manipulator_.setToolGoalValue(tool_name, tool_value);
  if(using_platform_)
  {
    tool_actuator_.at(manipulator_.getComponentActuatorName(tool_name))
        ->sendToolActuatorValue(tool_value );
  }
}



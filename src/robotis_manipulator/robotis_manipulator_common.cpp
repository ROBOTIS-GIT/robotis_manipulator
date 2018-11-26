/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "../../include/robotis_manipulator/robotis_manipulator_common.h"

using namespace ROBOTIS_MANIPULATOR;

/////////////////////Manipulator class//////////////////////
Manipulator::Manipulator()
    :dof_(0)
{}
///////////////////////////////add function//////////////////////////////////
void Manipulator::addWorld(Name world_name,
                           Name child_name,
                           Eigen::Vector3d world_position,
                           Eigen::Matrix3d world_orientation)
{
  world_.name = world_name;
  world_.child = child_name;
  world_.pose.position = world_position;
  world_.pose.orientation = world_orientation;
  world_.dynamic_pose.linear.velocity = Eigen::Vector3d::Zero(3);
  world_.dynamic_pose.linear.effort = Eigen::Vector3d::Zero(3);
  world_.dynamic_pose.angular.velocity = Eigen::Vector3d::Zero(3);
  world_.dynamic_pose.angular.effort = Eigen::Vector3d::Zero(3);
}

void Manipulator::addJoint(Name my_name,
                               Name parent_name,
                               Name child_name,
                               Eigen::Vector3d relative_position,
                               Eigen::Matrix3d relative_orientation,
                               Eigen::Vector3d axis_of_rotation,
                               int8_t joint_actuator_id,
                               double max_limit,
                               double min_limit,
                               double coefficient,
                               double mass,
                               Eigen::Matrix3d inertia_tensor,
                               Eigen::Vector3d center_of_mass)
{
  if (joint_actuator_id != -1)
    dof_++;

  Component temp_component;

  temp_component.name.parent = parent_name;
  temp_component.name.child.push_back(child_name);
  temp_component.component_type = JOINT_COMPONET;
  temp_component.relative.pose_from_parent.position = relative_position;
  temp_component.relative.pose_from_parent.orientation = relative_orientation;
  temp_component.relative.inertia.mass = mass;
  temp_component.relative.inertia.inertia_tensor = inertia_tensor;
  temp_component.relative.inertia.center_of_mass = center_of_mass;
  temp_component.actuator_constant.id = joint_actuator_id;
  temp_component.actuator_constant.coefficient = coefficient;
  temp_component.actuator_constant.axis = axis_of_rotation;
  temp_component.actuator_constant.limit.maximum = max_limit;
  temp_component.actuator_constant.limit.minimum = min_limit;

  temp_component.from_world.pose.position = Eigen::Vector3d::Zero();
  temp_component.from_world.pose.orientation = Eigen::Matrix3d::Identity();
  temp_component.from_world.dynamic_pose.linear.velocity = Eigen::Vector3d::Zero(3);
  temp_component.from_world.dynamic_pose.linear.effort = Eigen::Vector3d::Zero(3);
  temp_component.from_world.dynamic_pose.angular.velocity = Eigen::Vector3d::Zero(3);
  temp_component.from_world.dynamic_pose.angular.effort = Eigen::Vector3d::Zero(3);

  temp_component.actuator_variable.value = 0.0;
  temp_component.actuator_variable.velocity = 0.0;
  temp_component.actuator_variable.effort = 0.0;
  temp_component.actuator_variable.goal_value = 0.0;

  component_.insert(std::make_pair(my_name, temp_component));
}

void Manipulator::addComponentChild(Name my_name, Name child_name)
{
  component_.at(my_name).name.child.push_back(child_name);
}

void Manipulator::addTool(Name my_name,
                          Name parent_name,
                          Eigen::Vector3d relative_position,
                          Eigen::Matrix3d relative_orientation,
                          int8_t tool_id,
                          double max_limit,
                          double min_limit,
                          double coefficient,
                          double mass,
                          Eigen::Matrix3d inertia_tensor,
                          Eigen::Vector3d center_of_mass)
{
  Component temp_component;

  temp_component.name.parent = parent_name;
  temp_component.name.child.resize(0);
  temp_component.component_type = TOOL_COMPONET;
  temp_component.relative.pose_from_parent.position = relative_position;
  temp_component.relative.pose_from_parent.orientation = relative_orientation;
  temp_component.relative.inertia.mass = mass;
  temp_component.relative.inertia.inertia_tensor = inertia_tensor;
  temp_component.relative.inertia.center_of_mass = center_of_mass;
  temp_component.actuator_constant.id = tool_id;
  temp_component.actuator_constant.coefficient = coefficient;
  temp_component.actuator_constant.axis = Eigen::Vector3d::Zero();
  temp_component.actuator_constant.limit.maximum = max_limit;
  temp_component.actuator_constant.limit.minimum = min_limit;

  temp_component.from_world.pose.position = Eigen::Vector3d::Zero();
  temp_component.from_world.pose.orientation = Eigen::Matrix3d::Identity();
  temp_component.from_world.dynamic_pose.linear.velocity = Eigen::Vector3d::Zero(3);
  temp_component.from_world.dynamic_pose.linear.effort = Eigen::Vector3d::Zero(3);
  temp_component.from_world.dynamic_pose.angular.velocity = Eigen::Vector3d::Zero(3);
  temp_component.from_world.dynamic_pose.angular.effort = Eigen::Vector3d::Zero(3);

  temp_component.actuator_variable.value = 0.0;
  temp_component.actuator_variable.velocity = 0.0;
  temp_component.actuator_variable.effort = 0.0;
  temp_component.actuator_variable.goal_value = 0.0;

  component_.insert(std::make_pair(my_name, temp_component));
}

void Manipulator::checkManipulatorSetting()
{
  //use debug
}

///////////////////////////////Set function//////////////////////////////////
void Manipulator::setWorldPose(Pose world_pose)
{
  world_.pose = world_pose;
}

void Manipulator::setWorldPosition(Eigen::Vector3d world_position)
{
  world_.pose.position = world_position;
}

void Manipulator::setWorldOrientation(Eigen::Matrix3d world_orientation)
{
  world_.pose.orientation = world_orientation;
}

void Manipulator::setWorldDynamicPose(Dynamicpose world_dynamic_pose)
{
  world_.dynamic_pose = world_dynamic_pose;
}

void Manipulator::setWorldLinearVelocity(Eigen::Vector3d world_linear_velocity)
{
  world_.dynamic_pose.linear.velocity = world_linear_velocity;
}

void Manipulator::setWorldAngularVelocity(Eigen::Vector3d world_angular_velocity)
{
  world_.dynamic_pose.angular.velocity = world_angular_velocity;
}

void Manipulator::setWorldLinearEffort(Eigen::Vector3d world_linear_effort)
{
  world_.dynamic_pose.linear.effort = world_linear_effort;
}

void Manipulator::setWorldAngularEffort(Eigen::Vector3d world_angular_effort)
{
  world_.dynamic_pose.angular.effort = world_angular_effort;
}

void Manipulator::setComponent(Name component_name, Component component)
{
  component_.at(component_name) = component;
}

void Manipulator::setComponentActuatorName(Name component_name, Name actuator_name)
{
  component_.at(component_name).actuator_name = actuator_name;
}

void Manipulator::setComponentPoseToWorld(Name name, Pose pose_to_world)
{
  if (component_.find(name) != component_.end())
  {
    component_.at(name).from_world.pose = pose_to_world;
  }
  else
  {
    RM_LOG::ERROR("[setComponentPoseToWorld] Wrong name.");
  }
}

void Manipulator::setComponentPositionToWorld(Name name, Eigen::Vector3d position_to_world)
{
  if (component_.find(name) != component_.end())
  {
    component_.at(name).from_world.pose.position = position_to_world;
  }
  else
  {
    RM_LOG::ERROR("[setComponentPositionToWorld] Wrong name.");
  }
}

void Manipulator::setComponentOrientationToWorld(Name name, Eigen::Matrix3d orientation_to_wolrd)
{
  if (component_.find(name) != component_.end())
  {
    component_.at(name).from_world.pose.orientation = orientation_to_wolrd;
  }
  else
  {
    RM_LOG::ERROR("[setComponentOrientationToWorld] Wrong name.");
  }
}

void Manipulator::setComponentDynamicPoseToWorld(Name name, Dynamicpose dynamic_pose)
{
  if (component_.find(name) != component_.end())
  {
    component_.at(name).from_world.dynamic_pose = dynamic_pose;
  }
  else
  {
    RM_LOG::ERROR("[setComponentDynamicPoseToWorld] Wrong name.");
  }
}

void Manipulator::setJointValue(Name name, double joint_value)
{
  if (component_.at(name).component_type == JOINT_COMPONET)
  {
    component_.at(name).actuator_variable.value = joint_value;
  }
  else
  {
    RM_LOG::ERROR("[setJointValue] Input name is not joint name.");
  }
}

void Manipulator::setJointVelocity(Name name, double joint_velocity)
{
  if (component_.at(name).component_type == JOINT_COMPONET)
  {
    component_.at(name).actuator_variable.velocity = joint_velocity;
  }
  else
  {
    RM_LOG::ERROR("[setJointVelocity] Input name is not joint name.");
  }
}

void Manipulator::setJointAcceleration(Name name, double joint_acceleration)
{
  if (component_.at(name).component_type == JOINT_COMPONET)
  {
    component_.at(name).actuator_variable.acceleration = joint_acceleration;
  }
  else
  {
    RM_LOG::ERROR("[setJointAcceleration] Input name is not joint name.");
  }
}

void Manipulator::setJointEffort(Name name, double joint_effort)
{
  if (component_.at(name).component_type == JOINT_COMPONET)
  {
    component_.at(name).actuator_variable.effort = joint_effort;
  }
  else
  {
    RM_LOG::ERROR("[setJointEffort] Input name is not joint name.");
  }
}

void Manipulator::setJointValue(Name name, WayPoint joint_value)
{
  if (component_.at(name).component_type == JOINT_COMPONET)
  {
    component_.at(name).actuator_variable.value = joint_value.value;
    component_.at(name).actuator_variable.velocity = joint_value.velocity;
    component_.at(name).actuator_variable.acceleration = joint_value.acceleration;
    component_.at(name).actuator_variable.effort = joint_value.effort;
  }
  else
  {
    RM_LOG::ERROR("[setJointValue] Input name is not joint name.");
  }
}

void Manipulator::setAllActiveJointValue(std::vector<double> joint_value_vector)
{
  int8_t index = 0;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (component_.at(it_component->first).component_type == JOINT_COMPONET)
    {
      if (component_.at(it_component->first).actuator_constant.id != -1)
      {
        component_.at(it_component->first).actuator_variable.value = joint_value_vector.at(index);
        index++;
      }
    }
  }
}

void Manipulator::setAllActiveJointValue(std::vector<WayPoint> joint_way_point_vector)
{
  int8_t index = 0;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (component_.at(it_component->first).component_type == JOINT_COMPONET)
    {
      if (component_.at(it_component->first).actuator_constant.id != -1)
      {
        component_.at(it_component->first).actuator_variable.value = joint_way_point_vector.at(index).value;
        component_.at(it_component->first).actuator_variable.velocity = joint_way_point_vector.at(index).velocity;
        component_.at(it_component->first).actuator_variable.acceleration = joint_way_point_vector.at(index).acceleration;
        component_.at(it_component->first).actuator_variable.effort = joint_way_point_vector.at(index).effort;
        index++;
      }
    }
  }
}

void Manipulator::setAllJointValue(std::vector<double> joint_value_vector)
{
  int8_t index = 0;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (component_.at(it_component->first).component_type == JOINT_COMPONET)
    {
      component_.at(it_component->first).actuator_variable.value = joint_value_vector.at(index);
      index++;
    }
  }
}


void Manipulator::setAllJointValue(std::vector<WayPoint> joint_way_point_vector)
{
  int8_t index = 0;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (component_.at(it_component->first).component_type == JOINT_COMPONET)
    {
      component_.at(it_component->first).actuator_variable.value = joint_way_point_vector.at(index).value;
      component_.at(it_component->first).actuator_variable.velocity = joint_way_point_vector.at(index).velocity;
      component_.at(it_component->first).actuator_variable.acceleration = joint_way_point_vector.at(index).acceleration;
      component_.at(it_component->first).actuator_variable.effort = joint_way_point_vector.at(index).effort;
      index++;
    }
  }
}

void Manipulator::setToolValue(Name name, double tool_value)
{
  if (component_.at(name).component_type == TOOL_COMPONET)
  {
    component_.at(name).actuator_variable.value = tool_value;
  }
  else
  {
    RM_LOG::ERROR("[setToolValue] Input name is not tool name.");
  }
}

void Manipulator::setToolGoalValue(Name name, double tool_goal_value)
{
  if (component_.at(name).component_type == TOOL_COMPONET)
  {
    component_.at(name).actuator_variable.goal_value = tool_goal_value;
  }
  else
  {
    RM_LOG::ERROR("[setToolValue] Input name is not tool name.");
  }
}


///////////////////////////////Get function//////////////////////////////////

int8_t Manipulator::getDOF()
{
  return dof_;
}

Name Manipulator::getWorldName()
{
  return world_.name;
}

Name Manipulator::getWorldChildName()
{
  return world_.child;
}

Pose Manipulator::getWorldPose()
{
  return world_.pose;
}

Eigen::Vector3d Manipulator::getWorldPosition()
{
  return world_.pose.position;
}

Eigen::Matrix3d Manipulator::getWorldOrientation()
{
  return world_.pose.orientation;
}

Dynamicpose Manipulator::getWorldDynamicPose()
{
  return world_.dynamic_pose;
}

int8_t Manipulator::getComponentSize()
{
  return component_.size();
}

std::map<Name, Component> Manipulator::getAllComponent()
{
  return component_;
}

std::map<Name, Component>::iterator Manipulator::getIteratorBegin()
{
  return component_.begin();
}

std::map<Name, Component>::iterator Manipulator::getIteratorEnd()
{
  return component_.end();;
}

Component Manipulator::getComponent(Name name)
{
  return component_.at(name);
}

Name Manipulator::getComponentActuatorName(Name component_name)
{
  return component_.at(component_name).actuator_name;
}

Name Manipulator::getComponentParentName(Name name)
{
  return component_.at(name).name.parent;
}

std::vector<Name> Manipulator::getComponentChildName(Name name)
{
  return component_.at(name).name.child;
}

Pose Manipulator::getComponentPoseToWorld(Name name)
{
  return component_.at(name).from_world.pose;
}

Eigen::Vector3d Manipulator::getComponentPositionToWorld(Name name)
{
  return component_.at(name).from_world.pose.position;
}

Eigen::Matrix3d Manipulator::getComponentOrientationToWorld(Name name)
{
  return component_.at(name).from_world.pose.orientation;
}

Dynamicpose Manipulator::getComponentDynamicPoseToWorld(Name name)
{
  return component_.at(name).from_world.dynamic_pose;
}

Pose Manipulator::getComponentRelativePoseToParent(Name name)
{
  return component_.at(name).relative.pose_from_parent;
}

Eigen::Vector3d Manipulator::getComponentRelativePositionToParent(Name name)
{
  return component_.at(name).relative.pose_from_parent.position;
}

Eigen::Matrix3d Manipulator::getComponentRelativeOrientationToParent(Name name)
{
  return component_.at(name).relative.pose_from_parent.orientation;
}

int8_t Manipulator::getJointId(Name name)
{
  if (component_.at(name).component_type == JOINT_COMPONET)
  {
    return component_.at(name).actuator_constant.id;
  }
  else
  {
    RM_LOG::ERROR("[getJointId] Input name is not joint name");
    return component_.at(name).actuator_constant.id;
  }
}

double Manipulator::getJointCoefficient(Name name)
{
  if (component_.at(name).component_type == JOINT_COMPONET)
  {
    return component_.at(name).actuator_constant.coefficient;
  }
  else
  {
    RM_LOG::ERROR("[getJointCoefficient] Input name is not joint name");
    return component_.at(name).actuator_constant.coefficient;
  }
}

Eigen::Vector3d Manipulator::getJointAxis(Name name)
{
  if (component_.at(name).component_type == JOINT_COMPONET)
  {
    return component_.at(name).actuator_constant.axis;
  }
  else
  {
    RM_LOG::ERROR("[getJointAxis] Input name is not joint name");
    return component_.at(name).actuator_constant.axis;
  }
}

double Manipulator::getJointValue(Name name)
{
  if (component_.at(name).component_type == JOINT_COMPONET)
  {
    return component_.at(name).actuator_variable.value;
  }
  else
  {
    RM_LOG::ERROR("[getJointValue] Input name is not joint name");
    return component_.at(name).actuator_variable.value;
  }
}

double Manipulator::getJointVelocity(Name name)
{
  if (component_.at(name).component_type == JOINT_COMPONET)
  {
    return component_.at(name).actuator_variable.velocity;
  }
  else
  {
    RM_LOG::ERROR("[getJointVelocity] Input name is not joint name");
    return component_.at(name).actuator_variable.velocity;
  }
}

double Manipulator::getJointAcceleration(Name name)
{
  if (component_.at(name).component_type == JOINT_COMPONET)
  {
    return component_.at(name).actuator_variable.acceleration;
  }
  else
  {
    RM_LOG::ERROR("[getJointAcceleration] Input name is not joint name");
    return component_.at(name).actuator_variable.acceleration;
  }
}

double Manipulator::getJointEffort(Name name)
{
  if (component_.at(name).component_type == JOINT_COMPONET)
  {
    return component_.at(name).actuator_variable.effort;
  }
  else
  {
    RM_LOG::ERROR("[getJointEffort] Input name is not joint name");
    return component_.at(name).actuator_variable.effort;
  }
}


int8_t Manipulator::getToolId(Name name)
{
  if (component_.at(name).component_type == TOOL_COMPONET)
  {
    return component_.at(name).actuator_constant.id;
  }
  else
  {
    RM_LOG::ERROR("[getJointEffort] Input name is not tool name");
    return component_.at(name).actuator_constant.id;
  }
}

double Manipulator::getToolCoefficient(Name name)
{
  if (component_.at(name).component_type == TOOL_COMPONET)
  {
    return component_.at(name).actuator_constant.coefficient;
  }
  else
  {
    RM_LOG::ERROR("[getToolCoefficient] Input name is not tool name");
    return component_.at(name).actuator_constant.coefficient;
  }
}

double Manipulator::getToolValue(Name name)
{
  if (component_.at(name).component_type == TOOL_COMPONET)
  {
    return component_.at(name).actuator_variable.value;
  }
  else
  {
    RM_LOG::ERROR("[getToolValue] Input name is not tool name");
    return component_.at(name).actuator_variable.value;
  }
}

double Manipulator::getToolGoalValue(Name name)
{
  if (component_.at(name).component_type == TOOL_COMPONET)
  {
    return component_.at(name).actuator_variable.goal_value;
  }
  else
  {
    RM_LOG::ERROR("[getToolValue] Input name is not tool name");
    return component_.at(name).actuator_variable.goal_value;
  }
}

double Manipulator::getComponentMass(Name name)
{
  return component_.at(name).relative.inertia.mass;
}

Eigen::Matrix3d Manipulator::getComponentInertiaTensor(Name name)
{
  return component_.at(name).relative.inertia.inertia_tensor;
}

Eigen::Vector3d Manipulator::getComponentCenterOfMass(Name name)
{
  return component_.at(name).relative.inertia.center_of_mass;
}

std::vector<double> Manipulator::getAllJointValue()
{
  std::vector<double> result_vector;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (component_.at(it_component->first).component_type == JOINT_COMPONET)
    {
      result_vector.push_back(component_.at(it_component->first).actuator_variable.value);
    }
  }
  return result_vector;
}

std::vector<double> Manipulator::getAllActiveJointValue()
{
  std::vector<double> result_vector;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (component_.at(it_component->first).component_type == JOINT_COMPONET)
    {
      if (component_.at(it_component->first).actuator_constant.id != -1) // Check whether Active or Passive
      {
        result_vector.push_back(component_.at(it_component->first).actuator_variable.value);
      }
    }
  }
  return result_vector;
}

void Manipulator::getAllActiveJointValue(std::vector<double> *joint_value_vector, std::vector<double> *joint_velocity_vector, std::vector<double> *joint_accelerarion_vector, std::vector<double> *joint_effort_vector)
{
  std::map<Name, Component>::iterator it_component;

  joint_value_vector->clear();
  joint_velocity_vector->clear();
  joint_accelerarion_vector->clear();
  joint_effort_vector->clear();

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (component_.at(it_component->first).component_type == JOINT_COMPONET)
    {
      if (component_.at(it_component->first).actuator_constant.id != -1) // Check whether Active or Passive
      {
        joint_value_vector->push_back(component_.at(it_component->first).actuator_variable.value);
        joint_velocity_vector->push_back(component_.at(it_component->first).actuator_variable.velocity);
        joint_accelerarion_vector->push_back(component_.at(it_component->first).actuator_variable.acceleration);
        joint_effort_vector->push_back(component_.at(it_component->first).actuator_variable.effort);
      }
    }
  }
}

std::vector<uint8_t> Manipulator::getAllJointID()
{
  std::vector<uint8_t> joint_id;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (component_.at(it_component->first).component_type == JOINT_COMPONET)
    {
      joint_id.push_back(component_.at(it_component->first).actuator_constant.id);
    }
  }
  return joint_id;
}

std::vector<uint8_t> Manipulator::getAllActiveJointID()
{
  std::vector<uint8_t> active_joint_id;
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (component_.at(it_component->first).component_type == JOINT_COMPONET)
    {
      if (component_.at(it_component->first).actuator_constant.id != -1) // Check whether Active or Passive
      {
        active_joint_id.push_back(component_.at(it_component->first).actuator_constant.id);
      }
    }
  }
  return active_joint_id;
}


Name Manipulator::findJointComponentNameFromId(int8_t id)
{
  std::map<Name, Component>::iterator it_component;

  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (component_.at(it_component->first).component_type == JOINT_COMPONET)
    {
      if (component_.at(it_component->first).actuator_constant.id == id)
      {
        return it_component->first;
      }
    }
  }
}

Name Manipulator::findToolComponentNameFromId(int8_t id)
{
  std::map<Name, Component>::iterator it_component;
  for (it_component = component_.begin(); it_component != component_.end(); it_component++)
  {
    if (component_.at(it_component->first).component_type == TOOL_COMPONET)
    {
      if (component_.at(it_component->first).actuator_constant.id == id)
      {
        return it_component->first;
      }
    }
  }
}

////////////////////////////////////////////////////////////

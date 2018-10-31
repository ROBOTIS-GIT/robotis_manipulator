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
  manipulator_.world.name = world_name;
  manipulator_.world.child = child_name;
  manipulator_.world.pose.position = world_position;
  manipulator_.world.pose.orientation = world_orientation;
  manipulator_.world.dynamic_pose.linear.velocity = Eigen::Vector3d::Zero(3);
  manipulator_.world.dynamic_pose.linear.accelation = Eigen::Vector3d::Zero(3);
  manipulator_.world.dynamic_pose.angular.velocity = Eigen::Vector3d::Zero(3);
  manipulator_.world.dynamic_pose.angular.accelation = Eigen::Vector3d::Zero(3);
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
  if (joint_actuator_id != -1)
    manipulator_.dof++;

  Component temp_component;

  temp_component.parent = parent_name;
  temp_component.child.push_back(child_name);
  temp_component.relative_to_parent.position = relative_position;
  temp_component.relative_to_parent.orientation = relative_orientation;
  temp_component.pose_to_world.position = Eigen::Vector3d::Zero();
  temp_component.pose_to_world.orientation = Eigen::Quaterniond::Identity();
  temp_component.dynamic_pose.linear.velocity = Eigen::Vector3d::Zero(3);
  temp_component.dynamic_pose.linear.accelation = Eigen::Vector3d::Zero(3);
  temp_component.dynamic_pose.angular.velocity = Eigen::Vector3d::Zero(3);
  temp_component.dynamic_pose.angular.accelation = Eigen::Vector3d::Zero(3);
  temp_component.joint.id = joint_actuator_id;
  temp_component.joint.coefficient = coefficient;
  temp_component.joint.axis = axis_of_rotation;
  temp_component.joint.value = 0.0;
  temp_component.joint.velocity = 0.0;
  temp_component.joint.acceleration = 0.0;
  temp_component.tool.id = -1;
  temp_component.tool.coefficient = 0;
  temp_component.tool.value = 0.0;
  temp_component.inertia.mass = mass;
  temp_component.inertia.inertia_tensor = inertia_tensor;
  temp_component.inertia.center_of_mass = center_of_mass;

  manipulator_.component.insert(std::make_pair(my_name, temp_component));
}

void RobotisManipulator::addComponentChild(Name my_name, Name child_name)
{
  manipulator_.component.at(my_name).child.push_back(child_name);
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
  Component temp_component;

  temp_component.parent = parent_name;
  temp_component.relative_to_parent.position = relative_position;
  temp_component.relative_to_parent.orientation = relative_orientation;
  temp_component.pose_to_world.position = Eigen::Vector3d::Zero();
  temp_component.pose_to_world.orientation = Eigen::Quaterniond::Identity();
  temp_component.dynamic_pose.linear.velocity = Eigen::Vector3d::Zero(3);
  temp_component.dynamic_pose.linear.accelation = Eigen::Vector3d::Zero(3);
  temp_component.dynamic_pose.angular.velocity = Eigen::Vector3d::Zero(3);
  temp_component.dynamic_pose.angular.accelation = Eigen::Vector3d::Zero(3);
  temp_component.joint.id = -1;
  temp_component.joint.coefficient = 0;
  temp_component.joint.axis = Eigen::Vector3d::Zero();
  temp_component.joint.value = 0.0;
  temp_component.joint.velocity = 0.0;
  temp_component.joint.acceleration = 0.0;
  temp_component.tool.id = tool_id;
  temp_component.tool.coefficient = coefficient;

  temp_component.tool.value = 0.0;
  temp_component.inertia.mass = mass;
  temp_component.inertia.inertia_tensor = inertia_tensor;
  temp_component.inertia.center_of_mass = center_of_mass;

  manipulator_.component.insert(std::make_pair(my_name, temp_component));
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

///////////////////////////////Set function//////////////////////////////////
void RobotisManipulator::setWorldPose(Pose world_pose)
{
  manipulator_.world.pose = world_pose;
}

void RobotisManipulator::setWorldPosition(Eigen::Vector3d world_position)
{
  manipulator_.world.pose.position = world_position;
}

void RobotisManipulator::setWorldOrientation(Eigen::Quaterniond world_orientation)
{
  manipulator_.world.pose.orientation = world_orientation;
}

void RobotisManipulator::setWorldDynamicPose(Dynamicpose world_dynamic_pose)
{
  manipulator_.world.dynamic_pose = world_dynamic_pose;
}

void RobotisManipulator::setWorldLinearVelocity(Eigen::Vector3d world_linear_velocity)
{
  manipulator_.world.dynamic_pose.linear.velocity = world_linear_velocity;
}

void RobotisManipulator::setWorldAngularVelocity(Eigen::Vector3d world_angular_velocity)
{
  manipulator_.world.dynamic_pose.angular.velocity = world_angular_velocity;
}

void RobotisManipulator::setWorldLinearAcceleration(Eigen::Vector3d world_linear_acceleration)
{
  manipulator_.world.dynamic_pose.linear.accelation = world_linear_acceleration;
}

void RobotisManipulator::setWorldAngularAcceleration(Eigen::Vector3d world_angular_acceleration)
{
  manipulator_.world.dynamic_pose.angular.accelation = world_angular_acceleration;
}


void RobotisManipulator::setComponentPoseToWorld(Name name, Pose pose_to_world)
{
  if (manipulator_.component.find(name) != manipulator_.component.end())
  {
    manipulator_.component.at(name).pose_to_world = pose_to_world;
  }
  else
  {
    //error
  }
}

void RobotisManipulator::setComponentPositionToWorld(Name name, Eigen::Vector3d position_to_world)
{
  if (manipulator_.component.find(name) != manipulator_.component.end())
  {
    manipulator_.component.at(name).pose_to_world.position = position_to_world;
  }
  else
  {
    //error
  }
}

void RobotisManipulator::setComponentOrientationToWorld(Name name, Eigen::Quaterniond orientation_to_wolrd)
{
  if (manipulator_.component.find(name) != manipulator_.component.end())
  {
    manipulator_.component.at(name).pose_to_world.orientation = orientation_to_wolrd;
  }
  else
  {
    //error
  }
}

void RobotisManipulator::setComponentDynamicPoseToWorld(Name name, Dynamicpose dynamic_pose)
{
  if (manipulator_.component.find(name) != manipulator_.component.end())
  {
    manipulator_.component.at(name).dynamic_pose = dynamic_pose;
  }
  else
  {
    //error
  }
}

void RobotisManipulator::setJointValue(Name name, double joint_value)
{
  if (manipulator_.component.at(name).tool.id > 0)
  {
    //error
  }
  else
  {
    if (manipulator_.component.find(name) != manipulator_.component.end())
    {
      manipulator_.component.at(name).joint.value = joint_value;
    }
    else
    {
      //error
    }
  }
}

void RobotisManipulator::setJointVelocity(Name name, double joint_velocity)
{
  if (manipulator_.component.at(name).tool.id > 0)
  {
    //error
  }
  else
  {
    if (manipulator_.component.find(name) != manipulator_.component.end())
    {
      manipulator_.component.at(name).joint.velocity = joint_velocity;
    }
    else
    {
      //error
    }
  }
}

void RobotisManipulator::setJointAcceleration(Name name, double joint_acceleration)
{
  if (manipulator_.component.at(name).tool.id > 0)
  {
    //error
  }
  else
  {
    if (manipulator_.component.find(name) != manipulator_.component.end())
    {
      manipulator_.component.at(name).joint.acceleration = joint_acceleration;
    }
    else
    {
      //error
    }
  }
}

void RobotisManipulator::setAllAcutiveJointValue(std::vector<double> joint_value_vector)
{
  std::map<Name, Component>::iterator it;
  int8_t index = 0;

  for (it = manipulator_.component.begin(); it != manipulator_.component.end(); it++)
  {
    if (manipulator_.component.at(it->first).joint.id != -1)
    {
      manipulator_.component.at(it->first).joint.value = joint_value_vector.at(index);
    }
    index++;
  }
}

void RobotisManipulator::setAllAcutiveJointValue(std::vector<double> joint_value_vector, std::vector<double> joint_velocity_vector, std::vector<double> joint_acceleration_vector)
{
  std::map<Name, Component>::iterator it;
  int8_t index = 0;

  for (it = manipulator_.component.begin(); it != manipulator_.component.end(); it++)
  {
    if (manipulator_.component.at(it->first).joint.id != -1)
    {
      manipulator_.component.at(it->first).joint.value = joint_value_vector.at(index);
      manipulator_.component.at(it->first).joint.velocity = joint_velocity_vector.at(index);
      manipulator_.component.at(it->first).joint.acceleration = joint_acceleration_vector.at(index);
    }
    index++;
  }
}

void RobotisManipulator::setAllJointValue(std::vector<double> joint_value_vector)
{
  std::map<Name, Component>::iterator it;
  int8_t index = 0;

  for (it = manipulator_.component.begin(); it != manipulator_.component.end(); it++)
  {
    if (manipulator_.component.at(it->first).tool.id == -1)
    {
      manipulator_.component.at(it->first).joint.value = joint_value_vector.at(index);
    }
    index++;
  }
}

void RobotisManipulator::setAllJointValue(std::vector<double> joint_value_vector, std::vector<double> joint_velocity_vector, std::vector<double> joint_acceleration_vector)
{
  std::map<Name, Component>::iterator it;
  int8_t index = 0;

  for (it = manipulator_.component.begin(); it != manipulator_.component.end(); it++)
  {
    if (manipulator_.component.at(it->first).tool.id == -1)
    {
      manipulator_.component.at(it->first).joint.value = joint_value_vector.at(index);
      manipulator_.component.at(it->first).joint.velocity = joint_velocity_vector.at(index);
      manipulator_.component.at(it->first).joint.acceleration = joint_acceleration_vector.at(index);
    }
    index++;
  }
}

void RobotisManipulator::setJointActuatorValue(Name name, Actuator actuator_value)
{
  if (manipulator_.component.at(name).joint.id == -1)
  {
    //error not active joint
  }
  else
  {
    if (manipulator_.component.find(name) != manipulator_.component.end())
    {
      manipulator_.component.at(name).joint.value = manipulator_.component.at(name).joint.coefficient *  actuator_value.value;
      manipulator_.component.at(name).joint.velocity = manipulator_.component.at(name).joint.coefficient *  actuator_value.velocity;
      manipulator_.component.at(name).joint.acceleration = manipulator_.component.at(name).joint.coefficient *  actuator_value.acceleration;
    }
    else
    {
      //error
    }
  }

}

void RobotisManipulator::setAllJointActuatorValue(std::vector<Actuator> actuator_value_vector)
{
  std::map<Name, Component>::iterator it;
  int8_t index = 0;

  for (it = manipulator_.component.begin(); it != manipulator_.component.end(); it++)
  {
    if (manipulator_.component.at(it->first).joint.id != -1)
    {
      manipulator_.component.at(it->first).joint.value = manipulator_.component.at(it->first).joint.coefficient *  actuator_value_vector.at(index).value;
      manipulator_.component.at(it->first).joint.velocity = manipulator_.component.at(it->first).joint.coefficient *  actuator_value_vector.at(index).velocity;
      manipulator_.component.at(it->first).joint.acceleration = manipulator_.component.at(it->first).joint.coefficient *  actuator_value_vector.at(index).acceleration;
    }
    index++;
  }
}

void RobotisManipulator::setToolValue(Name name, double tool_value)
{
  if (manipulator_.component.at(name).tool.id > 0)
  {
    if (manipulator_.component.find(name) != manipulator_.component.end())
    {
      manipulator_.component.at(name).tool.value = tool_value;
    }
    else
    {
      //error
    }
  }
  else
  {
    //error
  }
}

void RobotisManipulator::setToolActuatorValue(Name name, double actuator_value)
{
  if (manipulator_.component.at(name).tool.id == -1)
  {
    //error not tool
  }
  else
  {
    if (manipulator_.component.find(name) != manipulator_.component.end())
    {
      manipulator_.component.at(name).tool.value = manipulator_.component.at(name).tool.coefficient * actuator_value;
    }
    else
    {
      //error
    }
  }
}


///////////////////////////////Get function//////////////////////////////////

int8_t RobotisManipulator::getDOF()
{
  return manipulator_.dof;
}

Name RobotisManipulator::getWorldName()
{
  return manipulator_.world.name;
}

Name RobotisManipulator::getWorldChildName()
{
  return manipulator_.world.child;
}

Pose RobotisManipulator::getWorldPose()
{
  return manipulator_.world.pose;
}

Eigen::Vector3d RobotisManipulator::getWorldPosition()
{
  return manipulator_.world.pose.position;
}

Eigen::Quaterniond RobotisManipulator::getWorldOrientation()
{
  return manipulator_.world.pose.orientation;
}

Dynamicpose RobotisManipulator::getWorldDynamicPose()
{
  return manipulator_.world.dynamic_pose;
}

int8_t RobotisManipulator::getComponentSize()
{
  return manipulator_.component.size();
}

std::map<Name, Component> RobotisManipulator::getAllComponent()
{
  return manipulator_.component;
}

std::map<Name, Component>::iterator RobotisManipulator::getIteratorBegin()
{
  return manipulator_.component.begin();
}

std::map<Name, Component>::iterator RobotisManipulator::getIteratorEnd()
{
  return manipulator_.component.end();;
}

Component RobotisManipulator::getComponent(Name name)
{
  return manipulator_.component.at(name);
}

Name RobotisManipulator::getComponentParentName(Name name)
{
  return manipulator_.component.at(name).parent;
}

std::vector<Name> RobotisManipulator::getComponentChildName(Name name)
{
  return manipulator_.component.at(name).child;
}

Pose RobotisManipulator::getComponentPoseToWorld(Name name)
{
  return manipulator_.component.at(name).pose_to_world;
}

Eigen::Vector3d RobotisManipulator::getComponentPositionToWorld(Name name)
{
  return manipulator_.component.at(name).pose_to_world.position;
}

Eigen::Quaterniond RobotisManipulator::getComponentOrientationToWorld(Name name)
{
  return manipulator_.component.at(name).pose_to_world.orientation;
}

Dynamicpose RobotisManipulator::getComponentDynamicPoseToWorld(Name name)
{
  return manipulator_.component.at(name).dynamic_pose;
}

Pose RobotisManipulator::getComponentRelativePoseToParent(Name name)
{
  return manipulator_.component.at(name).relative_to_parent;
}

Eigen::Vector3d RobotisManipulator::getComponentRelativePositionToParent(Name name)
{
  return manipulator_.component.at(name).relative_to_parent.position;
}

Eigen::Quaterniond RobotisManipulator::getComponentRelativeOrientationToParent(Name name)
{
  return manipulator_.component.at(name).relative_to_parent.orientation;
}

Joint RobotisManipulator::getComponentJoint(Name name)
{
  return manipulator_.component.at(name).joint;
}

int8_t RobotisManipulator::getJointId(Name name)
{
  return manipulator_.component.at(name).joint.id;
}

double RobotisManipulator::getJointCoefficient(Name name)
{
  return manipulator_.component.at(name).joint.coefficient;
}

Eigen::Vector3d RobotisManipulator::getJointAxis(Name name)
{
  return manipulator_.component.at(name).joint.axis;
}

double RobotisManipulator::getJointValue(Name name)
{

  return manipulator_.component.at(name).joint.value;
}

double RobotisManipulator::getJointVelocity(Name name)
{
  return manipulator_.component.at(name).joint.velocity;
}

double RobotisManipulator::getJointAcceleration(Name name)
{
  return manipulator_.component.at(name).joint.acceleration;
}

Actuator RobotisManipulator::getJointActuatorValue(Name name)
{
  Actuator result_value;

  result_value.value = manipulator_.component.at(name).joint.value / manipulator_.component.at(name).joint.coefficient;
  result_value.velocity = manipulator_.component.at(name).joint.velocity / manipulator_.component.at(name).joint.coefficient;
  result_value.acceleration = manipulator_.component.at(name).joint.acceleration / manipulator_.component.at(name).joint.coefficient;

  return result_value;
}

int8_t RobotisManipulator::getToolId(Name name)
{
  return manipulator_.component.at(name).tool.id;
}

double RobotisManipulator::getToolCoefficient(Name name)
{
  return manipulator_.component.at(name).tool.coefficient;
}

double RobotisManipulator::getToolValue(Name name)
{
  return manipulator_.component.at(name).tool.value;
}

double RobotisManipulator::getToolActuatorValue(Name name)
{
  return manipulator_.component.at(name).tool.value / manipulator_.component.at(name).tool.coefficient;
}

double RobotisManipulator::getComponentMass(Name name)
{
  return manipulator_.component.at(name).inertia.mass;
}

Eigen::Matrix3d RobotisManipulator::getComponentInertiaTensor(Name name)
{
  return manipulator_.component.at(name).inertia.inertia_tensor;
}

Eigen::Vector3d RobotisManipulator::getComponentCenterOfMass(Name name)
{
  return manipulator_.component.at(name).inertia.center_of_mass;
}

std::vector<double> RobotisManipulator::getAllJointValue()
{
  std::vector<double> result_vector;
  std::map<Name, Component>::iterator it;

  for (it = manipulator_.component.begin(); it != manipulator_.component.end(); it++)
  {
    if (manipulator_.component.at(it->first).tool.id == -1) // Check whether Tool or not
    {
      // This is not Tool -> This is Joint
      result_vector.push_back(manipulator_.component.at(it->first).joint.value);
    }
  }
  return result_vector;
}

std::vector<double> RobotisManipulator::getAllActiveJointValue()
{
  std::vector<double> result_vector;
  std::map<Name, Component>::iterator it;

  for (it = manipulator_.component.begin(); it != manipulator_.component.end(); it++)
  {
    if (manipulator_.component.at(it->first).joint.id != -1) // Check whether Active or Passive
    {
      // Active
      result_vector.push_back(manipulator_.component.at(it->first).joint.value);
    }
  }
  return result_vector;
}

void RobotisManipulator::getAllActiveJointValue(std::vector<double> *joint_value_vector, std::vector<double> *joint_velocity_vector, std::vector<double> *joint_accelerarion_vector)
{
  std::map<Name, Component>::iterator it;

  joint_value_vector->clear();
  joint_velocity_vector->clear();
  joint_accelerarion_vector->clear();

  for (it = manipulator_.component.begin(); it != manipulator_.component.end(); it++)
  {
    if (manipulator_.component.at(it->first).joint.id != -1) // Check whether Active or Passive
    {
      // Active
      joint_value_vector->push_back(manipulator_.component.at(it->first).joint.value);
      joint_velocity_vector->push_back(manipulator_.component.at(it->first).joint.velocity);
      joint_accelerarion_vector->push_back(manipulator_.component.at(it->first).joint.acceleration);
    }
  }
}

std::vector<Actuator> RobotisManipulator::getAllJointActuatorValue()
{
  std::map<Name, Component>::iterator it;
  Actuator result;
  std::vector<Actuator> result_vector;


  for (it = manipulator_.component.begin(); it != manipulator_.component.end(); it++)
  {
    if (manipulator_.component.at(it->first).joint.id != -1) // Check whether Active or Passive
    {
      // Active
      result.value = manipulator_.component.at(it->first).joint.value / manipulator_.component.at(it->first).joint.coefficient;
      result.velocity = manipulator_.component.at(it->first).joint.velocity / manipulator_.component.at(it->first).joint.coefficient;
      result.acceleration = manipulator_.component.at(it->first).joint.acceleration / manipulator_.component.at(it->first).joint.coefficient;

      result_vector.push_back(result);
    }
  }

  return result_vector;
}

std::vector<uint8_t> RobotisManipulator::getAllJointID()
{
  std::vector<uint8_t> joint_id;
  std::map<Name, Component>::iterator it;

  for (it = manipulator_.component.begin(); it != manipulator_.component.end(); it++)
  {
    if (manipulator_.component.at(it->first).tool.id == -1)
    {
      joint_id.push_back(manipulator_.component.at(it->first).joint.id);
    }
  }
  return joint_id;
}

std::vector<uint8_t> RobotisManipulator::getAllActiveJointID()
{
  std::vector<uint8_t> active_joint_id;
  std::map<Name, Component>::iterator it;

  for (it = manipulator_.component.begin(); it != manipulator_.component.end(); it++)
  {
    if (manipulator_.component.at(it->first).joint.id != -1)
    {
      active_joint_id.push_back(manipulator_.component.at(it->first).joint.id);
    }
  }
  return active_joint_id;
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

void RobotisManipulator::JointActuatorSetMode(Name actuator_name, uint8_t id_array, const void *arg)
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

  trajectory_.start_way_point_.reserve(manipulator_.dof);
  trajectory_.goal_way_point_.reserve(manipulator_.dof);
}


void RobotisManipulator::UpdatePresentWayPoint()
{
  //kinematics (position)
  kinematics_->updatePassiveJointValue(&trajectory_.manipulator_);
  kinematics_->forward(&trajectory_.manipulator_);

  //dynamics (velocity)
  std::map<Name, Component>::iterator it;
  Eigen::VectorXd joint_velocity(trajectory_.manipulator_.dof);
  Eigen::VectorXd pose_velocity(6);
  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d angular_velocity;

  int8_t index = 0;
  for (it = trajectory_.manipulator_.component.begin(); it != trajectory_.manipulator_.component.end(); it++)
  {
    if (trajectory_.manipulator_.component.at(it->first).joint.id != -1) // Check whether Active or Passive
    {
      // Active
      joint_velocity[index] = trajectory_.manipulator_.component.at(it->first).joint.velocity;
      index++;
    }
  }
  for (it = trajectory_.manipulator_.component.begin(); it != trajectory_.manipulator_.component.end(); it++)
  {
    if (trajectory_.manipulator_.component.at(it->first).tool.id != -1)
    {
      for(int index2 = 0; index2 < trajectory_.manipulator_.dof; index2++)
      {
        joint_velocity[index2] =trajectory_.manipulator_.component.at(it->first).joint.velocity;
      }
      pose_velocity = kinematics_->jacobian(&trajectory_.manipulator_, it->first)*joint_velocity;
      linear_velocity[0] = pose_velocity[0];
      linear_velocity[1] = pose_velocity[1];
      linear_velocity[2] = pose_velocity[2];
      angular_velocity[0] = pose_velocity[3];
      angular_velocity[1] = pose_velocity[4];
      angular_velocity[2] = pose_velocity[5];
      trajectory_.manipulator_.component.at(it->first).dynamic_pose.linear.velocity = linear_velocity;
      trajectory_.manipulator_.component.at(it->first).dynamic_pose.linear.velocity = angular_velocity;

      trajectory_.manipulator_.component.at(it->first).dynamic_pose.linear.accelation = Eigen::Vector3d::Zero();
      trajectory_.manipulator_.component.at(it->first).dynamic_pose.linear.accelation = Eigen::Vector3d::Zero();
    }
  }
}

void RobotisManipulator::setPresentJointWayPoint(std::vector<WayPoint> joint_value_vector)
{
  std::map<Name, Component>::iterator it;
  int8_t index = 0;

  for (it = trajectory_.manipulator_.component.begin(); it != trajectory_.manipulator_.component.end(); it++)
  {
    if (trajectory_.manipulator_.component.at(it->first).joint.id != -1)
    {
      trajectory_.manipulator_.component.at(it->first).joint.value = joint_value_vector.at(index).value;
      trajectory_.manipulator_.component.at(it->first).joint.velocity = joint_value_vector.at(index).velocity;
      trajectory_.manipulator_.component.at(it->first).joint.acceleration = joint_value_vector.at(index).acceleration;
    }
    index++;
  }
}

void RobotisManipulator::setPresentTaskWayPoint(Name tool_name, std::vector<WayPoint> tool_value_vector)
{
  for(int pos_count = 0; pos_count < 3; pos_count++)
  {
    trajectory_.manipulator_.component.at(tool_name).pose_to_world.position[pos_count] = tool_value_vector.at(pos_count).value;
    trajectory_.manipulator_.component.at(tool_name).dynamic_pose.linear.velocity[pos_count] = tool_value_vector.at(pos_count).velocity;
    trajectory_.manipulator_.component.at(tool_name).dynamic_pose.linear.accelation[pos_count] = tool_value_vector.at(pos_count).acceleration;
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
  trajectory_.manipulator_.component.at(tool_name).pose_to_world.orientation = orientation_Quat;
  trajectory_.manipulator_.component.at(tool_name).dynamic_pose.angular.velocity = orientation_velocity_vector;
  trajectory_.manipulator_.component.at(tool_name).dynamic_pose.angular.accelation = orientation_acceleration_vector;
}


std::vector<WayPoint> RobotisManipulator::getPresentJointWayPoint()
{
  std::map<Name, Component>::iterator it;
  WayPoint result;
  std::vector<WayPoint> result_vector;

  for (it = trajectory_.manipulator_.component.begin(); it != trajectory_.manipulator_.component.end(); it++)
  {
    if (trajectory_.manipulator_.component.at(it->first).joint.id != -1) // Check whether Active or Passive
    {
      // Active
      result.value = trajectory_.manipulator_.component.at(it->first).joint.value;
      result.velocity = trajectory_.manipulator_.component.at(it->first).joint.velocity;
      result.acceleration = trajectory_.manipulator_.component.at(it->first).joint.acceleration;

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
    result.value = trajectory_.manipulator_.component.at(tool_name).pose_to_world.position[pos_count];
    result.velocity = trajectory_.manipulator_.component.at(tool_name).dynamic_pose.linear.velocity[pos_count];
    result.acceleration = trajectory_.manipulator_.component.at(tool_name).dynamic_pose.linear.accelation[pos_count];

    result_vector.push_back(result);
  }

  Eigen::Vector3d orientation_vector =  RM_MATH::convertQuaternionToRPY(trajectory_.manipulator_.component.at(tool_name).pose_to_world.orientation);
  for(int ori_count = 0; ori_count < 3; ori_count++)
  {
    result.value = orientation_vector[ori_count];
    result.velocity = trajectory_.manipulator_.component.at(tool_name).dynamic_pose.angular.velocity[ori_count];
    result.acceleration = trajectory_.manipulator_.component.at(tool_name).dynamic_pose.angular.accelation[ori_count];

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
  for (uint8_t index = 0; index < trajectory_.manipulator_.dof; index++)
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
  for (uint8_t index = 0; index < trajectory_.manipulator_.dof; index++)
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
  for (it = trajectory_.manipulator_.component.begin(); it != trajectory_.manipulator_.component.end(); it++)
  {
    if (trajectory_.manipulator_.component.at(it->first).joint.id != -1) // Check whether Active or Passive
    {
      // Active
      result.at(index_it).value = result_joint_actuator_value.at(index_it).value / trajectory_.manipulator_.component.at(it->first).joint.coefficient;
      result.at(index_it).velocity = result_joint_actuator_value.at(index_it).velocity / trajectory_.manipulator_.component.at(it->first).joint.coefficient;
      result.at(index_it).acceleration = result_joint_actuator_value.at(index_it).acceleration / trajectory_.manipulator_.component.at(it->first).joint.coefficient;
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
    sendAllJointActuatorValue(getAllActiveJointID(), joint_goal_way_point);
    /////////////////////////////////////////////////////////////////////
  }

}

double RobotisManipulator::toolMove(Name tool_name, double tool_value)
{
  setToolValue(tool_name, tool_value);
  sendToolActuatorValue(getToolId(tool_name), getToolActuatorValue(tool_name));
}








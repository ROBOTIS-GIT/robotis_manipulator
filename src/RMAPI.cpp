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

#include "robotis_manipulator/RMAPI.h"

using namespace OPEN_MANIPULATOR;

////////////////////////////////////////////////////////////////////////////
////////////////////////////////Basic Function//////////////////////////////
////////////////////////////////////////////////////////////////////////////

void Manager::addWorld(RM_MANAGER::Manipulator *manipulator,
                       Name world_name,
                       Name child_name,
                       Vector3f world_position,
                       Matrix3f world_orientation)
{
  manipulator->addWorld(world_name, child_name, world_position, world_orientation);
}

void Manager::addComponent(RM_MANAGER::Manipulator *manipulator,
                           Name my_name,
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
  manipulator->addComponent(my_name, parent_name, child_name, relative_position, relative_orientation, axis_of_rotation, actuator_id, coefficient, mass, inertia_tensor, center_of_mass);
}

void Manager::addTool(RM_MANAGER::Manipulator *manipulator,
                      Name my_name,
                      Name parent_name,
                      Vector3f relative_position,
                      Matrix3f relative_orientation,
                      int8_t tool_id,
                      double coefficient,
                      double mass,
                      Matrix3f inertia_tensor,
                      Vector3f center_of_mass)
{
  manipulator->addTool(my_name, parent_name, relative_position, relative_orientation, tool_id, coefficient, mass, inertia_tensor, center_of_mass);
}

void Manager::addComponentChild(RM_MANAGER::Manipulator *manipulator, Name my_name, Name child_name)
{
  manipulator->addComponentChild(my_name, child_name);
}

void Manager::checkManipulatorSetting(RM_MANAGER::Manipulator *manipulator)
{
  manipulator->checkManipulatorSetting();
}

void Manager::setWorldPose(RM_MANAGER::Manipulator *manipulator, Pose world_pose)
{
  manipulator->setWorldPose(world_pose);
}

void Manager::setWorldPosition(RM_MANAGER::Manipulator *manipulator, Vector3f world_position)
{
  manipulator->setWorldPosition(world_position);
}

void Manager::setWorldOrientation(RM_MANAGER::Manipulator *manipulator, Matrix3f world_orientation)
{
  manipulator->setWorldOrientation(world_orientation);
}

void Manager::setWorldState(RM_MANAGER::Manipulator *manipulator, State world_state)
{
  manipulator->setWorldState(world_state);
}

void Manager::setWorldVelocity(RM_MANAGER::Manipulator *manipulator, VectorXf world_velocity)
{
  manipulator->setWorldVelocity(world_velocity);
}

void Manager::setWorldAcceleration(RM_MANAGER::Manipulator *manipulator, VectorXf world_acceleration)
{
  manipulator->setWorldAcceleration(world_acceleration);
}

void Manager::setComponent(RM_MANAGER::Manipulator *manipulator, Name name, Component component)
{
  manipulator->setComponent(name, component);
}

void Manager::setComponentPoseToWorld(RM_MANAGER::Manipulator *manipulator, Name name, Pose pose_to_world)
{
  manipulator->setComponentPoseToWorld(name, pose_to_world);
}

void Manager::setComponentPositionToWorld(RM_MANAGER::Manipulator *manipulator, Name name, Vector3f position_to_world)
{
  manipulator->setComponentPositionToWorld(name, position_to_world);
}

void Manager::setComponentOrientationToWorld(RM_MANAGER::Manipulator *manipulator, Name name, Matrix3f orientation_to_wolrd)
{
  manipulator->setComponentOrientationToWorld(name, orientation_to_wolrd);
}

void Manager::setComponentStateToWorld(RM_MANAGER::Manipulator *manipulator, Name name, State state_to_world)
{
  manipulator->setComponentStateToWorld(name, state_to_world);
}

void Manager::setComponentVelocityToWorld(RM_MANAGER::Manipulator *manipulator, Name name, VectorXf velocity)
{
  manipulator->setComponentVelocityToWorld(name, velocity);
}

void Manager::setComponentAccelerationToWorld(RM_MANAGER::Manipulator *manipulator, Name name, VectorXf accelaration)
{
  manipulator->setComponentAccelerationToWorld(name, accelaration);
}

void Manager::setComponentJointAngle(RM_MANAGER::Manipulator *manipulator, Name name, double angle)
{
  manipulator->setComponentJointAngle(name, angle);
}

void Manager::setComponentJointVelocity(RM_MANAGER::Manipulator *manipulator, Name name, double angular_velocity)
{
  manipulator->setComponentJointVelocity(name, angular_velocity);
}

void Manager::setComponentJointAcceleration(RM_MANAGER::Manipulator *manipulator, Name name, double angular_acceleration)
{
  manipulator->setComponentJointAcceleration(name, angular_acceleration);
}

void Manager::setComponentToolOnOff(RM_MANAGER::Manipulator *manipulator, Name name, bool on_off)
{
  manipulator->setComponentToolOnOff(name, on_off);
}

void Manager::setComponentToolValue(RM_MANAGER::Manipulator *manipulator, Name name, double actuator_value)
{
  manipulator->setComponentToolValue(name, actuator_value);
}

void Manager::setAllActiveJointAngle(RM_MANAGER::Manipulator *manipulator, std::vector<double> angle_vector)
{
  manipulator->setAllActiveJointAngle(angle_vector);
}

int8_t Manager::getDOF(RM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getDOF();
}

int8_t Manager::getComponentSize(RM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getComponentSize();
}

Name Manager::getWorldName(RM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldName();
}

Name Manager::getWorldChildName(RM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldChildName();
}

Pose Manager::getWorldPose(RM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldPose();
}

Vector3f Manager::getWorldPosition(RM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldPosition();
}

Matrix3f Manager::getWorldOrientation(RM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldOrientation();
}

State Manager::getWorldState(RM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldState();
}

VectorXf Manager::getWorldVelocity(RM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldVelocity();
}

VectorXf Manager::getWorldAcceleration(RM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getWorldAcceleration();
}

std::map<Name, Component> Manager::getAllComponent(RM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getAllComponent();
}

std::map<Name, Component>::iterator Manager::getIteratorBegin(RM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getIteratorBegin();
}

std::map<Name, Component>::iterator Manager::getIteratorEnd(RM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getIteratorEnd();
}

Component Manager::getComponent(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponent(name);
}

Name Manager::getComponentParentName(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentParentName(name);
}

std::vector<Name> Manager::getComponentChildName(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentChildName(name);
}

Pose Manager::getComponentPoseToWorld(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentPoseToWorld(name);
}

Vector3f Manager::getComponentPositionToWorld(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentPositionToWorld(name);
}

Matrix3f Manager::getComponentOrientationToWorld(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentOrientationToWorld(name);
}

State Manager::getComponentStateToWorld(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentStateToWorld(name);
}

VectorXf Manager::getComponentVelocityToWorld(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentVelocityToWorld(name);
}

VectorXf Manager::getComponentAccelerationToWorld(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentAccelerationToWorld(name);
}

Pose Manager::getComponentRelativePoseToParent(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentRelativePoseToParent(name);
}

Vector3f Manager::getComponentRelativePositionToParent(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentRelativePositionToParent(name);
}

Matrix3f Manager::getComponentRelativeOrientationToParent(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentRelativeOrientationToParent(name);
}

Joint Manager::getComponentJoint(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJoint(name);
}

int8_t Manager::getComponentJointId(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointId(name);
}

double Manager::getComponentJointCoefficient(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointCoefficient(name);
}

Vector3f Manager::getComponentJointAxis(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointAxis(name);
}

double Manager::getComponentJointAngle(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointAngle(name);
}

std::vector<double> Manager::getAllJointAngle(RM_MANAGER::Manipulator *manipulator)
{
  std::vector<double> joint_angle = manipulator->getAllJointAngle();

  return joint_angle;
}

std::vector<double> Manager::getAllActiveJointAngle(RM_MANAGER::Manipulator *manipulator)
{
  std::vector<double> joint_angle = manipulator->getAllActiveJointAngle();
  
  return joint_angle;
}

double Manager::getComponentJointVelocity(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointVelocity(name);
}

double Manager::getComponentJointAcceleration(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentJointAcceleration(name);
}

Tool Manager::getComponentTool(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentTool(name);
}

int8_t Manager::getComponentToolId(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentToolId(name);
}

double Manager::getComponentToolCoefficient(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentToolCoefficient(name);
}

bool Manager::getComponentToolOnOff(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentToolOnOff(name);
}

double Manager::getComponentToolValue(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentToolValue(name);
}

double Manager::getComponentMass(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentMass(name);
}

Matrix3f Manager::getComponentInertiaTensor(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentInertiaTensor(name);
}

Vector3f Manager::getComponentCenterOfMass(RM_MANAGER::Manipulator *manipulator, Name name)
{
  return manipulator->getComponentCenterOfMass(name);
}

std::vector<uint8_t> Manager::getAllActiveJointID(RM_MANAGER::Manipulator *manipulator)
{
  return manipulator->getAllActiveJointID();
}

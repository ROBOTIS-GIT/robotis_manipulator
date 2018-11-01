#include "robotis_manipulator/robotis_manipulator_common.h"

using namespace ROBOTIS_MANIPULATOR;

/////////////////////Manipulator class//////////////////////

///////////////////////////////add function//////////////////////////////////
void Manipulator::addWorld(Name world_name,
                           Name child_name,
                           Eigen::Vector3d world_position,
                           Eigen::Quaterniond world_orientation)
{
  world_.name = world_name;
  world_.child = child_name;
  world_.pose.position = world_position;
  world_.pose.orientation = world_orientation;
  world_.dynamic_pose.linear.velocity = Eigen::Vector3d::Zero(3);
  world_.dynamic_pose.linear.accelation = Eigen::Vector3d::Zero(3);
  world_.dynamic_pose.angular.velocity = Eigen::Vector3d::Zero(3);
  world_.dynamic_pose.angular.accelation = Eigen::Vector3d::Zero(3);
}

void Manipulator::addComponent(Name my_name,
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
    dof_++;

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

  component_.insert(std::make_pair(my_name, temp_component));
}

void Manipulator::addComponentChild(Name my_name, Name child_name)
{
  component_.at(my_name).child.push_back(child_name);
}

void Manipulator::addTool(Name my_name,
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

void Manipulator::setWorldOrientation(Eigen::Quaterniond world_orientation)
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

void Manipulator::setWorldLinearAcceleration(Eigen::Vector3d world_linear_acceleration)
{
  world_.dynamic_pose.linear.accelation = world_linear_acceleration;
}

void Manipulator::setWorldAngularAcceleration(Eigen::Vector3d world_angular_acceleration)
{
  world_.dynamic_pose.angular.accelation = world_angular_acceleration;
}


void Manipulator::setComponentPoseToWorld(Name name, Pose pose_to_world)
{
  if (component_.find(name) != component_.end())
  {
    component_.at(name).pose_to_world = pose_to_world;
  }
  else
  {
    //error
  }
}

void Manipulator::setComponentPositionToWorld(Name name, Eigen::Vector3d position_to_world)
{
  if (component_.find(name) != component_.end())
  {
    component_.at(name).pose_to_world.position = position_to_world;
  }
  else
  {
    //error
  }
}

void Manipulator::setComponentOrientationToWorld(Name name, Eigen::Quaterniond orientation_to_wolrd)
{
  if (component_.find(name) != component_.end())
  {
    component_.at(name).pose_to_world.orientation = orientation_to_wolrd;
  }
  else
  {
    //error
  }
}

void Manipulator::setComponentDynamicPoseToWorld(Name name, Dynamicpose dynamic_pose)
{
  if (component_.find(name) != component_.end())
  {
    component_.at(name).dynamic_pose = dynamic_pose;
  }
  else
  {
    //error
  }
}

void Manipulator::setJointValue(Name name, double joint_value)
{
  if (component_.at(name).tool.id > 0)
  {
    //error
  }
  else
  {
    if (component_.find(name) != component_.end())
    {
      component_.at(name).joint.value = joint_value;
    }
    else
    {
      //error
    }
  }
}

void Manipulator::setJointVelocity(Name name, double joint_velocity)
{
  if (component_.at(name).tool.id > 0)
  {
    //error
  }
  else
  {
    if (component_.find(name) != component_.end())
    {
      component_.at(name).joint.velocity = joint_velocity;
    }
    else
    {
      //error
    }
  }
}

void Manipulator::setJointAcceleration(Name name, double joint_acceleration)
{
  if (component_.at(name).tool.id > 0)
  {
    //error
  }
  else
  {
    if (component_.find(name) != component_.end())
    {
      component_.at(name).joint.acceleration = joint_acceleration;
    }
    else
    {
      //error
    }
  }
}

void Manipulator::setAllActiveJointValue(std::vector<double> joint_value_vector)
{
  std::map<Name, Component>::iterator it;
  int8_t index = 0;

  for (it = component_.begin(); it != component_.end(); it++)
  {
    if (component_.at(it->first).joint.id != -1)
    {
      component_.at(it->first).joint.value = joint_value_vector.at(index);
    }
    index++;
  }
}

void Manipulator::setAllActiveJointValue(std::vector<double> joint_value_vector, std::vector<double> joint_velocity_vector, std::vector<double> joint_acceleration_vector)
{
  std::map<Name, Component>::iterator it;
  int8_t index = 0;

  for (it = component_.begin(); it != component_.end(); it++)
  {
    if (component_.at(it->first).joint.id != -1)
    {
      component_.at(it->first).joint.value = joint_value_vector.at(index);
      component_.at(it->first).joint.velocity = joint_velocity_vector.at(index);
      component_.at(it->first).joint.acceleration = joint_acceleration_vector.at(index);
    }
    index++;
  }
}

void Manipulator::setAllJointValue(std::vector<double> joint_value_vector)
{
  std::map<Name, Component>::iterator it;
  int8_t index = 0;

  for (it = component_.begin(); it != component_.end(); it++)
  {
    if (component_.at(it->first).tool.id == -1)
    {
      component_.at(it->first).joint.value = joint_value_vector.at(index);
    }
    index++;
  }
}

void Manipulator::setAllJointValue(std::vector<double> joint_value_vector, std::vector<double> joint_velocity_vector, std::vector<double> joint_acceleration_vector)
{
  std::map<Name, Component>::iterator it;
  int8_t index = 0;

  for (it = component_.begin(); it != component_.end(); it++)
  {
    if (component_.at(it->first).tool.id == -1)
    {
      component_.at(it->first).joint.value = joint_value_vector.at(index);
      component_.at(it->first).joint.velocity = joint_velocity_vector.at(index);
      component_.at(it->first).joint.acceleration = joint_acceleration_vector.at(index);
    }
    index++;
  }
}

void Manipulator::setJointActuatorValue(Name name, Actuator actuator_value)
{
  if (component_.at(name).joint.id == -1)
  {
    //error not active joint
  }
  else
  {
    if (component_.find(name) != component_.end())
    {
      component_.at(name).joint.value = component_.at(name).joint.coefficient *  actuator_value.value;
      component_.at(name).joint.velocity = component_.at(name).joint.coefficient *  actuator_value.velocity;
      component_.at(name).joint.acceleration = component_.at(name).joint.coefficient *  actuator_value.acceleration;
    }
    else
    {
      //error
    }
  }

}

void Manipulator::setAllJointActuatorValue(std::vector<Actuator> actuator_value_vector)
{
  std::map<Name, Component>::iterator it;
  int8_t index = 0;

  for (it = component_.begin(); it != component_.end(); it++)
  {
    if (component_.at(it->first).joint.id != -1)
    {
      component_.at(it->first).joint.value = component_.at(it->first).joint.coefficient *  actuator_value_vector.at(index).value;
      component_.at(it->first).joint.velocity = component_.at(it->first).joint.coefficient *  actuator_value_vector.at(index).velocity;
      component_.at(it->first).joint.acceleration = component_.at(it->first).joint.coefficient *  actuator_value_vector.at(index).acceleration;
    }
    index++;
  }
}

void Manipulator::setToolValue(Name name, double tool_value)
{
  if (component_.at(name).tool.id > 0)
  {
    if (component_.find(name) != component_.end())
    {
      component_.at(name).tool.value = tool_value;
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

void Manipulator::setToolActuatorValue(Name name, double actuator_value)
{
  if (component_.at(name).tool.id == -1)
  {
    //error not tool
  }
  else
  {
    if (component_.find(name) != component_.end())
    {
      component_.at(name).tool.value = component_.at(name).tool.coefficient * actuator_value;
    }
    else
    {
      //error
    }
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

Eigen::Quaterniond Manipulator::getWorldOrientation()
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

Name Manipulator::getComponentParentName(Name name)
{
  return component_.at(name).parent;
}

std::vector<Name> Manipulator::getComponentChildName(Name name)
{
  return component_.at(name).child;
}

Pose Manipulator::getComponentPoseToWorld(Name name)
{
  return component_.at(name).pose_to_world;
}

Eigen::Vector3d Manipulator::getComponentPositionToWorld(Name name)
{
  return component_.at(name).pose_to_world.position;
}

Eigen::Quaterniond Manipulator::getComponentOrientationToWorld(Name name)
{
  return component_.at(name).pose_to_world.orientation;
}

Dynamicpose Manipulator::getComponentDynamicPoseToWorld(Name name)
{
  return component_.at(name).dynamic_pose;
}

Pose Manipulator::getComponentRelativePoseToParent(Name name)
{
  return component_.at(name).relative_to_parent;
}

Eigen::Vector3d Manipulator::getComponentRelativePositionToParent(Name name)
{
  return component_.at(name).relative_to_parent.position;
}

Eigen::Quaterniond Manipulator::getComponentRelativeOrientationToParent(Name name)
{
  return component_.at(name).relative_to_parent.orientation;
}

Joint Manipulator::getComponentJoint(Name name)
{
  return component_.at(name).joint;
}

int8_t Manipulator::getJointId(Name name)
{
  return component_.at(name).joint.id;
}

double Manipulator::getJointCoefficient(Name name)
{
  return component_.at(name).joint.coefficient;
}

Eigen::Vector3d Manipulator::getJointAxis(Name name)
{
  return component_.at(name).joint.axis;
}

double Manipulator::getJointValue(Name name)
{

  return component_.at(name).joint.value;
}

double Manipulator::getJointVelocity(Name name)
{
  return component_.at(name).joint.velocity;
}

double Manipulator::getJointAcceleration(Name name)
{
  return component_.at(name).joint.acceleration;
}

Actuator Manipulator::getJointActuatorValue(Name name)
{
  Actuator result_value;

  result_value.value = component_.at(name).joint.value / component_.at(name).joint.coefficient;
  result_value.velocity = component_.at(name).joint.velocity / component_.at(name).joint.coefficient;
  result_value.acceleration = component_.at(name).joint.acceleration / component_.at(name).joint.coefficient;

  return result_value;
}

int8_t Manipulator::getToolId(Name name)
{
  return component_.at(name).tool.id;
}

double Manipulator::getToolCoefficient(Name name)
{
  return component_.at(name).tool.coefficient;
}

double Manipulator::getToolValue(Name name)
{
  return component_.at(name).tool.value;
}

double Manipulator::getToolActuatorValue(Name name)
{
  return component_.at(name).tool.value / component_.at(name).tool.coefficient;
}

double Manipulator::getComponentMass(Name name)
{
  return component_.at(name).inertia.mass;
}

Eigen::Matrix3d Manipulator::getComponentInertiaTensor(Name name)
{
  return component_.at(name).inertia.inertia_tensor;
}

Eigen::Vector3d Manipulator::getComponentCenterOfMass(Name name)
{
  return component_.at(name).inertia.center_of_mass;
}

std::vector<double> Manipulator::getAllJointValue()
{
  std::vector<double> result_vector;
  std::map<Name, Component>::iterator it;

  for (it = component_.begin(); it != component_.end(); it++)
  {
    if (component_.at(it->first).tool.id == -1) // Check whether Tool or not
    {
      // This is not Tool -> This is Joint
      result_vector.push_back(component_.at(it->first).joint.value);
    }
  }
  return result_vector;
}

std::vector<double> Manipulator::getAllActiveJointValue()
{
  std::vector<double> result_vector;
  std::map<Name, Component>::iterator it;

  for (it = component_.begin(); it != component_.end(); it++)
  {
    if (component_.at(it->first).joint.id != -1) // Check whether Active or Passive
    {
      // Active
      result_vector.push_back(component_.at(it->first).joint.value);
    }
  }
  return result_vector;
}

void Manipulator::getAllActiveJointValue(std::vector<double> *joint_value_vector, std::vector<double> *joint_velocity_vector, std::vector<double> *joint_accelerarion_vector)
{
  std::map<Name, Component>::iterator it;

  joint_value_vector->clear();
  joint_velocity_vector->clear();
  joint_accelerarion_vector->clear();

  for (it = component_.begin(); it != component_.end(); it++)
  {
    if (component_.at(it->first).joint.id != -1) // Check whether Active or Passive
    {
      // Active
      joint_value_vector->push_back(component_.at(it->first).joint.value);
      joint_velocity_vector->push_back(component_.at(it->first).joint.velocity);
      joint_accelerarion_vector->push_back(component_.at(it->first).joint.acceleration);
    }
  }
}

std::vector<Actuator> Manipulator::getAllJointActuatorValue()
{
  std::map<Name, Component>::iterator it;
  Actuator result;
  std::vector<Actuator> result_vector;


  for (it = component_.begin(); it != component_.end(); it++)
  {
    if (component_.at(it->first).joint.id != -1) // Check whether Active or Passive
    {
      // Active
      result.value = component_.at(it->first).joint.value / component_.at(it->first).joint.coefficient;
      result.velocity = component_.at(it->first).joint.velocity / component_.at(it->first).joint.coefficient;
      result.acceleration = component_.at(it->first).joint.acceleration / component_.at(it->first).joint.coefficient;

      result_vector.push_back(result);
    }
  }

  return result_vector;
}

std::vector<uint8_t> Manipulator::getAllJointID()
{
  std::vector<uint8_t> joint_id;
  std::map<Name, Component>::iterator it;

  for (it = component_.begin(); it != component_.end(); it++)
  {
    if (component_.at(it->first).tool.id == -1)
    {
      joint_id.push_back(component_.at(it->first).joint.id);
    }
  }
  return joint_id;
}

std::vector<uint8_t> Manipulator::getAllActiveJointID()
{
  std::vector<uint8_t> active_joint_id;
  std::map<Name, Component>::iterator it;

  for (it = component_.begin(); it != component_.end(); it++)
  {
    if (component_.at(it->first).joint.id != -1)
    {
      active_joint_id.push_back(component_.at(it->first).joint.id);
    }
  }
  return active_joint_id;
}











////////////////////////////////////////////////////////////

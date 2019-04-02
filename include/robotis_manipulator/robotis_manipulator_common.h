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

/**
 * @file robotis_manipulator_common.h
 * @brief
 * @details
 */

#ifndef ROBOTIS_MANIPULATOR_COMMON_H
#define ROBOTIS_MANIPULATOR_COMMON_H

#include <unistd.h>
#if defined(__OPENCR__)
  #include <Eigen.h>  // Calls main Eigen matrix class library
  #include <Eigen/LU> // Calls inverse, determinant, LU decomp., etc.
  #include <WString.h>
#else
  #include <eigen3/Eigen/Eigen>
  #include <eigen3/Eigen/LU>
#endif
#include <math.h>
#include <vector>
#include <map>
#include "robotis_manipulator_math.h"
#include "robotis_manipulator_log.h"

namespace robotis_manipulator
{

typedef STRING Name;

/*****************************************************************************
** Value Set
*****************************************************************************/
typedef struct KinematicPose
{
  Eigen::Vector3d position;
  Eigen::Matrix3d orientation;
} KinematicPose;

typedef struct Dynamicvector
{
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
} Dynamicvector;

typedef struct DynamicPose
{
  Dynamicvector linear;
  Dynamicvector angular;
} DynamicPose;

typedef struct Inertia
{
  double mass;
  Eigen::Matrix3d inertia_tensor;
  Eigen::Vector3d center_of_mass;
} Inertia;

typedef struct Limit
{
  double maximum;
  double minimum;
} Limit;


/*****************************************************************************
** Time Set
*****************************************************************************/
typedef struct Time
{
  double total_move_time;
  double present_time;
  double start_time;
} Time;


/*****************************************************************************
** Trajectory Set
*****************************************************************************/
typedef enum TrajectoryType
{
  NONE = 0,
  JOINT_TRAJECTORY,
  TASK_TRAJECTORY,
  CUSTOM_JOINT_TRAJECTORY,
  CUSTOM_TASK_TRAJECTORY
} TrajectoryType;

typedef struct Point
{
  double position;
  double velocity;
  double acceleration;
  double effort;
} Point, ActuatorValue, JointValue, ToolValue;

typedef std::vector<JointValue> JointWaypoint;

typedef struct Pose
{
  KinematicPose kinematic;
  DynamicPose dynamic;
} TaskWaypoint, Pose;


/*****************************************************************************
** Component Set
*****************************************************************************/
typedef enum ComponentType
{
  PASSIVE_JOINT_COMPONENT = 0,
  ACTIVE_JOINT_COMPONENT,
  TOOL_COMPONENT
} ComponentType;

typedef struct ChainingName
{
  Name parent;
  std::vector<Name> child;
} ChainingName;

typedef struct Relative
{
  KinematicPose pose_from_parent;
  Inertia inertia;
} Relative;

typedef struct JointConstant
{
  int8_t id;
  Eigen::Vector3d axis;
  double coefficient;       // joint angle over actuator angle
  Limit position_limit;
} JointConstant;

typedef struct World
{
  Name name;
  Name child;
  Pose pose;
} World;

typedef struct Component
{
  //constant
  ChainingName name;
  ComponentType component_type;
  Relative relative;
  JointConstant joint_constant;

  //variable
  Pose pose_from_world;
  JointValue joint_value;

  //Actuator
  Name actuator_name;
} Component;


/*****************************************************************************
** Manipulator Class
*****************************************************************************/
/**
 * @brief The Manipulator class
 */
class Manipulator
{
private:
  int8_t dof_;
  World world_;
  std::map<Name, Component> component_;

public:
  Manipulator();
  ~Manipulator() {}

  /*****************************************************************************
  ** Add Function
  *****************************************************************************/
  /**
   * @brief addWorld
   * @param world_name
   * @param child_name
   * @param world_position
   * @param world_orientation
   */
  void addWorld(Name world_name,
                Name child_name,
                Eigen::Vector3d world_position = Eigen::Vector3d::Zero(),
                Eigen::Matrix3d world_orientation = Eigen::Matrix3d::Identity());
  /**
   * @brief addJoint
   * @param my_name
   * @param parent_name
   * @param child_name
   * @param relative_position
   * @param relative_orientation
   * @param axis_of_rotation
   * @param joint_actuator_id
   * @param max_position_limit
   * @param min_position_limit
   * @param coefficient
   * @param mass
   * @param inertia_tensor
   * @param center_of_mass
   */
  void addJoint(Name my_name,
                Name parent_name,
                Name child_name,
                Eigen::Vector3d relative_position,
                Eigen::Matrix3d relative_orientation,
                Eigen::Vector3d axis_of_rotation = Eigen::Vector3d::Zero(),
                int8_t joint_actuator_id = -1, 
                double max_position_limit = M_PI, 
                double min_position_limit = -M_PI,
                double coefficient = 1.0,
                double mass = 0.0,
                Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Identity(),
                Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero());
  /**
   * @brief addTool
   * @param my_name
   * @param parent_name
   * @param relative_position
   * @param relative_orientation
   * @param tool_id
   * @param max_position_limit
   * @param min_position_limit
   * @param coefficient
   * @param mass
   * @param inertia_tensor
   * @param center_of_mass
   */
  void addTool(Name my_name,
               Name parent_name,
               Eigen::Vector3d relative_position,
               Eigen::Matrix3d relative_orientation,
               int8_t tool_id = -1, 
               double max_position_limit = M_PI, 
               double min_position_limit = -M_PI,
               double coefficient = 1.0,
               double mass = 0.0,
               Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Identity(),
               Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero());
  /**
   * @brief addComponentChild
   * @param my_name
   * @param child_name
   */
  void addComponentChild(Name my_name, Name child_name);
  /**
   * @brief printManipulatorSetting
   */
  void printManipulatorSetting();


  /*****************************************************************************
  ** Set Function
  *****************************************************************************/
  /**
   * @brief setWorldPose
   * @param world_pose
   */
  void setWorldPose(Pose world_pose);
  /**
   * @brief setWorldKinematicPose
   * @param world_kinematic_pose
   */
  void setWorldKinematicPose(KinematicPose world_kinematic_pose);
  /**
   * @brief setWorldPosition
   * @param world_position
   */
  void setWorldPosition(Eigen::Vector3d world_position);
  /**
   * @brief setWorldOrientation
   * @param world_orientation
   */
  void setWorldOrientation(Eigen::Matrix3d world_orientation);
  /**
   * @brief setWorldDynamicPose
   * @param world_dynamic_pose
   */
  void setWorldDynamicPose(DynamicPose world_dynamic_pose);
  /**
   * @brief setWorldLinearVelocity
   * @param world_linear_velocity
   */
  void setWorldLinearVelocity(Eigen::Vector3d world_linear_velocity);
  /**
   * @brief setWorldAngularVelocity
   * @param world_angular_velocity
   */
  void setWorldAngularVelocity(Eigen::Vector3d world_angular_velocity);
  /**
   * @brief setWorldLinearAcceleration
   * @param world_linear_acceleration
   */
  void setWorldLinearAcceleration(Eigen::Vector3d world_linear_acceleration);
  /**
   * @brief setWorldAngularAcceleration
   * @param world_angular_acceleration
   */
  void setWorldAngularAcceleration(Eigen::Vector3d world_angular_acceleration);
  /**
   * @brief setComponent
   * @param component_name
   * @param component
   */
  void setComponent(Name component_name, Component component);
  /**
   * @brief setComponentActuatorName
   * @param component_name
   * @param actuator_name
   */
  void setComponentActuatorName(Name component_name, Name actuator_name);
  /**
   * @brief setComponentPoseFromWorld
   * @param component_name
   * @param pose_to_world
   */
  void setComponentPoseFromWorld(Name component_name, Pose pose_to_world);
  /**
   * @brief setComponentKinematicPoseFromWorld
   * @param component_name
   * @param pose_to_world
   */
  void setComponentKinematicPoseFromWorld(Name component_name, KinematicPose pose_to_world);
  /**
   * @brief setComponentPositionFromWorld
   * @param component_name
   * @param position_to_world
   */
  void setComponentPositionFromWorld(Name component_name, Eigen::Vector3d position_to_world);
  /**
   * @brief setComponentOrientationFromWorld
   * @param component_name
   * @param orientation_to_wolrd
   */
  void setComponentOrientationFromWorld(Name component_name, Eigen::Matrix3d orientation_to_wolrd);
  /**
   * @brief setComponentDynamicPoseFromWorld
   * @param component_name
   * @param dynamic_pose
   */
  void setComponentDynamicPoseFromWorld(Name component_name, DynamicPose dynamic_pose);
  /**
   * @brief setJointPosition
   * @param name
   * @param position
   */
  void setJointPosition(Name name, double position);
  /**
   * @brief setJointVelocity
   * @param name
   * @param velocity
   */
  void setJointVelocity(Name name, double velocity);
  /**
   * @brief setJointAcceleration
   * @param name
   * @param acceleration
   */
  void setJointAcceleration(Name name, double acceleration);
  /**
   * @brief setJointEffort
   * @param name
   * @param effort
   */
  void setJointEffort(Name name, double effort);
  /**
   * @brief setJointValue
   * @param name
   * @param joint_value
   */
  void setJointValue(Name name, JointValue joint_value);
  /**
   * @brief setAllActiveJointPosition
   * @param joint_position_vector
   */
  void setAllActiveJointPosition(std::vector<double> joint_position_vector);
  /**
   * @brief setAllActiveJointValue
   * @param joint_value_vector
   */
  void setAllActiveJointValue(std::vector<JointValue> joint_value_vector);
  /**
   * @brief setAllJointPosition
   * @param joint_position_vector
   */
  void setAllJointPosition(std::vector<double> joint_position_vector);
  /**
   * @brief setAllJointValue
   * @param joint_value_vector
   */
  void setAllJointValue(std::vector<JointValue> joint_value_vector);
  /**
   * @brief setAllToolPosition
   * @param tool_position_vector
   */
  void setAllToolPosition(std::vector<double> tool_position_vector);
  /**
   * @brief setAllToolValue
   * @param tool_value_vector
   */
  void setAllToolValue(std::vector<JointValue> tool_value_vector);


  /*****************************************************************************
  ** Get Function
  *****************************************************************************/
  /**
   * @brief getDOF
   * @return
   */
  int8_t getDOF();
  /**
   * @brief getWorldName
   * @return
   */
  Name getWorldName();
  /**
   * @brief getWorldChildName
   * @return
   */
  Name getWorldChildName();
  /**
   * @brief getWorldPose
   * @return
   */
  Pose getWorldPose();
  /**
   * @brief getWorldKinematicPose
   * @return
   */
  KinematicPose getWorldKinematicPose();
  /**
   * @brief getWorldPosition
   * @return
   */
  Eigen::Vector3d getWorldPosition();
  /**
   * @brief getWorldOrientation
   * @return
   */
  Eigen::Matrix3d getWorldOrientation();
  /**
   * @brief getWorldDynamicPose
   * @return
   */
  DynamicPose getWorldDynamicPose();
  /**
   * @brief getComponentSize
   * @return
   */
  int8_t getComponentSize();
  /**
   * @brief getAllComponent
   * @return
   */
  std::map<Name, Component> getAllComponent();
  /**
   * @brief getIteratorBegin
   * @return
   */
  std::map<Name, Component>::iterator getIteratorBegin();
  /**
   * @brief getIteratorEnd
   * @return
   */
  std::map<Name, Component>::iterator getIteratorEnd();
  /**
   * @brief getComponent
   * @param component_name
   * @return
   */
  Component getComponent(Name component_name);
  /**
   * @brief getComponentActuatorName
   * @param component_name
   * @return
   */
  Name getComponentActuatorName(Name component_name);
  /**
   * @brief getComponentParentName
   * @param component_name
   * @return
   */
  Name getComponentParentName(Name component_name);
  /**
   * @brief getComponentChildName
   * @param component_name
   * @return
   */
  std::vector<Name> getComponentChildName(Name component_name);
  /**
   * @brief getComponentPoseFromWorld
   * @param component_name
   * @return
   */
  Pose getComponentPoseFromWorld(Name component_name);
  /**
   * @brief getComponentKinematicPoseFromWorld
   * @param component_name
   * @return
   */
  KinematicPose getComponentKinematicPoseFromWorld(Name component_name);
  /**
   * @brief getComponentPositionFromWorld
   * @param component_name
   * @return
   */
  Eigen::Vector3d getComponentPositionFromWorld(Name component_name);
  /**
   * @brief getComponentOrientationFromWorld
   * @param component_name
   * @return
   */
  Eigen::Matrix3d getComponentOrientationFromWorld(Name component_name);
  /**
   * @brief getComponentDynamicPoseFromWorld
   * @param component_name
   * @return
   */
  DynamicPose getComponentDynamicPoseFromWorld(Name component_name);
  /**
   * @brief getComponentRelativePoseFromParent
   * @param component_name
   * @return
   */
  KinematicPose getComponentRelativePoseFromParent(Name component_name);
  /**
   * @brief getComponentRelativePositionFromParent
   * @param component_name
   * @return
   */
  Eigen::Vector3d getComponentRelativePositionFromParent(Name component_name);
  /**
   * @brief getComponentRelativeOrientationFromParent
   * @param component_name
   * @return
   */
  Eigen::Matrix3d getComponentRelativeOrientationFromParent(Name component_name);
  /**
   * @brief getId
   * @param component_name
   * @return
   */
  int8_t getId(Name component_name);
  /**
   * @brief getCoefficient
   * @param component_name
   * @return
   */
  double getCoefficient(Name component_name);
  /**
   * @brief getAxis
   * @param component_name
   * @return
   */
  Eigen::Vector3d getAxis(Name component_name);
  /**
   * @brief getJointPosition
   * @param component_name
   * @return
   */
  double getJointPosition(Name component_name);
  /**
   * @brief getJointVelocity
   * @param component_name
   * @return
   */
  double getJointVelocity(Name component_name);
  /**
   * @brief getJointAcceleration
   * @param component_name
   * @return
   */
  double getJointAcceleration(Name component_name);
  /**
   * @brief getJointEffort
   * @param component_name
   * @return
   */
  double getJointEffort(Name component_name);
  /**
   * @brief getJointValue
   * @param component_name
   * @return
   */
  JointValue getJointValue(Name component_name);
  /**
   * @brief getComponentMass
   * @param component_name
   * @return
   */
  double getComponentMass(Name component_name);
  /**
   * @brief getComponentInertiaTensor
   * @param component_name
   * @return
   */
  Eigen::Matrix3d getComponentInertiaTensor(Name component_name);
  /**
   * @brief getComponentCenterOfMass
   * @param component_name
   * @return
   */
  Eigen::Vector3d getComponentCenterOfMass(Name component_name);
  /**
   * @brief getAllJointPosition
   * @return
   */
  std::vector<double> getAllJointPosition();
  /**
   * @brief getAllJointValue
   * @return
   */
  std::vector<JointValue> getAllJointValue();
  /**
   * @brief getAllActiveJointPosition
   * @return
   */
  std::vector<double> getAllActiveJointPosition();
  /**
   * @brief getAllActiveJointValue
   * @return
   */
  std::vector<JointValue> getAllActiveJointValue();
  /**
   * @brief getAllToolPosition
   * @return
   */
  std::vector<double> getAllToolPosition();
  /**
   * @brief getAllToolValue
   * @return
   */
  std::vector<JointValue> getAllToolValue();
  /**
   * @brief getAllJointID
   * @return
   */
  std::vector<uint8_t> getAllJointID();
  /**
   * @brief getAllActiveJointID
   * @return
   */
  std::vector<uint8_t> getAllActiveJointID();
  /**
   * @brief getAllToolComponentName
   * @return
   */
  std::vector<Name> getAllToolComponentName();
  /**
   * @brief getAllActiveJointComponentName
   * @return
   */
  std::vector<Name> getAllActiveJointComponentName();


  /*****************************************************************************
  ** Check Function
  *****************************************************************************/
  /**
   * @brief checkJointLimit
   * @param Component_name
   * @param value
   * @return
   */
  bool checkJointLimit(Name Component_name, double value);
  /**
   * @brief checkComponentType
   * @param component_name
   * @param component_type
   * @return
   */
  bool checkComponentType(Name component_name, ComponentType component_type);


  /*****************************************************************************
  ** Find Function
  *****************************************************************************/
  /**
   * @brief findComponentNameUsingId
   * @param id
   * @return
   */
  Name findComponentNameUsingId(int8_t id);
};

}
#endif // ROBOTIS_MANIPULATOR_COMMON_H

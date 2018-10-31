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

/* Authors: Darby Lim, Hye-Jong KIM */

#ifndef ROBOTIS_MANIPULATOR_H_
#define ROBOTIS_MANIPULATOR_H_

#include "robotis_manipulator_common.h"
#include "robotis_manipulator_manager.h"
#include "robotis_manipulator_trajectory_generator.h"
#include "robotis_manipulator_math.h"

#include <algorithm> // for sort()


namespace ROBOTIS_MANIPULATOR
{

class RobotisManipulator
{
private:
  Manipulator manipulator_;
  ManipulationTrajectory trajectory_;

  std::map<Name, JointActuator *> joint_actuator_;
  std::map<Name, ToolActuator *> tool_actuator_;
  Kinematics *kinematics_;

  ManipulationTime manipulation_time_;
  bool moving_;

public:
  RobotisManipulator();
  virtual ~RobotisManipulator();

  ///////////////////////////*initialize function*/////////////////////////////
  void addWorld(Name world_name,
                Name child_name,
                Eigen::Vector3d world_position = Eigen::Vector3d::Zero(3),
                Eigen::Quaterniond world_orientation = Eigen::Quaterniond::Identity());



  void addComponent(Name my_name,
                    Name parent_name,
                    Name child_name,
                    Eigen::Vector3d relative_position,
                    Eigen::Quaterniond relative_orientation,
                    Eigen::Vector3d axis_of_rotation = Eigen::Vector3d::Zero(),
                    int8_t joint_actuator_id = -1,
                    double coefficient = 1.0,
                    double mass = 0.0,
                    Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Identity(3, 3),
                    Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero());

  void addTool(Name my_name,
               Name parent_name,
               Eigen::Vector3d relative_position,
               Eigen::Quaterniond relative_orientation,
               int8_t tool_id = -1,
               double coefficient = 1.0,
               double mass = 0.0,
               Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Identity(3, 3),
               Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero());

  void addComponentChild(Name my_name, Name child_name);
  void checkManipulatorSetting();

  void addKinematics(Kinematics *kinematics);
  void addJointActuator(Name actuator_name, JointActuator *joint_actuator);
  void addToolActuator(Name tool_name, ToolActuator *tool_actuator);
  void addDrawingTrajectory(Name name, DrawingTrajectory *drawing);

  void initWayPoint(std::vector<double> joint_value_vector); //****

  // MANIPULATOR
  ///////////////////////////////Set function//////////////////////////////////
  void setWorldPose(Pose world_pose);
  void setWorldPosition(Eigen::Vector3d world_position);
  void setWorldOrientation(Eigen::Quaterniond world_orientation);
  void setWorldDynamicPose(Dynamicpose world_dynamic_pose);
  void setWorldLinearVelocity(Eigen::Vector3d world_linear_velocity);
  void setWorldAngularVelocity(Eigen::Vector3d world_angular_velocity);
  void setWorldLinearAcceleration(Eigen::Vector3d world_angular_velocity);
  void setWorldAngularAcceleration(Eigen::Vector3d world_angular_acceleration);
  void setComponent(Name component_name, Component component);
  void setComponentPoseToWorld(Name name, Pose pose_to_world);
  void setComponentPositionToWorld(Name name, Eigen::Vector3d position_to_world);
  void setComponentOrientationToWorld(Name name, Eigen::Quaterniond orientation_to_wolrd);
  void setComponentDynamicPoseToWorld(Name name, Dynamicpose dynamic_pose);
  void setJointValue(Name name, double joint_value);
  void setJointVelocity(Name name, double joint_velocity);
  void setJointAcceleration(Name name, double joint_acceleration);
  void setAllAcutiveJointValue(std::vector<double> joint_value_vector);
  void setAllAcutiveJointValue(std::vector<double> joint_value_vector, std::vector<double> joint_velocity_vector, std::vector<double> joint_acceleration_vector);
  void setAllJointValue(std::vector<double> joint_value_vector);
  void setAllJointValue(std::vector<double> joint_value_vector, std::vector<double> joint_velocity_vector, std::vector<double> joint_acceleration_vector);
  void setJointActuatorValue(Name name, Actuator actuator_value);
  void setAllJointActuatorValue(std::vector<Actuator> actuator_value_vector);
  void setToolValue(Name name, double tool_value);
  void setToolActuatorValue(Name name, double actuator_value);

  ///////////////////////////////Get function//////////////////////////////////
  int8_t getDOF();
  Name getWorldName();
  Name getWorldChildName();
  Pose getWorldPose();
  Eigen::Vector3d getWorldPosition();
  Eigen::Quaterniond getWorldOrientation();
  Dynamicpose getWorldDynamicPose();
  int8_t getComponentSize();
  std::map<Name, Component> getAllComponent();
  std::map<Name, Component>::iterator getIteratorBegin();
  std::map<Name, Component>::iterator getIteratorEnd();
  Component getComponent(Name name);
  Name getComponentParentName(Name name);
  std::vector<Name> getComponentChildName(Name name);
  Pose getComponentPoseToWorld(Name name);
  Eigen::Vector3d getComponentPositionToWorld(Name name);
  Eigen::Quaterniond getComponentOrientationToWorld(Name name);
  Dynamicpose getComponentDynamicPoseToWorld(Name name);
  Pose getComponentRelativePoseToParent(Name name);
  Eigen::Vector3d getComponentRelativePositionToParent(Name name);
  Eigen::Quaterniond getComponentRelativeOrientationToParent(Name name);
  Joint getComponentJoint(Name name);
  int8_t getJointId(Name name);
  double getJointCoefficient(Name name);
  Eigen::Vector3d getJointAxis(Name name);
  double getJointValue(Name name);
  double getJointVelocity(Name name);
  double getJointAcceleration(Name name);
  Actuator getJointActuatorValue(Name name);
  int8_t getToolId(Name name);
  double getToolCoefficient(Name name);
  double getToolValue(Name name);
  double getToolActuatorValue(Name name);
  double getComponentMass(Name name);
  Eigen::Matrix3d getComponentInertiaTensor(Name name);
  Eigen::Vector3d getComponentCenterOfMass(Name name);
  std::vector<double> getAllJointValue();
  std::vector<double> getAllActiveJointValue();
  void getAllActiveJointValue(std::vector<double> *joint_value_vector, std::vector<double> *joint_velocity_vector, std::vector<double> *joint_accelerarion_vector);
  std::vector<Actuator> getAllJointActuatorValue();
  std::vector<uint8_t> getAllJointID();
  std::vector<uint8_t> getAllActiveJointID();

  // KINEMATICS (INCLUDES VIRTUAL)
  void updatePassiveJointValue();
  Eigen::MatrixXd jacobian(Name tool_name);
  void forward();
  void forward(Name first_component_name);
  std::vector<double> inverse(Name tool_name, Pose goal_pose);

  // ACTUATOR (INCLUDES VIRTUAL)
  void JointActuatorInit(Name actuator_name, std::vector<uint8_t> id_array, const void *arg);
  void toolActuatorInit(Name actuator_name, uint8_t id, const void *arg);
  void JointActuatorSetMode(Name actuator_name, std::vector<uint8_t> id_array, const void *arg);
  void toolActuatorSetMode(Name actuator_name, const void *arg);
  std::vector<uint8_t> getJointActuatorId(Name actuator_name);
  uint8_t getToolActuatorId(Name actuator_name);

  void actuatorEnable(Name actuator_name);
  void actuatorDisable(Name actuator_name);
  void allActuatorEnable();
  void allActuatorDisable();

  void allJointActuatorEnable();
  void allJointActuatorDisable();
  bool sendJointActuatorValue(Name actuator_name, uint8_t actuator_id, Actuator value);
  bool sendMultipleJointActuatorValue(Name actuator_name, std::vector<uint8_t> actuator_id, std::vector<Actuator> value_vector);
  Actuator receiveJointActuatorValue(Name actuator_name, uint8_t actuator_id);
  std::vector<Actuator> receiveMultipleJointActuatorValue(Name actuator_name, std::vector<uint8_t> actuator_id);
  bool sendAllJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<Actuator> value_vector);
  std::vector<Actuator> receiveAllJointActuatorValue(std::vector<uint8_t> actuator_id);

  void allToolActuatorEnable();
  void allToolActuatorDisable();
  bool sendToolActuatorValue(uint8_t actuator_id, double value);
  double receiveToolActuatorValue(Name actuator_name, uint8_t actuator_id);
  bool sendAllToolActuatorValue(std::vector<uint8_t> actuator_id, std::vector<double> value_vector);
  std::vector<double> receiveAllToolActuatorValue(std::vector<uint8_t> actuator_id);

  // time
  void setPresentTime(double present_time);
  void setMoveTime(double move_time);
  void setControlTime(double control_time);
  double getMoveTime();
  double getControlTime();
  void startMoving();
  bool isMoving();

  //Way Point

  void setPresentJointWayPoint(std::vector<WayPoint> joint_value_vector);
  void setPresentTaskWayPoint(Name tool_name, std::vector<WayPoint> tool_position_value_vector);
  std::vector<WayPoint> getPresentJointWayPoint();
  std::vector<WayPoint> getPresentTaskWayPoint(Name tool_name);
  void UpdatePresentWayPoint(); //forward kinematics,dynamics

  void setStartWayPoint(std::vector<WayPoint> start_way_point);
  void setGoalWayPoint(std::vector<WayPoint> goal_way_point);
  void clearStartWayPoint();
  void clearGoalWayPoint();
  std::vector<WayPoint> getStartWayPoint();
  std::vector<WayPoint> getGoalWayPoint();

  //Trajectory

  void makeJointTrajectory();
  void makeTaskTrajectory();
  void makeDrawingTrajectory(Name drawing_name, const void *arg);

  //Trajectory Control Fuction

  void jointTrajectoryMove(std::vector<double> goal_joint_angle, double move_time);
  void jointTrajectoryMove(Name tool_name, Pose goal_pose, double move_time);
  void taskTrajectoryMove(Name tool_name, Pose goal_pose, double move_time);
  void drawingTrajectoryMove(Name drawing_name, Name tool_name, double *arg, double move_time);
  void drawingTrajectoryMove(Name drawing_name, double *arg, double move_time);
  void TrajectoryWait(double wait_time);

  // Additional Functions

  std::vector<Actuator> getTrajectoryJointValue(double tick_time);
  std::vector<Actuator> TrajectoryTimeCounter();
  void trajectoryControllerLoop(double present_time);

  double toolMove(Name tool_name, double tool_value);


};
} // namespace OPEN_MANIPULATOR

#endif

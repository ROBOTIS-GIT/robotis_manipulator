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
  std::map<Name, JointActuator *>::iterator it_joint_actuator_;

  std::map<Name, ToolActuator *> tool_actuator_;  
  std::map<Name, ToolActuator *>::iterator it_tool_actuator_;

  Kinematics *kinematics_;

  ManipulationTime manipulation_time_;
  bool moving_;

private:
  // time
  void setPresentTime(double present_time);
  void setMoveTime(double move_time);
  void startMoving();

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

  std::vector<Actuator> getTrajectoryJointValue(double tick_time);
  std::vector<Actuator> TrajectoryTimeCounter();

public:
  RobotisManipulator();
  virtual ~RobotisManipulator();

  ///////////////////////////*initialize function*/////////////////////////////
  void addWorld(Name world_name,
                Name child_name,
                Eigen::Vector3d world_position = Eigen::Vector3d::Zero(3),
                Eigen::Matrix3d world_orientation = Eigen::Matrix3d::Identity());

  void addComponent(Name my_name,
                    Name parent_name,
                    Name child_name,
                    Eigen::Vector3d relative_position,
                    Eigen::Matrix3d relative_orientation,
                    Eigen::Vector3d axis_of_rotation = Eigen::Vector3d::Zero(),
                    int8_t joint_actuator_id = -1,
                    double coefficient = 1.0,
                    double mass = 0.0,
                    Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Identity(3, 3),
                    Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero());

  void addTool(Name my_name,
               Name parent_name,
               Eigen::Vector3d relative_position,
               Eigen::Matrix3d relative_orientation,
               int8_t tool_id = -1,
               double coefficient = 1.0,
               double mass = 0.0,
               Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Identity(3, 3),
               Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero());

  void addComponentChild(Name my_name, Name child_name);
  void checkManipulatorSetting();

  void addKinematics(Kinematics *kinematics);
  void addJointActuator(Name actuator_name, JointActuator *joint_actuator, std::vector<uint8_t> id_array, const void *arg);
  void addToolActuator(Name tool_name, ToolActuator *tool_actuator, uint8_t id, const void *arg);
  void addDrawingTrajectory(Name name, DrawingTrajectory *drawing);

  void initTrajectoryWayPoint(); //****

  // MANIPULATOR
  Manipulator *getManipulator();

  // KINEMATICS (INCLUDES VIRTUAL)
  void updatePassiveJointValue();
  Eigen::MatrixXd jacobian(Name tool_name);
  void forward();
  void forward(Name first_component_name);
  std::vector<double> inverse(Name tool_name, Pose goal_pose);

  // ACTUATOR (INCLUDES VIRTUAL)
  void jointActuatorSetMode(Name actuator_name, std::vector<uint8_t> id_array, const void *arg);
  void toolActuatorSetMode(Name actuator_name, const void *arg);

  std::vector<uint8_t> getJointActuatorId(Name actuator_name);
  uint8_t getToolActuatorId(Name actuator_name);

  void actuatorEnable(Name actuator_name);
  void actuatorDisable(Name actuator_name);
  void allJointActuatorEnable();
  void allJointActuatorDisable();
  void allToolActuatorEnable();
  void allToolActuatorDisable();
  void allActuatorEnable();
  void allActuatorDisable();

  bool sendJointActuatorValue(Name joint_component_name, WayPoint value);
  bool sendMultipleJointActuatorValue(std::vector<Name> tool_component_name, std::vector<WayPoint> value_vector);
  bool sendAllJointActuatorValue(std::vector<WayPoint> value_vector);

  WayPoint receiveJointActuatorValue(Name joint_component_name);
  std::vector<WayPoint> receiveMultipleJointActuatorValue(std::vector<Name> joint_component_name);
  std::vector<WayPoint> receiveAllJointActuatorValue();

  bool sendToolActuatorValue(Name tool_component_name, double value);
  double receiveToolActuatorValue(Name tool_component_name);

  bool sendAllToolActuatorValue(std::vector<Name> joint_component_name, std::vector<double> value_vector);
  std::vector<double> receiveAllToolActuatorValue(std::vector<Name> tool_component_name);

  // time
  void setControlTime(double control_time);
  double getMoveTime();
  double getControlTime();
  bool isMoving();

  //Trajectory Control Fuction

  void jointTrajectoryMove(std::vector<double> goal_joint_angle, double move_time);
  void jointTrajectoryMove(Name tool_name, Eigen::Vector3d goal_position, double move_time);
  void jointTrajectoryMove(Name tool_name, Eigen::Matrix3d goal_orientation, double move_time);
  void jointTrajectoryMove(Name tool_name, Pose goal_pose, double move_time);

  void taskTrajectoryMove(Name tool_name, Eigen::Vector3d goal_position, double move_time);
  void taskTrajectoryMove(Name tool_name, Eigen::Matrix3d goal_orientation, double move_time);
  void taskTrajectoryMove(Name tool_name, Pose goal_pose, double move_time);

  void drawingTrajectoryMove(Name drawing_name, Name tool_name, double *arg, double move_time);
  void drawingTrajectoryMove(Name drawing_name, double *arg, double move_time);
  void TrajectoryWait(double wait_time);

  // Additional Functions
  void trajectoryControllerLoop(double present_time);
  double toolMove(Name tool_name, double tool_value);

};
} // namespace OPEN_MANIPULATOR

#endif

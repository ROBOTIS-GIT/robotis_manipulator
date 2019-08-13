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
 * @file robotis_manipulator_trajecoty_generator.h
 * @brief
 * @details
 */

#ifndef ROBOTIS_MNAMIPULATOR_TRAJECTORY_GENERATOR_H_
#define ROBOTIS_MNAMIPULATOR_TRAJECTORY_GENERATOR_H_

#include <math.h>
#include <vector>

#include "robotis_manipulator_manager.h"

#if defined(__OPENCR__)
  #include <Eigen.h>  // Calls main Eigen matrix class library
  #include <Eigen/LU> // Calls inverse, determinant, LU decomp., etc.
  #include <Eigen/QR>
#else
  #include <eigen3/Eigen/Eigen>
  #include <eigen3/Eigen/LU>
  #include <eigen3/Eigen/QR>

  #define PI 3.141592
#endif

namespace robotis_manipulator
{

/**
 * @brief The MinimumJerk class
 */
class MinimumJerk
{
private:
  Eigen::VectorXd coefficient_;

public:
  MinimumJerk();
  virtual ~MinimumJerk();

  /**
   * @brief calcCoefficient
   * @param start
   * @param goal
   * @param move_time
   */
  void calcCoefficient(Point start,
                       Point goal,
                       double move_time);
  /**
   * @brief getCoefficient
   * @return
   */
  Eigen::VectorXd getCoefficient();
};

/**
 * @brief The JointTrajectory class
 */
class JointTrajectory
{
private:
  uint8_t coefficient_size_;
  MinimumJerk minimum_jerk_trajectory_generator_;
  Eigen::MatrixXd minimum_jerk_coefficient_;

public:
  JointTrajectory();
  virtual ~JointTrajectory();

  /**
   * @brief makeJointTrajectory
   * @param move_time
   * @param start
   * @param goal
   */
  bool makeJointTrajectory(double move_time,
            JointWaypoint start,
            JointWaypoint goal
            );
  /**
   * @brief getMinimumJerkCoefficient
   * @return
   */
  Eigen::MatrixXd getMinimumJerkCoefficient();
  /**
   * @brief getJointWaypoint
   * @param tick
   * @return
   */
  JointWaypoint getJointWaypoint(double tick);
};

/**
 * @brief The TaskTrajectory class
 */
class TaskTrajectory
{
private:
  uint8_t coefficient_size_;
  MinimumJerk minimum_jerk_trajectory_generator_;
  Eigen::MatrixXd minimum_jerk_coefficient_;

public:
  TaskTrajectory();
  virtual ~TaskTrajectory();

  /**
   * @brief makeTaskTrajectory
   * @param move_time
   * @param start
   * @param goal
   */
  bool makeTaskTrajectory(double move_time,
            TaskWaypoint start,
            TaskWaypoint goal
            );
  /**
   * @brief getMinimumJerkCoefficient
   * @return
   */
  Eigen::MatrixXd getMinimumJerkCoefficient();
  /**
   * @brief getTaskWaypoint
   * @param tick
   * @return
   */
  TaskWaypoint getTaskWaypoint(double tick);
};


/*****************************************************************************
** Trajectory Class
*****************************************************************************/

/**
 * @brief The Trajectory class
 */
class Trajectory
{
private:
  TrajectoryType trajectory_type_;
  Time trajectory_time_;
  Manipulator manipulator_;

  JointTrajectory joint_;
  TaskTrajectory task_;
  std::map<Name, CustomJointTrajectory *> cus_joint_;
  std::map<Name, CustomTaskTrajectory *> cus_task_;

  Name present_custom_trajectory_name_;
  Name present_control_tool_name_;

public:
  Trajectory() {}
  ~Trajectory() {}

  // Time
  /**
   * @brief setMoveTime
   * @param move_time
   */
  void setMoveTime(double move_time);
  /**
   * @brief setPresentTime
   * @param present_time
   */
  void setPresentTime(double present_time);
  /**
   * @brief setStartTimeToPresentTime
   */
  void setStartTimeToPresentTime();
  /**
   * @brief setStartTime
   * @param start_time
   */
  void setStartTime(double start_time);
  /**
   * @brief getMoveTime
   * @return
   */
  double getMoveTime();
  /**
   * @brief getTickTime
   * @return
   */
  double getTickTime();

  // Manipulator
  /**
   * @brief setManipulator
   * @param manipulator
   */
  void setManipulator(Manipulator manipulator);
  /**
   * @brief getManipulator
   * @return
   */
  Manipulator* getManipulator();

  // Get Trajectory
  /**
   * @brief getJointTrajectory
   * @return
   */
  JointTrajectory getJointTrajectory();
  /**
   * @brief getTaskTrajectory
   * @return
   */
  TaskTrajectory getTaskTrajectory();
  /**
   * @brief getCustomJointTrajectory
   * @param name
   * @return
   */
  CustomJointTrajectory* getCustomJointTrajectory(Name name);
  /**
   * @brief getCustomTaskTrajectory
   * @param name
   * @return
   */
  CustomTaskTrajectory* getCustomTaskTrajectory(Name name);

  // Custom Trajectory Setting
  /**
   * @brief addCustomTrajectory
   * @param trajectory_name
   * @param custom_trajectory
   */
  void addCustomTrajectory(Name trajectory_name, CustomJointTrajectory *custom_trajectory);
  /**
   * @brief addCustomTrajectory
   * @param trajectory_name
   * @param custom_trajectory
   */
  void addCustomTrajectory(Name trajectory_name, CustomTaskTrajectory *custom_trajectory);
  /**
   * @brief setCustomTrajectoryOption
   * @param trajectory_name
   * @param arg
   */
  void setCustomTrajectoryOption(Name trajectory_name, const void* arg);
  /**
   * @brief setPresentControlToolName
   * @param present_control_tool_name
   */
  void setPresentControlToolName(Name present_control_tool_name);
  /**
   * @brief getPresentCustomTrajectoryName
   * @return
   */
  Name getPresentCustomTrajectoryName();
  /**
   * @brief getPresentControlToolName
   * @return
   */
  Name getPresentControlToolName();

  // Waypoint
  /**
   * @brief initTrajectoryWaypoint
   * @param actual_manipulator
   * @param kinematics
   */
  void initTrajectoryWaypoint(Manipulator actual_manipulator, Kinematics *kinematics);

  /**
   * @brief updatePresentWaypoint
   * @param kinematics
   */
  void updatePresentWaypoint(Kinematics* kinematics); //forward kinematics,dynamics
  /**
   * @brief setPresentJointWaypoint
   * @param joint_value_vector
   */
  void setPresentJointWaypoint(JointWaypoint joint_value_vector);
  /**
   * @brief setPresentTaskWaypoint
   * @param tool_name
   * @param tool_position_value_vector
   */
  void setPresentTaskWaypoint(Name tool_name, TaskWaypoint tool_position_value_vector);
  /**
   * @brief getPresentJointWaypoint
   * @return
   */
  JointWaypoint getPresentJointWaypoint();
  /**
   * @brief getPresentTaskWaypoint
   * @param tool_name
   * @return
   */
  TaskWaypoint getPresentTaskWaypoint(Name tool_name);
  /**
   * @brief removeWaypointDynamicData
   * @param value
   * @return
   */
  JointWaypoint removeWaypointDynamicData(JointWaypoint value);
  /**
   * @brief removeWaypointDynamicData
   * @param value
   * @return
   */
  TaskWaypoint removeWaypointDynamicData(TaskWaypoint value);

  // Trajectory
  /**
   * @brief setTrajectoryType
   * @param trajectory_type
   */
  void setTrajectoryType(TrajectoryType trajectory_type);
  /**
   * @brief checkTrajectoryType
   * @param trajectory_type
   * @return
   */
  bool checkTrajectoryType(TrajectoryType trajectory_type);
  /**
   * @brief makeJointTrajectory
   * @param start_way_point
   * @param goal_way_point
   */
  bool makeJointTrajectory(JointWaypoint start_way_point, JointWaypoint goal_way_point);
  /**
   * @brief makeTaskTrajectory
   * @param start_way_point
   * @param goal_way_point
   */
  bool makeTaskTrajectory(TaskWaypoint start_way_point, TaskWaypoint goal_way_point);
  /**
   * @brief makeCustomTrajectory
   * @param trajectory_name
   * @param start_way_point
   * @param arg
   */
  bool makeCustomTrajectory(Name trajectory_name, JointWaypoint start_way_point, const void *arg);
  /**
   * @brief makeCustomTrajectory
   * @param trajectory_name
   * @param start_way_point
   * @param arg
   */
  bool makeCustomTrajectory(Name trajectory_name, TaskWaypoint start_way_point, const void *arg);

  // Tool
  /**
   * @brief setToolGoalPosition
   * @param tool_name
   * @param tool_goal_position
   */
  bool setToolGoalPosition(Name tool_name, double tool_goal_position);
  /**
   * @brief setToolGoalValue
   * @param tool_name
   * @param tool_goal_value
   */
  bool setToolGoalValue(Name tool_name, JointValue tool_goal_value);
  /**
   * @brief getToolGoalPosition
   * @param tool_name
   * @return
   */
  double getToolGoalPosition(Name tool_name);
  /**
   * @brief getToolGoalValue
   * @param tool_name
   * @return
   */
  JointValue getToolGoalValue(Name tool_name);
};

} // namespace robotis_manipulator
#endif // ROBOTIS_MNAMIPULATOR_TRAJECTORY_GENERATOR_H_




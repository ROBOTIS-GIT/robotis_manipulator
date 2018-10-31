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

#ifndef RMTRAJECTORY_H_
#define RMTRAJECTORY_H_

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/QR>

#include <math.h>
#include <vector>

#include "robotis_manipulator/robotis_manipulator_manager.h"

#define PI 3.141592

namespace ROBOTIS_MANIPULATOR
{
class MinimumJerk
{
private:
  Eigen::VectorXd coefficient_;

public:
  MinimumJerk();
  virtual ~MinimumJerk();

  void calcCoefficient(WayPoint start,
                       WayPoint goal,
                       double move_time,
                       double control_time);

  Eigen::VectorXd getCoefficient();
};

class JointTrajectory
{
private:
  MinimumJerk trajectory_generator_;

  uint8_t joint_num_;
  Eigen::MatrixXd coefficient_;
  std::vector<WayPoint> joint_way_point_;

public:
  JointTrajectory(uint8_t joint_num);
  virtual ~JointTrajectory();

  void init(double move_time,
            double control_time,
            std::vector<WayPoint> start,
            std::vector<WayPoint> goal
            );

  std::vector<WayPoint> getJointWayPoint(double tick);

  Eigen::MatrixXd getCoefficient();
};

class TaskTrajectory
{
private:
  MinimumJerk trajectory_generator_;

  uint8_t dof_;
  Eigen::MatrixXd position_coefficient_;
  std::vector<WayPoint> task_position_way_point_;

public:
  TaskTrajectory();
  virtual ~TaskTrajectory();

  void init(double move_time,
            double control_time,
            std::vector<WayPoint> start,
            std::vector<WayPoint> goal
            );
  std::vector<WayPoint> getTaskWayPoint(double tick);

  Eigen::MatrixXd getCoefficient();
};


class ManipulationTrajectory
{
public:
  TrajectoryType trajectory_type_;
  Manipulator manipulator_;

  std::vector<WayPoint> start_way_point_;
  std::vector<WayPoint> goal_way_point_;

  JointTrajectory joint_;
  TaskTrajectory task_;
  std::map<Name, DrawingTrajectory *> drawing_;
  Name present_drawing_object_name_;
  Name present_controled_tool_name_;


public:
  ManipulationTrajectory();
  ~ManipulationTrajectory();
};


} // namespace RM_TRAJECTORY
#endif // RMTRAJECTORY_H_





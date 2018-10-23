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

#include "robotis_manipulator/RMAPI.h"

#define PI 3.141592
using namespace Eigen;

typedef struct
{
  double position;
  double velocity;
  double acceleration;
} Trajectory;

namespace RM_TRAJECTORY
{
class MinimumJerk
{
private:
  VectorXf coefficient_;

public:
  MinimumJerk();
  virtual ~MinimumJerk();

  void calcCoefficient(Trajectory start,
                       Trajectory goal,
                       double move_time,
                       double control_time);

  VectorXf getCoefficient();
};

class JointTrajectory
{
private:
  MinimumJerk trajectory_generator_;

  uint8_t joint_num_;
  MatrixXf coefficient_;
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> acceleration_;

public:
  JointTrajectory(uint8_t joint_num);
  virtual ~JointTrajectory();

  void init(std::vector<Trajectory> start,
            std::vector<Trajectory> goal,
            double move_time,
            double control_time);

  std::vector<double> getPosition(double tick);
  std::vector<double> getVelocity(double tick);
  std::vector<double> getAcceleration(double tick);

  MatrixXf getCoefficient();
};

class TaskTrajectory
{
private:
  MinimumJerk trajectory_generator_;

  uint8_t num_of_axis_;
  MatrixXf coefficient_;
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> acceleration_;

public:
  TaskTrajectory();
  virtual ~TaskTrajectory();

  void init(std::vector<Trajectory> start,
            std::vector<Trajectory> goal,
            double move_time,
            double control_time);

  std::vector<double> getPosition(double tick);
  std::vector<double> getVelocity(double tick);
  std::vector<double> getAcceleration(double tick);

  MatrixXf getCoefficient();
};


} // namespace RM_TRAJECTORY
#endif // RMTRAJECTORY_H_





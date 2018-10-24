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

#ifndef OPEN_MANIPULATOR_H_
#define OPEN_MANIPULATOR_H_

#include "robotis_manipulator_common.h"
#include "robotis_manipulator_manager.h"
#include "robotis_manipulator_trajectory_generator.h"
#include "robotis_manipulator_math.h"

#include <algorithm> // for sort()

#define ACTUATOR_CONTROL_TIME 0.010//0.010    //go out
#define NUM_OF_DOF 4

#define JOINT_TRAJECTORY  0
#define TASK_TRAJECTORY   1
#define DRAWING           2


using namespace Eigen;

namespace MUTEX
{
void create();
void wait();
void release();
} // namespace MUTEX

namespace THREAD
{
void Robot_State(void const *argument);
void Actuator_Control(void const *argument);
} // namespace THREAD

namespace ROBOTIS_MANIPULATOR
{

class RobotisManipulator
{
private:
  Manipulator manipulator_;

  Goal previous_goal_;

  JointTrajectory *joint_trajectory_;
  std::vector<Trajectory> start_joint_trajectory_;
  std::vector<Trajectory> goal_joint_trajectory_;

  TaskTrajectory *task_trajectory_;
  std::vector<Trajectory> start_task_trajectory_;
  std::vector<Trajectory> goal_task_trajectory_;

  Kinematics *kinematics_;
  std::map<Name, Actuator *> actuator_;
  std::map<Name, Drawing *> drawing_;

  double move_time_;
  double control_time_;

  bool moving_;
  double present_time_;        //[s]
  double start_time_;

  bool platform_;
  bool processing_;

  Name object_;
  Name trajectory_type_;

public:
  RobotisManipulator();
  virtual ~RobotisManipulator();

  void initKinematics(Kinematics *kinematics);
  void addActuator(Name name, Actuator *actuator);
  void addDraw(Name name, Drawing *drawing);

  void initTrajectory(std::vector<double> angle_vector);

  ///////////////////////////*initialize function*/////////////////////////////
  void addWorld(Name world_name,
                Name child_name,
                Vector3f world_position = Vector3f::Zero(),
                Matrix3f world_orientation = Matrix3f::Identity(3, 3));

  void addComponent(Name my_name,
                    Name parent_name,
                    Name child_name,
                    Vector3f relative_position,
                    Matrix3f relative_orientation,
                    Vector3f axis_of_rotation = Vector3f::Zero(),
                    int8_t actuator_id = -1,
                    double coefficient = 1,
                    double mass = 0.0,
                    Matrix3f inertia_tensor = Matrix3f::Identity(3, 3),
                    Vector3f center_of_mass = Vector3f::Zero());

  void addTool(Name my_name,
               Name parent_name,
               Vector3f relative_position,
               Matrix3f relative_orientation,
               int8_t tool_id = -1,
               double coefficient = 1,
               double mass = 0.0,
               Matrix3f inertia_tensor = Matrix3f::Identity(3, 3),
               Vector3f center_of_mass = Vector3f::Zero());

  void addComponentChild(Name my_name, Name child_name);
  void checkManipulatorSetting();

  ///////////////////////////////Set function//////////////////////////////////
  void setWorldPose(Pose world_pose);
  void setWorldPosition(Vector3f world_position);
  void setWorldOrientation(Matrix3f world_orientation);
  void setWorldState(State world_state);
  void setWorldVelocity(VectorXf world_velocity);
  void setWorldAcceleration(VectorXf world_acceleration);

  void setComponent(Name name, Component component);
  void setComponentPoseToWorld(Name name, Pose pose_to_world);
  void setComponentPositionToWorld(Name name, Vector3f position_to_world);
  void setComponentOrientationToWorld(Name name, Matrix3f orientation_to_wolrd);
  void setComponentStateToWorld(Name name, State state_to_world);
  void setComponentVelocityToWorld(Name name, VectorXf velocity);
  void setComponentAccelerationToWorld(Name name, VectorXf accelaration);
  void setComponentJointAngle(Name name, double angle);
  void setComponentJointVelocity(Name name, double angular_velocity);
  void setComponentJointAcceleration(Name name, double angular_acceleration);
  void setComponentToolOnOff(Name name, bool on_off);
  void setComponentToolValue(Name name, double actuator_value);

  void setAllActiveJointAngle(std::vector<double> angle_vector);

  ///////////////////////////////Get function//////////////////////////////////
  int8_t getDOF();

  int8_t getComponentSize();
  Name getWorldName();
  Name getWorldChildName();
  Pose getWorldPose();
  Vector3f getWorldPosition();
  Matrix3f getWorldOrientation();
  State getWorldState();
  VectorXf getWorldVelocity();
  VectorXf getWorldAcceleration();

  std::map<Name, Component> getAllComponent();
  std::map<Name, Component>::iterator getIteratorBegin();
  std::map<Name, Component>::iterator getIteratorEnd();
  Component getComponent(Name name);
  Name getComponentParentName(Name name);
  std::vector<Name> getComponentChildName(Name name);
  Pose getComponentPoseToWorld(Name name);
  Vector3f getComponentPositionToWorld(Name name);
  Matrix3f getComponentOrientationToWorld(Name name);
  State getComponentStateToWorld(Name name);
  VectorXf getComponentVelocityToWorld(Name name);
  VectorXf getComponentAccelerationToWorld(Name name);
  Pose getComponentRelativePoseToParent(Name name);
  Vector3f getComponentRelativePositionToParent(Name name);
  Matrix3f getComponentRelativeOrientationToParent(Name name);
  Joint getComponentJoint(Name name);
  int8_t getComponentJointId(Name name);
  double getComponentJointCoefficient(Name name);
  Vector3f getComponentJointAxis(Name name);
  double getComponentJointAngle(Name name);
  double getComponentJointVelocity(Name name);
  double getComponentJointAcceleration(Name name);
  Tool getComponentTool(Name name);
  int8_t getComponentToolId(Name name);
  double getComponentToolCoefficient(Name name);
  bool getComponentToolOnOff(Name name);
  double getComponentToolValue(Name name);
  double getComponentMass(Name name);
  Matrix3f getComponentInertiaTensor(Name name);
  Vector3f getComponentCenterOfMass(Name name);

  std::vector<double> getAllJointAngle();
  std::vector<double> getAllActiveJointAngle();
  std::vector<uint8_t> getAllActiveJointID();

  // KINEMATICS (INCLUDES VIRTUAL)
  MatrixXf jacobian(Name tool_name);
  void forward();
  void forward(Name first_component_name);
  std::vector<double> inverse(Name tool_name, Pose goal_pose);

  // ACTUATOR (INCLUDES VIRTUAL)
  void actuatorInit(Name actuator_name, const void *arg);
  void setActuatorControlMode(Name actuator_name);
  void actuatorEnable(Name actuator_name);
  void actuatorDisable(Name actuator_name);
  std::vector<double> sendAllActuatorAngle(Name actuator_name, std::vector<double> radian_vector);
  std::vector<double> sendMultipleActuatorAngle(std::vector<uint8_t> active_joint_id, std::vector<double> radian_vector);
  double sendActuatorAngle(uint8_t active_joint_id, double radian);
  bool sendActuatorSignal(Name actuator_name, uint8_t active_joint_id, bool onoff);
  std::vector<double> receiveAllActuatorAngle(Name actuator_name);

  // DRAW (INCLUDES VIRTUAL)
  void drawInit(Name name, double move_time, const void *arg);
  void setRadiusForDrawing(Name name, double radius);
  void setStartAngularPositionForDrawing(Name name, double start_position);

  Pose getPoseForDrawing(Name name, double tick);

  // trajectory
  void setPresentTime(double present_time);
  void setMoveTime(double move_time);
  void setControlTime(double control_time);

  double getMoveTime();
  double getControlTime();
  void startMoving();
  bool isMoving();

  void makeTrajectory(std::vector<Trajectory> start, std::vector<Trajectory> goal);

  void setStartTrajectory(Trajectory trajectory);
  void clearStartTrajectory();
  std::vector<Trajectory> getStartTrajectory();

  void setGoalTrajectory(Trajectory trajectory);
  void clearGoalTrajectory();
  std::vector<Trajectory> getGoalTrajectory();


  // Additional Functions

  double toolMove(Name tool_name, double tool_value);
  void wait(double wait_time = 1.0f);

  void setPreviousGoalPosition(std::vector<double> data);
  std::vector<double> getPreviousGoalPosition();
  void setStartPoseForDrawing(Name name, Pose start_pose);
  void setEndPoseForDrawing(Name name, Pose end_pose);

  std::vector<double> controlLoop(double present_time, Name tool_name, Name actuator_name);
  Goal getJointAngleFromJointTraj();
  Goal getJointAngleFromTaskTraj(Name tool_name);
  Goal getJointAngleFromDrawing(Name tool_name);
  void setJointTrajectory(std::vector<double> goal_position, double move_time);
  void setJointTrajectory(Name tool_name, Pose goal_pose, double move_time);
  void setTaskTrajectory(Name tool_name, Pose goal_pose, double move_time);
  void setDrawing(Name tool_name, int object, double move_time, double option);
  void setDrawing(Name tool_name, int object, double move_time, Vector3f meter);

};
} // namespace OPEN_MANIPULATOR

#endif

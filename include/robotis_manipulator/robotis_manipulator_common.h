#ifndef ROBOTIS_MANIPULATOR_COMMON_H
#define ROBOTIS_MANIPULATOR_COMMON_H

#include <unistd.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
#include <math.h>
#include <vector>
#include <map>
#include "robotis_manipulator_math.h"

namespace ROBOTIS_MANIPULATOR
{

typedef int8_t Name;

/////////////////////Pose struct////////////////////////////

typedef struct _Pose
{
  Eigen::Vector3d position;
  Eigen::Matrix3d orientation;
} Pose;

typedef struct _Dynamicvector
{
  Eigen::Vector3d velocity;
  Eigen::Vector3d effort;
} Dynamicvector;

typedef struct _Dynamicpose
{
  Dynamicvector linear;
  Dynamicvector angular;
} Dynamicpose;

////////////////////////////////////////////////////////////

/////////////////////Inertia struct/////////////////////////

typedef struct
{
  double mass;
  Eigen::Matrix3d inertia_tensor;
  Eigen::Vector3d center_of_mass;
} Inertia;

////////////////////////////////////////////////////////////

typedef struct _Point
{
  double value;
  double velocity;
  double effort;
} Actuator, WayPoint;

///////////////////Trajectory struct////////////////////////

enum TrajectoryType
{
  NONE = 0,
  JOINT_TRAJECTORY,
  TASK_TRAJECTORY,
  DRAWING_TRAJECTORY
};

typedef enum _WayPointType
{
  JOINT = 0,
  TASK
}WayPointType;

////////////////////////////////////////////////////////////

/////////////////////Joint struct///////////////////////////

typedef struct _Joint
{
  int8_t id;
  Eigen::Vector3d axis;
  double coefficient; //actuator angle to joint angle
  double value;
  double velocity;
  double effort;
} Joint;

////////////////////////////////////////////////////////////

/////////////////////Tool struct////////////////////////////

typedef struct _Tool
{
  int8_t id;
  double coefficient; //actuator value to tool value
  double value;       //m or rad
} Tool;

////////////////////////////////////////////////////////////

/////////////////////World struct///////////////////////////

typedef struct _World
{
  Name name;
  Name child;
  Pose pose;
  Dynamicpose dynamic_pose;
} World;

////////////////////////////////////////////////////////////

/////////////////////Componet struct////////////////////////

typedef struct _Component
{
  Name parent;
  std::vector<Name> child;
  Pose relative_to_parent;
  Pose pose_to_world;
  Dynamicpose dynamic_pose;
  Joint joint;
  Tool tool;
  Inertia inertia;
  Name actuator_name;
} Component;

////////////////////////////////////////////////////////////

/////////////////////////Time struct////////////////////////

typedef struct _ManipulationTime
{
  double move_time;
  double control_time;
  double present_time;
  double start_time;
}ManipulationTime;

////////////////////////////////////////////////////////////

/////////////////////Manipulator class//////////////////////

class Manipulator
{
private:
  int8_t dof_;
  World world_;
  std::map<Name, Component> component_;
  std::map<Name, Component>::iterator it_component_;

public:
  Manipulator();
  ~Manipulator() {}

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

  ///////////////////////////////Set function//////////////////////////////////
  void setWorldPose(Pose world_pose);
  void setWorldPosition(Eigen::Vector3d world_position);
  void setWorldOrientation(Eigen::Matrix3d world_orientation);
  void setWorldDynamicPose(Dynamicpose world_dynamic_pose);
  void setWorldLinearVelocity(Eigen::Vector3d world_linear_velocity);
  void setWorldAngularVelocity(Eigen::Vector3d world_angular_velocity);
  void setWorldLinearEffort(Eigen::Vector3d world_angular_velocity);
  void setWorldAngularEffort(Eigen::Vector3d world_angular_effort);
  void setComponent(Name component_name, Component component);
  void setComponentActuatorName(Name component_name, Name actuator_name);
  void setComponentPoseToWorld(Name name, Pose pose_to_world);
  void setComponentPositionToWorld(Name name, Eigen::Vector3d position_to_world);
  void setComponentOrientationToWorld(Name name, Eigen::Matrix3d orientation_to_wolrd);
  void setComponentDynamicPoseToWorld(Name name, Dynamicpose dynamic_pose);
  void setJointValue(Name name, double joint_value);
  void setJointVelocity(Name name, double joint_velocity);
  void setJointeffort(Name name, double joint_effort);
  void setJointValue(Name name, WayPoint joint_value);
//  void setJointValueFromId(int8_t joint_id, WayPoint joint_value);
  void setAllActiveJointValue(std::vector<double> joint_value_vector);
  void setAllActiveJointValue(std::vector<WayPoint> joint_way_point_vector);
  void setAllJointValue(std::vector<double> joint_value_vector);
  void setAllJointValue(std::vector<WayPoint> joint_way_point_vector);
//  void setJointActuatorValue(Name name, Actuator actuator_value);
//  void setAllJointActuatorValue(std::vector<Actuator> actuator_value_vector);
  void setToolValue(Name name, double tool_value);
//  void setToolValueFromId(int8_t tool_id, double tool_value);
//  void setToolActuatorValue(Name name, double actuator_value);

  ///////////////////////////////Get function//////////////////////////////////
  int8_t getDOF();
  Name getWorldName();
  Name getWorldChildName();
  Pose getWorldPose();
  Eigen::Vector3d getWorldPosition();
  Eigen::Matrix3d getWorldOrientation();
  Dynamicpose getWorldDynamicPose();
  int8_t getComponentSize();
  std::map<Name, Component> getAllComponent();
  std::map<Name, Component>::iterator getIteratorBegin();
  std::map<Name, Component>::iterator getIteratorEnd();
  Component getComponent(Name name);
  Name getComponentActuatorName(Name component_name);
  Name getComponentParentName(Name name);
  std::vector<Name> getComponentChildName(Name name);
  Pose getComponentPoseToWorld(Name name);
  Eigen::Vector3d getComponentPositionToWorld(Name name);
  Eigen::Matrix3d getComponentOrientationToWorld(Name name);
  Dynamicpose getComponentDynamicPoseToWorld(Name name);
  Pose getComponentRelativePoseToParent(Name name);
  Eigen::Vector3d getComponentRelativePositionToParent(Name name);
  Eigen::Matrix3d getComponentRelativeOrientationToParent(Name name);
  Joint getComponentJoint(Name name);
  int8_t getJointId(Name name);
  double getJointCoefficient(Name name);
//  double getJointCoefficientFromId(int8_t id);
  Eigen::Vector3d getJointAxis(Name name);
  double getJointValue(Name name);
  double getJointVelocity(Name name);
  double getJointeffort(Name name);
//  Actuator getJointActuatorValue(Name name);
  int8_t getToolId(Name name);
  double getToolCoefficient(Name name);
//  double getToolCoefficientFromId(int8_t id);
  double getToolValue(Name name);
//  double getToolActuatorValue(Name name);
  double getComponentMass(Name name);
  Eigen::Matrix3d getComponentInertiaTensor(Name name);
  Eigen::Vector3d getComponentCenterOfMass(Name name);
  std::vector<double> getAllJointValue();
  std::vector<double> getAllActiveJointValue();
  void getAllActiveJointValue(std::vector<double> *joint_value_vector, std::vector<double> *joint_velocity_vector, std::vector<double> *joint_accelerarion_vector);
//  std::vector<Actuator> getAllJointActuatorValue();
  std::vector<uint8_t> getAllJointID();
  std::vector<uint8_t> getAllActiveJointID();

  ///////////////////////////////Find function//////////////////////////////////
  Name findJointComponentNameFromId(int8_t id);
  Name findToolComponentNameFromId(int8_t id);
};

////////////////////////////////////////////////////////////


}


#endif // ROBOTIS_MANIPULATOR_COMMON_H

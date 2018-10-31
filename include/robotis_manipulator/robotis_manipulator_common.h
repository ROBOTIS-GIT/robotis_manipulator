#ifndef ROBOTIS_MANIPULATOR_COMMON_H
#define ROBOTIS_MANIPULATOR_COMMON_H

#include <unistd.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
#include <math.h>
#include <vector>
#include <map>

namespace ROBOTIS_MANIPULATOR
{

typedef int8_t Name;

/////////////////////Pose struct////////////////////////////

typedef struct _Pose
{
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
} Pose;

typedef struct _Dynamicvector
{
  Eigen::Vector3d velocity;
  Eigen::Vector3d accelation;
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

typedef struct _Actuator
{
  double value;
  double velocity;
  double acceleration;
} Actuator;

///////////////////Trajectory struct////////////////////////

typedef struct _WayPoint
{
  double value;
  double velocity;
  double acceleration;
} WayPoint;

////////////////////////////////////////////////////////////

/////////////////////Joint struct///////////////////////////

typedef struct _Joint
{
  int8_t id;
  Eigen::Vector3d axis;
  double coefficient; //actuator angle to joint angle
  double value;
  double velocity;
  double acceleration;
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
} Component;

////////////////////////////////////////////////////////////

typedef struct _Manipulator
{
  int8_t dof;
  World world;
  std::map<Name, Component> component;
  std::map<Name, Component>::iterator it_component;
} Manipulator;

typedef struct _ManipulationTime
{
  double move_time;
  double control_time;
  double present_time;
  double start_time;
}ManipulationTime;

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

}


#endif // ROBOTIS_MANIPULATOR_COMMON_H

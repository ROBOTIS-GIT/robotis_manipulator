#ifndef ROBOTIS_MANIPULATOR_COMMON_H
#define ROBOTIS_MANIPULATOR_COMMON_H

#include <eigen3/Eigen/Eigen>

using namespace Eigen;

typedef int8_t Name;

typedef struct
{
  Vector3f position;
  Matrix3f orientation;
} Pose;

typedef struct
{
  VectorXf velocity;
  VectorXf acceleration;
} State;

typedef struct
{
  int8_t id;
  Vector3f axis;
  double coefficient; //actuator angle to joint angle
  double angle;
  double velocity;
  double acceleration;
} Joint;

typedef struct
{
  int8_t id;
  bool on_off;
  double coefficient; //actuator value to tool value
  double value;       //m or rad
} Tool;

typedef struct
{
  double mass;
  Matrix3f inertia_tensor;
  Vector3f center_of_mass;
} Inertia;

typedef struct
{
  Name name;
  Name child;
  Pose pose;
  State origin;
} World;

typedef struct
{
  Name parent;
  std::vector<Name> child;
  Pose relative_to_parent;
  Pose pose_to_world;
  State origin;
  Joint joint;
  Tool tool;
  Inertia inertia;
} Component;

typedef struct
{
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> acceleration;
  Pose pose;
  Pose pose_vel;
  Pose pose_acc;
} Goal;


#endif // ROBOTIS_MANIPULATOR_COMMON_H

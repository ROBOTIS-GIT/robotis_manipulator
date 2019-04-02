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
 * @file robotis_manipulator_math.h
 * @brief
 * @details
 */

#ifndef ROBOTIS_MANIPULATOR_MATH_H_
#define ROBOTIS_MANIPULATOR_MATH_H_

#include <unistd.h>

#if defined(__OPENCR__)
  #include <Eigen.h>  // Calls main Eigen matrix class library
  #include <Eigen/LU> // Calls inverse, determinant, LU decomp., etc.
  #include <Eigen/Geometry>
#else
  #include <eigen3/Eigen/Eigen>
  #include <eigen3/Eigen/LU>
#endif

#include <math.h>

#define DEG2RAD 0.01745329252 //(M_PI / 180.0)
#define RAD2DEG 57.2957795131 //(180.0 / M_PI)

namespace robotis_manipulator
{

namespace math {

/*****************************************************************************
** Make a Vector or Matrix
*****************************************************************************/
/**
 * @brief vector3
 * @param v1
 * @param v2
 * @param v3
 * @return
 */
Eigen::Vector3d vector3(double v1, double v2, double v3);
/**
 * @brief matrix3
 * @param m11
 * @param m12
 * @param m13
 * @param m21
 * @param m22
 * @param m23
 * @param m31
 * @param m32
 * @param m33
 * @return
 */
Eigen::Matrix3d matrix3(double m11, double m12, double m13,
                        double m21, double m22, double m23,
                        double m31, double m32, double m33);
/**
 * @brief inertiaMatrix
 * @param ixx
 * @param ixy
 * @param ixz
 * @param iyy
 * @param iyz
 * @param izz
 * @return
 */
Eigen::Matrix3d inertiaMatrix(double ixx, double ixy, double ixz , double iyy , double iyz, double izz);


/*****************************************************************************
** Convert
*****************************************************************************/
// Translation Vector
/**
 * @brief convertXYZToVector
 * @param x
 * @param y
 * @param z
 * @return
 */
Eigen::Vector3d convertXYZToVector(double x, double y, double z);

// Rotation 
/**
 * @brief convertRollAngleToRotationMatrix
 * @param angle
 * @return
 */
Eigen::Matrix3d convertRollAngleToRotationMatrix(double angle);
/**
 * @brief convertPitchAngleToRotationMatrix
 * @param angle
 * @return
 */
Eigen::Matrix3d convertPitchAngleToRotationMatrix(double angle);
/**
 * @brief convertYawAngleToRotationMatrix
 * @param angle
 * @return
 */
Eigen::Matrix3d convertYawAngleToRotationMatrix(double angle);
/**
 * @brief convertRotationMatrixToRPYVector
 * @param rotation_matrix
 * @return
 */
Eigen::Vector3d convertRotationMatrixToRPYVector(const Eigen::Matrix3d& rotation_matrix);
/**
 * @brief convertRPYToRotationMatrix
 * @param roll
 * @param pitch
 * @param yaw
 * @return
 */
Eigen::Matrix3d convertRPYToRotationMatrix(double roll, double pitch, double yaw);
/**
 * @brief convertRPYToQuaternion
 * @param roll
 * @param pitch
 * @param yaw
 * @return
 */
Eigen::Quaterniond convertRPYToQuaternion(double roll, double pitch, double yaw);
/**
 * @brief convertRotationMatrixToQuaternion
 * @param rotation_matrix
 * @return
 */
Eigen::Quaterniond convertRotationMatrixToQuaternion(const Eigen::Matrix3d& rotation_matrix);
/**
 * @brief convertQuaternionToRPYVector
 * @param quaternion
 * @return
 */
Eigen::Vector3d convertQuaternionToRPYVector(const Eigen::Quaterniond& quaternion);
/**
 * @brief convertQuaternionToRotationMatrix
 * @param quaternion
 * @return
 */
Eigen::Matrix3d convertQuaternionToRotationMatrix(const Eigen::Quaterniond& quaternion);
/**
 * @brief convertRotationMatrixToOmega
 * @param rotation_matrix
 * @return
 */
Eigen::Vector3d convertRotationMatrixToOmega(const Eigen::Matrix3d& rotation_matrix);

// Transformation Matrix
/**
 * @brief convertXYZRPYToTransformationMatrix
 * @param x
 * @param y
 * @param z
 * @param roll
 * @param pitch
 * @param yaw
 * @return
 */
Eigen::Matrix4d convertXYZRPYToTransformationMatrix(double x, double y, double z , double roll, double pitch, double yaw);
/**
 * @brief convertXYZToTransformationMatrix
 * @param x
 * @param y
 * @param z
 * @return
 */
Eigen::Matrix4d convertXYZToTransformationMatrix(double x, double y, double z);
/**
 * @brief convertRPYToTransformationMatrix
 * @param roll
 * @param pitch
 * @param yaw
 * @return
 */
Eigen::Matrix4d convertRPYToTransformationMatrix(double roll, double pitch, double yaw);

// Dynamic Value
/**
 * @brief convertOmegaToRPYVelocity
 * @param rpy_vector
 * @param omega
 * @return
 */
Eigen::Vector3d convertOmegaToRPYVelocity(Eigen::Vector3d rpy_vector, Eigen::Vector3d omega);
/**
 * @brief convertRPYVelocityToOmega
 * @param rpy_vector
 * @param rpy_velocity
 * @return
 */
Eigen::Vector3d convertRPYVelocityToOmega(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity);
/**
 * @brief convertOmegaDotToRPYAcceleration
 * @param rpy_vector
 * @param rpy_velocity
 * @param omega_dot
 * @return
 */
Eigen::Vector3d convertOmegaDotToRPYAcceleration(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity, Eigen::Vector3d omega_dot);
/**
 * @brief convertRPYAccelerationToOmegaDot
 * @param rpy_vector
 * @param rpy_velocity
 * @param rpy_acceleration
 * @return
 */
Eigen::Vector3d convertRPYAccelerationToOmegaDot(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity, Eigen::Vector3d rpy_acceleration);


/*****************************************************************************
** Math
*****************************************************************************/
/**
 * @brief sign
 * @param value
 * @return
 */
double sign(double value);

/**
 * @brief inverseTransformationMatrix
 * @param transformation_matrix
 * @return
 */
Eigen::Matrix4d inverseTransformationMatrix(const Eigen::MatrixXd& transformation_matrix);
/**
 * @brief matrixLogarithm
 * @param rotation_matrix
 * @return
 */
Eigen::Vector3d matrixLogarithm(Eigen::Matrix3d rotation_matrix);
/**
 * @brief skewSymmetricMatrix
 * @param v
 * @return
 */
Eigen::Matrix3d skewSymmetricMatrix(Eigen::Vector3d v);
/**
 * @brief rodriguesRotationMatrix
 * @param axis
 * @param angle
 * @return
 */
Eigen::Matrix3d rodriguesRotationMatrix(Eigen::Vector3d axis, double angle);

/**
 * @brief positionDifference
 * @param desired_position
 * @param present_position
 * @return
 */
Eigen::Vector3d positionDifference(Eigen::Vector3d desired_position, Eigen::Vector3d present_position);
/**
 * @brief orientationDifference
 * @param desired_orientation
 * @param present_orientation
 * @return
 */
Eigen::Vector3d orientationDifference(Eigen::Matrix3d desired_orientation, Eigen::Matrix3d present_orientation);
/**
 * @brief poseDifference
 * @param desired_position
 * @param present_position
 * @param desired_orientation
 * @param present_orientation
 * @return
 */
Eigen::VectorXd poseDifference(Eigen::Vector3d desired_position, Eigen::Vector3d present_position,
                        Eigen::Matrix3d desired_orientation, Eigen::Matrix3d present_orientation);

} // math
} // namespace robotis_manipulator

#endif // ROBOTIS_MANIPULATOR_MATH_H_

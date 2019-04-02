/**
 * @mainpage Robotis Manipulator
 * @brief Robotis Manipulator is a library for controlling the manipulator provided by ROBOTIS.
 * @details This library provides a manipulator class for setting manipulator parameters, and provides some math functions to configure the manipulator controller and a basic trajectory generators that uses minimum jerk.
 * The user makes a class inheriting RobotisManipulator class, and set up the class by using the provided functions and the vurtual classes.
 * The class provides functions such as creating the trajectory, receiving joint positions from the actuators and sending the target positions to the actuators.
 * The open_manipulator_libs package can be refer as an example.
 * @authors Hye-Jong KIM <hjkim@robotis.com>, Darby Lim <thlim@robotis.com>, Yong-Ho Na <yhna@robotis.com>, Ryan Shim <jhshim@robotis.com>
 */

#include "../include/robotis_manipulator/robotis_manipulator.h"

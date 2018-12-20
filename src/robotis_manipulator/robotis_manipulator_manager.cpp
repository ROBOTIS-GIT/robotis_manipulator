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

#include "../../include/robotis_manipulator/robotis_manipulator_manager.h"

using namespace ROBOTIS_MANIPULATOR;

bool Kinematics::forwardDynamics(Manipulator *manipulator)
{
  forwardKinematics(manipulator);
  return true;
}
bool Kinematics::inverseDynamics(Manipulator *manipulator, Name tool_name, PoseValue target_pose, std::vector<JointValue> *active_joint_way_point)
{
  std::vector<double> goal_joint_position;
  if(!inverseKinematics(manipulator, tool_name, target_pose.kinematic, &goal_joint_position))
    return false;
  for(uint8_t index = 0; index < active_joint_way_point->size(); index++)
  {
    active_joint_way_point->at(index).position = goal_joint_position.at(index);
    active_joint_way_point->at(index).velocity = 0.0;
    active_joint_way_point->at(index).acceleration = 0.0;
    active_joint_way_point->at(index).effort = 0.0;
  }
  return true;
}

bool JointActuator::findId(uint8_t actuator_id)
{
  std::vector<uint8_t> id = getId();
  for(uint32_t index = 0; index < id.size(); index++)
  {
    if(id.at(index) == actuator_id)
      return true;
  }
  return false;
}

bool JointActuator::isEnabled()
{
  return enable_state_;
}


bool ToolActuator::findId(uint8_t actuator_id)
{
  if(getId() == actuator_id)
  {
    return true;
  }
  return false;
}

bool ToolActuator::isEnabled()
{
  return enable_state_;
}



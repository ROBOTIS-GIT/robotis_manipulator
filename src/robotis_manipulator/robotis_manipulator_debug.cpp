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

#include "../../include/robotis_manipulator/robotis_manipulator_debug.h"

using namespace ROBOTIS_MANIPULATOR;

void DEBUG::OpenDEBUGSerialPortInOpenCR()
{
#if defined(__OPENCR__)
  DEBUG.begin(57600);
#endif
}
void DEBUG::LOG(STRING str)
{
#if defined(__OPENCR__)
  DEBUG.println(str);
#else
  printf("%s\n", str.c_str());
#endif
}
void DEBUG::LOG(STRING str, double data)
{
#if defined(__OPENCR__)
  DEBUG.print(str);
  DEBUG.println(data, 3);
#else
  printf("%s %.3lf\n", str.c_str(), data);
#endif
}
void DEBUG::LOG(const char* str)
{
#if defined(__OPENCR__)
  DEBUG.println(str);
#else
  printf("%s\n", str);
#endif
}
void DEBUG::LOG(const char* str, double data)
{
#if defined(__OPENCR__)
  DEBUG.print(str);
  DEBUG.println(data, 3);
#else
  printf("%s %.3lf\n", str, data);
#endif
}
void DEBUG::INFO(STRING str)
{
#if defined(__OPENCR__)
  DEBUG.print("[INFO] ");
  DEBUG.println(str);
#else
  printf("[INFO] %s\n", str.c_str());
#endif
}
void DEBUG::INFO(STRING str, double data)
{
#if defined(__OPENCR__)
  DEBUG.print("[INFO] ");
  DEBUG.print(str);
  DEBUG.println(data, 3);
#else
  printf("[INFO] %s %.3lf\n", str.c_str(), data);
#endif
}
void DEBUG::INFO(const char* str)
{
#if defined(__OPENCR__)
  DEBUG.print("[INFO] ");
  DEBUG.println(str);
#else
  printf("[INFO] %s\n", str);
#endif
}
void DEBUG::INFO(const char* str, double data)
{
#if defined(__OPENCR__)
  DEBUG.print("[INFO] ");
  DEBUG.print(str);
  DEBUG.println(data, 3);
#else
  printf("[INFO] %s %.3lf\n", str, data);
#endif
}
void DEBUG::WARN(STRING str)
{
#if defined(__OPENCR__)
  DEBUG.print("[WARN] ");
  DEBUG.println(str);
#else
  printf(ANSI_COLOR_YELLOW);
  printf("[WARN] %s\n", str.c_str());
  printf(ANSI_COLOR_RESET);
#endif
}
void DEBUG::WARN(STRING str, double data)
{
#if defined(__OPENCR__)
  DEBUG.print("[WARN] ");
  DEBUG.print(str);
  DEBUG.println(data, 3);
#else
  printf(ANSI_COLOR_YELLOW);
  printf("[WARN] %s %.3lf\n",str.c_str(), data);
  printf(ANSI_COLOR_RESET);
#endif
}
void DEBUG::WARN(const char* str)
{
#if defined(__OPENCR__)
  DEBUG.print("[WARN] ");
  DEBUG.println(str);
#else
  printf(ANSI_COLOR_YELLOW);
  printf("[WARN] %s\n", str);
  printf(ANSI_COLOR_RESET);
#endif
}
void DEBUG::WARN(const char* str, double data)
{
#if defined(__OPENCR__)
  DEBUG.print("[WARN] ");
  DEBUG.print(str);
  DEBUG.println(data, 3);
#else
  printf(ANSI_COLOR_YELLOW);
  printf("[WARN] %s %.3lf\n",str, data);
  printf(ANSI_COLOR_RESET);
#endif
}
void DEBUG::ERROR(STRING str)
{
#if defined(__OPENCR__)
  DEBUG.print("[ERROR] ");
  DEBUG.println(str);
#else
  printf(ANSI_COLOR_RED);
  printf("[ERROR] %s\n", str.c_str());
  printf(ANSI_COLOR_RESET);
#endif
}
void DEBUG::ERROR(STRING str, double data)
{
#if defined(__OPENCR__)
  DEBUG.print("[ERROR] ");
  DEBUG.print(str);
  DEBUG.println(data, 3);
#else
  printf(ANSI_COLOR_RED);
  printf("[ERROR] %s %.3lf\n",str.c_str(), data);
  printf(ANSI_COLOR_RESET);
#endif
}

void DEBUG::ERROR(const char* str)
{
#if defined(__OPENCR__)
  DEBUG.print("[ERROR] ");
  DEBUG.println(str);
#else
  printf(ANSI_COLOR_RED);
  printf("[ERROR] %s\n", str);
  printf(ANSI_COLOR_RESET);
#endif
}
void DEBUG::ERROR(const char* str, double data)
{
#if defined(__OPENCR__)
  DEBUG.print("[ERROR] ");
  DEBUG.print(str);
  DEBUG.println(data, 3);
#else
  printf(ANSI_COLOR_RED);
  printf("[ERROR] %s %.3lf\n",str, data);
  printf(ANSI_COLOR_RESET);
#endif
}


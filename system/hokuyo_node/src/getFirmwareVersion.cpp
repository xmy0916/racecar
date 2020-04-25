/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "hokuyo_node/hokuyo.h"
#include <stdio.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

using namespace std;


int
main(int argc, char** argv)
{
  if (argc < 2 || argc > 3)
  {
    fprintf(stderr, "usage: getFirmwareVersion /dev/ttyACM? [quiet]\nOutputs the firmware version of a hokuyo at /dev/ttyACM?. Add a second argument for script friendly output.\n");
    return 1;
  }
  
  bool verbose = (argc == 2);
  
  if (!verbose)
  {
    // In quiet mode we want to turn off logging levels that go to stdout.
    log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Error]);
    ros::console::notifyLoggerLevelsChanged();
  }

  hokuyo::Laser laser;
  
  for (int retries = 10; retries; retries--)
  {
    try 
    {
      laser.open(argv[1]);
      std::string firmware_version = laser.getFirmwareVersion();
      if (verbose)
        printf("Device at %s has FW rev ", argv[1]);
      printf("%s\n", firmware_version.c_str());
      laser.close();
      return 0;
    }
    catch (hokuyo::Exception &e)
    {
      if (verbose)
        ROS_WARN("getFirmwareVersion failed: %s", e.what());
      laser.close();
    }
    sleep(1);
  }

  if (verbose)
    ROS_ERROR("getFirmwareVersion failed for 10 seconds. Giving up.");
  return 1;
}

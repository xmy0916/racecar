/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008-2010, Willow Garage, Inc.
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

#include "driver_base/driver.h"
#include "driver_base/driver_node.h"
#include <diagnostic_updater/publisher.h>

#include <assert.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"

#include "hokuyo_node/HokuyoConfig.h"

#include "hokuyo_node/hokuyo.h"

using namespace std;

class HokuyoDriver : public driver_base::Driver
{
  friend class HokuyoNode;

  typedef boost::function<void(const hokuyo::LaserScan &)> UseScanFunction;
  UseScanFunction useScan_;

  boost::shared_ptr<boost::thread> scan_thread_;

  std::string device_status_;
  std::string device_id_;
  std::string last_seen_device_id_;
  
  bool first_scan_;

  std::string vendor_name_;
  std::string product_name_;
  std::string protocol_version_;
  std::string firmware_version_;
  
  std::string connect_fail_;
  
  hokuyo::LaserScan  scan_;
  hokuyo::Laser laser_;
  hokuyo::LaserConfig laser_config_;

  bool calibrated_;
  int lost_scan_thread_count_;
  int corrupted_scan_count_;

public:
  hokuyo_node::HokuyoConfig config_;
  typedef hokuyo_node::HokuyoConfig Config;

  HokuyoDriver()
  {
    calibrated_ = false;
    lost_scan_thread_count_ = 0;
    corrupted_scan_count_ = 0;
  }

  bool checkAngleRange(Config &conf)
  {
    bool changed = false;

    if (conf.min_ang < laser_config_.min_angle)
    {
      changed = true;
      if (laser_config_.min_angle - conf.min_ang > 1e-10)  /// @todo Avoids warning when restarting node pending ros#2353 getting fixed.
      {
        ROS_WARN("Requested angle (%f rad) out of range, using minimum scan angle supported by device: %f rad.", 
            conf.min_ang, laser_config_.min_angle);
      }
      conf.min_ang = laser_config_.min_angle;
    }                                    
    
    double max_safe_angular_range_per_cluster_deg = 95;
    if (firmware_version_ == "1.16.01(16/Nov./2009)")
      max_safe_angular_range_per_cluster_deg = 190;
    
    int real_cluster = conf.cluster == 0 ? 1 : conf.cluster;
    double max_safe_angular_range = (real_cluster * max_safe_angular_range_per_cluster_deg) * M_PI / 180;

    if (conf.intensity && (conf.max_ang - conf.min_ang) > max_safe_angular_range + 1e-8 &&
        !config_.allow_unsafe_settings && laser_.getProductName() ==
            "SOKUIKI Sensor TOP-URG UTM-30LX")
    {
      changed = true;
      conf.max_ang = conf.min_ang + max_safe_angular_range;
      ROS_WARN("More than %f degree/cluster scan range requested on UTM-30LX firmware version %s in intensity mode with cluster=%i. The max_ang was adjusted to limit the range. You may extend the scanner's angular range using the allow_unsafe_settings option, but this may result in incorrect data or laser crashes that will require a power cycle of the laser.", max_safe_angular_range_per_cluster_deg, firmware_version_.c_str(), real_cluster);
    }

    if (conf.max_ang - laser_config_.max_angle > 1e-10)   /// @todo Avoids warning when restarting node pending ros#2353 getting fixed.
    {
      changed = true;
      ROS_WARN("Requested angle (%f rad) out of range, using maximum scan angle supported by device: %f rad.", 
          conf.max_ang, laser_config_.max_angle);
      conf.max_ang = laser_config_.max_angle;
    }
    
    if (conf.min_ang > conf.max_ang)
    {
      changed = true;
      if (conf.max_ang < laser_config_.min_angle)
      {
        if (laser_config_.min_angle - conf.max_ang > 1e-10)  /// @todo Avoids warning when restarting node pending ros#2353 getting fixed.
          ROS_WARN("Requested angle (%f rad) out of range, using minimum scan angle supported by device: %f rad.", 
              conf.max_ang, laser_config_.min_angle);
        conf.max_ang = laser_config_.min_angle;
      }
      ROS_WARN("Minimum angle must be greater than maximum angle. Adjusting min_ang.");
      conf.min_ang = conf.max_ang;
    }                                    
      
    return changed;
  }

  bool checkIntensitySupport(Config &conf)
  {
    if (conf.intensity && !laser_.isIntensitySupported())
    {
      ROS_WARN("This unit does not appear to support intensity mode. Turning intensity off.");
      conf.intensity = false;
      return true;
    }
    return false;
  }

  void doOpen()
  {
    try
    {
      std::string old_device_id = device_id_;
      device_id_ = "unknown";
      device_status_ =  "unknown";
      first_scan_ = true;
      
      laser_.open(config_.port.c_str());
      
      device_id_ = getID();
      vendor_name_ = laser_.getVendorName();
      firmware_version_ = laser_.getFirmwareVersion();
      product_name_ = laser_.getProductName();
      protocol_version_ = laser_.getProtocolVersion();

      device_status_ = laser_.getStatus();
      if (device_status_ != std::string("Sensor works well."))
      {
        doClose();
        setStatusMessagef("Laser returned abnormal status message, aborting: %s You may be able to find further information at http://www.ros.org/wiki/hokuyo_node/Troubleshooting/", device_status_.c_str());
        return;
      }
      
      if (old_device_id != device_id_)
      {
        ROS_INFO("Connected to device with ID: %s", device_id_.c_str());
        
        if (last_seen_device_id_ != device_id_)
        {
          // Recalibrate when the device changes.
          last_seen_device_id_ = device_id_;
          calibrated_ = false;
        }

        // Do this elaborate retry circuis if we were just plugged in.
        for (int retries = 10;; retries--)
          try {
            laser_.laserOn();
            break;
          }
        catch (hokuyo::Exception &e)
        { 
          if (!retries)
            throw e; // After trying for 10 seconds, give up and throw the exception.
          else if (retries == 10)
            ROS_WARN("Could not turn on laser. This may happen just after the device is plugged in. Will retry for 10 seconds.");
          ros::Duration(1).sleep();
        }
      }
      else
        laser_.laserOn(); // Otherwise, it should just work, so no tolerance.

      if (config_.calibrate_time && !calibrated_)
      {
        ROS_INFO("Starting calibration. This will take up a few seconds.");
        double latency = laser_.calcLatency(false && config_.intensity, config_.min_ang, config_.max_ang, config_.cluster, config_.skip) * 1e-9;
        calibrated_ = true; // This is a slow step that we only want to do once.
        ROS_INFO("Calibration finished. Latency is: %0.4f", latency);
      }
      else
      {
        calibrated_ = false;
        laser_.clearLatency();
      }

      setStatusMessage("Device opened successfully.", true);
      laser_.getConfig(laser_config_);
      
      state_ = OPENED;
    } 
    catch (hokuyo::Exception& e) 
    {
      doClose();
      setStatusMessagef("Exception thrown while opening Hokuyo.\n%s", e.what());
      return;
    }
  }

  void doClose()
  {
    try
    {
      laser_.close();
      setStatusMessage("Device closed successfully.", true);
    } catch (hokuyo::Exception& e) {
      setStatusMessagef("Exception thrown while trying to close:\n%s",e.what());
    }
    
    state_ = CLOSED; // If we can't close, we are done for anyways.
  }

  void doStart()
  {
    try
    {
      laser_.laserOn();
      
      int status = laser_.requestScans(config_.intensity, config_.min_ang, config_.max_ang, config_.cluster, config_.skip);

      if (status != 0) {
        setStatusMessagef("Failed to request scans from device.  Status: %d.", status);
        corrupted_scan_count_++;
        return;
      }
    
      setStatusMessagef("Waiting for first scan.", true);
      state_ = RUNNING;
      scan_thread_.reset(new boost::thread(boost::bind(&HokuyoDriver::scanThread, this)));
    } 
    catch (hokuyo::Exception& e) 
    {
      doClose();
      setStatusMessagef("Exception thrown while starting Hokuyo.\n%s", e.what());
      connect_fail_ = e.what();
      return;
    }
  }

  void doStop()
  {
    if (state_ != RUNNING) // RUNNING can exit asynchronously.
      return;

    state_ = OPENED;

    if (scan_thread_ && !scan_thread_->timed_join((boost::posix_time::milliseconds) 2000))
    {
      ROS_ERROR("scan_thread_ did not die after two seconds. Pretending that it did. This is probably a bad sign.");
      lost_scan_thread_count_++;
    }
    scan_thread_.reset();
    
    setStatusMessagef("Stopped.", true);
  }

  virtual std::string getID()
  {
    std::string id = laser_.getID();
    if (id == std::string("H0000000"))
      return "unknown";
    return id;
  }

  void config_update(Config &new_config, int level = 0)
  {
    ROS_DEBUG("Reconfigure called from state %i", state_);
    
    if (state_ == OPENED) 
      // If it is closed, we don't know what to check for. If it is running those parameters haven't changed,
      // and talking to the hokuyo would cause loads of trouble.
    {
      checkIntensitySupport(new_config);
      checkAngleRange(new_config);
    }

    config_ = new_config;
  }

  void scanThread()
  {
    while (state_ == RUNNING)
    {
      try
      {
        int status = laser_.serviceScan(scan_);

        if(status != 0)
        {
          ROS_WARN("Error getting scan: %d", status);
          break;
        }
      } catch (hokuyo::CorruptedDataException &e) {
        ROS_WARN("Skipping corrupted data");
        continue;
      } catch (hokuyo::Exception& e) {
        ROS_WARN("Exception thrown while trying to get scan.\n%s", e.what());
        doClose();
        return;
      }

      if (first_scan_)
      {
        first_scan_ = false;
        setStatusMessage("Streaming data.", true, true);
        ROS_INFO("Streaming data.");
      }

      useScan_(scan_);
    }

    try
    {
      laser_.stopScanning(); // This actually just calls laser Off internally.
    } catch (hokuyo::Exception &e)
    {
        ROS_WARN("Exception thrown while trying to stop scan.\n%s", e.what());
    }
    state_ = OPENED;
  }
};

class HokuyoNode : public driver_base::DriverNode<HokuyoDriver>
{
private:   
  string connect_fail_;
  
  double desired_freq_;

  ros::NodeHandle node_handle_;
  diagnostic_updater::DiagnosedPublisher<sensor_msgs::LaserScan> scan_pub_;
  sensor_msgs::LaserScan scan_msg_;
  diagnostic_updater::FunctionDiagnosticTask hokuyo_diagnostic_task_;

public:
  HokuyoNode(ros::NodeHandle &nh) :
    driver_base::DriverNode<HokuyoDriver>(nh),
    node_handle_(nh),
    scan_pub_(node_handle_.advertise<sensor_msgs::LaserScan>("scan", 100),
        diagnostic_,
        diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05),
        diagnostic_updater::TimeStampStatusParam()),
        hokuyo_diagnostic_task_("Hokuyo Diagnostics", boost::bind(&HokuyoNode::connectionStatus, this, _1))
  {
    desired_freq_ = 0;
    driver_.useScan_ = boost::bind(&HokuyoNode::publishScan, this, _1);
    driver_.setPostOpenHook(boost::bind(&HokuyoNode::postOpenHook, this));

    // Check if use_rep_117 is set.
    std::string key;
    if (nh.searchParam("use_rep_117", key))
    {
      ROS_ERROR("The use_rep_117 parameter has not been specified and is now ignored.  This parameter was removed in Hydromedusa.  Please see: http://ros.org/wiki/rep_117/migration");
    }
  }

  void postOpenHook()
  {
    private_node_handle_.setParam("min_ang_limit", (double) (driver_.laser_config_.min_angle));
    private_node_handle_.setParam("max_ang_limit", (double) (driver_.laser_config_.max_angle));
    private_node_handle_.setParam("min_range", (double) (driver_.laser_config_.min_range));
    private_node_handle_.setParam("max_range", (double) (driver_.laser_config_.max_range));

    diagnostic_.setHardwareID(driver_.getID());

    if (driver_.checkIntensitySupport(driver_.config_) || 
        driver_.checkAngleRange(driver_.config_)) // Might have been set before the device's range was known.
      reconfigure_server_.updateConfig(driver_.config_);
    
    scan_pub_.clear_window(); // Reduce glitches in the frequency diagnostic.
  }

  virtual void addOpenedTests()
  {
    self_test_.add( "Status Test", this, &HokuyoNode::statusTest );
    self_test_.add( "Laser Test", this, &HokuyoNode::laserTest );
    self_test_.add( "Polled Data Test", this, &HokuyoNode::polledDataTest );
    self_test_.add( "Streamed Data Test", this, &HokuyoNode::streamedDataTest );
    self_test_.add( "Streamed Intensity Data Test", this, &HokuyoNode::streamedIntensityDataTest );
    self_test_.add( "Laser Off Test", this, &HokuyoNode::laserOffTest );
  }

  virtual void addStoppedTests()
  { 
  }

  virtual void addRunningTests()
  { 
  }

  virtual void addDiagnostics()
  {
    driver_status_diagnostic_.addTask(&hokuyo_diagnostic_task_);
  }
  
  void reconfigureHook(int level)
  {
    if (private_node_handle_.hasParam("frameid"))
    {
      ROS_WARN("~frameid is deprecated, please use ~frame_id instead");
      private_node_handle_.getParam("frameid", driver_.config_.frame_id);
    }

    if (private_node_handle_.hasParam("min_ang_degrees"))
    {
      ROS_WARN("~min_ang_degrees is deprecated, please use ~min_ang instead");
      private_node_handle_.getParam("min_ang_degrees", driver_.config_.min_ang);
      driver_.config_.min_ang *= M_PI/180;
    }

    if (private_node_handle_.hasParam("max_ang_degrees"))
    {
      ROS_WARN("~max_ang_degrees is deprecated, please use ~max_ang instead");
      private_node_handle_.getParam("max_ang_degrees", driver_.config_.max_ang);
      driver_.config_.max_ang *= M_PI/180;
    }

    diagnostic_.force_update();   
    
    scan_pub_.clear_window(); // Reduce glitches in the frequency diagnostic.
  }

  int publishScan(const hokuyo::LaserScan &scan)
  {
    //ROS_DEBUG("publishScan");

    scan_msg_.angle_min = scan.config.min_angle;
    scan_msg_.angle_max = scan.config.max_angle;
    scan_msg_.angle_increment = scan.config.ang_increment;
    scan_msg_.time_increment = scan.config.time_increment;
    scan_msg_.scan_time = scan.config.scan_time;
    scan_msg_.range_min = scan.config.min_range;
    scan_msg_.range_max = scan.config.max_range;
    scan_msg_.ranges = scan.ranges;
    scan_msg_.intensities = scan.intensities;
    scan_msg_.header.stamp = ros::Time().fromNSec((uint64_t)scan.system_time_stamp) + ros::Duration(driver_.config_.time_offset);
    scan_msg_.header.frame_id = driver_.config_.frame_id;
  
    desired_freq_ = (1. / scan.config.scan_time);

    scan_pub_.publish(scan_msg_);

    //ROS_DEBUG("publishScan done");

    return(0);
  }

  void connectionStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if (driver_.state_ == driver_.CLOSED)
      status.summary(2, "Not connected. " + driver_.device_status_ + " " + connect_fail_);
    else if (driver_.device_status_ != std::string("Sensor works well."))
      status.summaryf(2, "Abnormal status: %s", driver_.device_status_.c_str());
    else if (driver_.state_ == driver_.RUNNING)
    {
      if (driver_.first_scan_)
        status.summary(0, "Waiting for first scan");
      else
        status.summary(0, "Streaming");
    }
    else if (driver_.state_ == driver_.OPENED)
      status.summary(0, "Open");
    else 
      status.summary(2, "Unknown state");

    status.add("Device Status", driver_.device_status_);
    status.add("Port", driver_.config_.port);
    status.add("Device ID", driver_.device_id_);
    status.add("Scan Thread Lost Count", driver_.lost_scan_thread_count_);
    status.add("Corrupted Scan Count", driver_.corrupted_scan_count_);
    status.add("Vendor Name", driver_.vendor_name_);
    status.add("Product Name", driver_.product_name_);
    status.add("Firmware Version", driver_.firmware_version_);
    status.add("Protocol Version", driver_.protocol_version_);
    status.add("Computed Latency", driver_.laser_.getLatency());
    status.add("User Time Offset", driver_.config_.time_offset);
  }

  void statusTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    std::string stat = driver_.laser_.getStatus();

    if (stat != std::string("Sensor works well."))
    {
      status.level = 2;
    } else {
      status.level = 0;
    }

    status.message = stat;
  }

  void laserTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    driver_.laser_.laserOn();

    status.level = 0;
    status.message = "Laser turned on successfully.";
  }

  void polledDataTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    hokuyo::LaserScan  scan;

    int res = driver_.laser_.pollScan(scan, driver_.laser_config_.min_angle, driver_.laser_config_.max_angle, 1, 1000);

    if (res != 0)
    {
      status.level = 2;
      ostringstream oss;
      oss << "Hokuyo error code: " << res << ". Consult manual for meaning.";
      status.message = oss.str();

    } else {
      status.level = 0;
      status.message = "Polled Hokuyo for data successfully.";
    }
  }

  void streamedDataTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    hokuyo::LaserScan  scan;

    int res = driver_.laser_.requestScans(false, driver_.laser_config_.min_angle, driver_.laser_config_.max_angle, 1, 1, 99, 1000);

    if (res != 0)
    {
      status.level = 2;
      ostringstream oss;
      oss << "Hokuyo error code: " << res << ". Consult manual for meaning.";
      status.message = oss.str();

    } else {

      for (int i = 0; i < 99; i++)
      {
        driver_.laser_.serviceScan(scan, 1000);
      }

      status.level = 0;
      status.message = "Streamed data from Hokuyo successfully.";

    }
  }

  void streamedIntensityDataTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    hokuyo::LaserScan  scan;

    int res = driver_.laser_.requestScans(false, driver_.laser_config_.min_angle, driver_.laser_config_.max_angle, 1, 1, 99, 1000);

    if (res != 0)
    {
      status.level = 2;
      ostringstream oss;
      oss << "Hokuyo error code: " << res << ". Consult manual for meaning.";
      status.message = oss.str();

    } else {

      int corrupted_data = 0;

      for (int i = 0; i < 99; i++)
      {
        try {
          driver_.laser_.serviceScan(scan, 1000);
        } catch (hokuyo::CorruptedDataException &e) {
          corrupted_data++;
        }
      }
      if (corrupted_data == 1)
      {
        status.level = 1;
        status.message = "Single corrupted message.  This is acceptable and unavoidable";
      } else if (corrupted_data > 1)
      {
        status.level = 2;
        ostringstream oss;
        oss << corrupted_data << " corrupted messages.";
        status.message = oss.str();
      } else
      {
        status.level = 0;
        status.message = "Stramed data with intensity from Hokuyo successfully.";
      }
    }
  }

  void laserOffTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    driver_.laser_.laserOff();

    status.level = 0;
    status.message = "Laser turned off successfully.";
  }
};

int main(int argc, char **argv)
{ 
  return driver_base::main<HokuyoNode>(argc, argv, "hokuyo_node");
}


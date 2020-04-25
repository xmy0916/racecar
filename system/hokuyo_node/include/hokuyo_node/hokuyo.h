/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2008-2010  Willow Garage
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* \mainpage hokuyo_node
 *  \htmlinclude manifest.html
 */

#ifndef HOKUYO_HH
#define HOKUYO_HH

#include <stdexcept>
#include <termios.h>
#include <string>
#include <vector>
#include <stdint.h>
#include <limits>

//! A namespace containing the hokuyo device driver
namespace hokuyo
{
  //! The maximum number of readings that can be returned from a hokuyo
  const uint32_t MAX_READINGS = 1128;

  //! The maximum length a command will ever be
  const int MAX_CMD_LEN = 100;

  //! The maximum number of bytes that should be skipped when looking for a response
  const int MAX_SKIPPED = 1000000;
  
  //! Macro for defining an exception with a given parent (std::runtime_error should be top parent)
#define DEF_EXCEPTION(name, parent) \
  class name  : public parent { \
  public: \
    name(const char* msg) : parent(msg) {} \
  }\
  
  //! A standard Hokuyo exception
  DEF_EXCEPTION(Exception, std::runtime_error);

  //! An exception for use when a timeout is exceeded
  DEF_EXCEPTION(TimeoutException, Exception);

  //! An exception for use when data is corrupted
  DEF_EXCEPTION(CorruptedDataException, Exception);

  //! An exception for use when the timestamp on the data is repeating (a good indicator something has gone wrong)
  DEF_EXCEPTION(RepeatedTimeException, Exception);

#undef DEF_EXCEPTION

  //! A struct for returning configuration from the Hokuyo
  struct LaserConfig
  {
    //! Start angle for the laser scan [rad].  0 is forward and angles are measured clockwise when viewing hokuyo from the top.
    float min_angle;
    //! Stop angle for the laser scan [rad].   0 is forward and angles are measured clockwise when viewing hokuyo from the top.
    float max_angle;
    //! Scan resolution [rad].
    float ang_increment;
    //! Scan resoltuion [s]
    float time_increment;
    //! Time between scans
    float scan_time;
    //! Minimum range [m]
    float min_range;
    //! Maximum range [m]
    float max_range;
    //! Range Resolution [m]
    float range_res;
  };


  //! A struct for returning laser readings from the Hokuyo
  struct LaserScan
  {
    //! Array of ranges
    std::vector<float> ranges;
    //! Array of intensities
    std::vector<float> intensities;
    //! Self reported time stamp in nanoseconds
    uint64_t self_time_stamp;
    //! System time when first range was measured in nanoseconds
    uint64_t system_time_stamp;
    //! Configuration of scan
    LaserConfig config;
  };


  //! A class for interfacing to an SCIP2.0-compliant Hokuyo laser device
  /*!
   * Almost all methods of this class may throw an exception of type
   * hokuyo::Exception in the event of an error when communicating
   * with the device.  This primarily include read/write problems
   * (hokuyo::Exception), communication time-outs
   * (hokuyo:TimeoutException), and corrupted data
   * (hokuyo::CorruptedDataException).  However, many methods which
   * wrap commands that are sent to the hokuyo will return a
   * hokuyo-supplied status code as well.  These are not
   * "exceptional," in that the device is operating correctly from a
   * communication standpoint, although the return code may still
   * indicate that the device has undergone some sort of failure.  In
   * these cases, return codes of 0 are normal operation.  For more
   * information on the possible device error codes, consult the
   * hokuyo manual.
   */
  class Laser
  {
  public:
    //! Constructor
    Laser();

    //! Destructor
    ~Laser();
  
    //! Open the port
    /*! 
     * This must be done before the hokuyo can be used. This call essentially
     * wraps fopen, with some additional calls to tcsetattr.
     * 
     * \param port_name   A character array containing the name of the port
     *
     */
    void open(const char * port_name);

    //! Close the port
    /*!
     * This call essentiall wraps fclose.
     */
    void close();

    //! Reset the laser to its power on state.
    /*!
     * This calls TM2 and RS. Exceptions raised by TM2 are ignored as they
     * are normal in the case where the laser is scanning. Exceptions
     * raised by RS are passed back to the caller.
     */
    void reset();
  
    //! Check whether the port is open
    bool portOpen() {  return laser_fd_ != -1; }

    //! Sends an SCIP2.0 command to the hokuyo device
	// sets up model 04LX to work in SCIP 2.0 mode
    void setToSCIP2();

    /*!
     *  This function allows commands to be sent directly to the hokuyo.
     *  Possible commands can be found in the hokuyo documentation.  It
     *  will only read up until the end of the status code.  Any
     *  additional bytes will be left on the line for parsing.
     *
     *  \param cmd    Command and arguments in a character array
     *  \param timeout Timeout in milliseconds.
     * 
     *  \return Status code returned from hokuyo device.
     */
    int sendCmd(const char* cmd, int timeout = -1);

    //! Retrieve the native hokuyo configuration.
    /*!
     * This function will retrieve the native configuration for the laser.
     * In particular, this specifies the smallest possible min, and largest
     * possible maximum values that can be used.
     *
     * \param config     A reference to the LaserConfig which will be populated
     *
     * \return Status code returned from hokuyo device.
     */
    void getConfig(LaserConfig& config);


    //! Retrieve a single scan from the hokuyo
    /*!
     * \param scan       A reference to the LaserScan which will be populated
     * \param min_ang    Minimal angle of the scan
     * \param max_ang    Maximum angle of the scan
     * \param clustering Number of adjascent ranges to be clustered into a single measurement.
     * \param timeout    Timeout in milliseconds.
     *
     * \return Status code returned from hokuyo device.
     */
    int pollScan(LaserScan& scan, double min_ang, double max_ang, int clustering = 0, int timeout = -1);

    //! Request a sequence of scans from the hokuyo
    /*!
     *  This function will request a sequence of scans from the hokuyo.  If
     *  indefinite scans are requested, stop_scanning() must be called
     *  to terminate scanning.  Service_scan() must be called repeatedly
     *  to receive scans as they come in.
     *
     * \param intensity  Whether or not intensity data should be provided
     * \param min_ang    Minimal angle of the scan (radians)
     * \param max_ang    Maximum angle of the scan (radians)
     * \param clustering Number of adjascent ranges to be clustered into a single measurement
     * \param skip       Number of scans to skip between returning measurements
     * \param num        Number of scans to return (0 for indefinite)
     * \param timeout    Timeout in milliseconds.
     *
     * \return Status code returned from hokuyo device.
     */
    int requestScans(bool intensity, double min_ang, double max_ang, int clustering = 0, int skip = 0, int num = 0, int timeout = -1);

    //! Retrieve a scan if they have been requested
    /*!
     * \param scan       A reference to the LaserScan which will be populated.
     * \param timeout    Timeout in milliseconds.
     *
     * \return 0 on succes, Status code returned from hokuyo device on failure.  (Normally 99 is used for success)
     */
    int serviceScan(LaserScan& scan, int timeout = -1);

    //! Turn the laser off
    /*!
     * \return Status code returned from hokuyo device.
     */
    int laserOff();

    //! Turn the laser on
    /*!
     * \return Status code returned from hokuyo device.
     */
    int laserOn();

    //! Stop retrieving scans
    /*!
     * \return Status code returned from hokuyo device.
     */
    int stopScanning();

    //! Return hokuyo serial number
    /*!
     * \return  Serial number of hokuyo.
     */
    std::string getID();

    //! Get status message from the hokuyo
    /*!
     * \return  Status message
     */
    std::string getStatus();

    //! Compute latency between actual hokuyo readings and system time
    /*!
     *  This function will estimate the latency between when the data
     *  is actually acquired by the hokuyo and a read call returns from
     *  the hokuyo with the data.  This latency is stored in an offset
     *  variable and applied to the measurement of system time which
     *  occurs immediately after a read.  This means that the
     *  system_time_stamp variable on LaserScan struct is corrected
     *  and accurately reflects the exact time of the hokuyo data in
     *  computer system time.  This function takes the same arguments
     *  as a call to request scans, since this latency may be
     *  parameter dependent.  NOTE: This method will take
     *  approximately 10 seconds to return.
     *
     * \param intensity  Whether or not intensity data should be provided
     * \param min_ang    Minimal angle of the scan (radians)
     * \param max_ang    Maximum angle of the scan (radians)
     * \param clustering Number of adjascent ranges to be clustered into a single measurement
     * \param skip       Number of scans to skip between returning measurements
     * \param num        Number of times to repeat the measurement 0 for auto.
     * \param timeout    Timeout in milliseconds.
     *
     * \return Median latency in nanoseconds
     */
    long long calcLatency(bool intensity, double min_ang, double max_ang, int clustering = 0, int skip = 0, int num = 0, int timeout = -1);

    //! This method clears the offset that was computed with calcLatency.
    void clearLatency()
    {
      offset_ = 0;
    }

    //! Returns the latency that is in use.
    //! \return Latency in nanoseconds.
    long long getLatency()
    {
      return offset_;
    }

    //! Return firmware version
    /*!
     * \return  Firmware version of hokuyo.
     */
    std::string getFirmwareVersion();

    //! Return hokuyo vendor name
    /*!
     * \return  Vendor name of hokuyo.
     */
    std::string getVendorName();

    //! Return hokuyo product name
    /*!
     * \return  Product name of hokuyo.
     */
    std::string getProductName();

    //! Return hokuyo protocol version
    /*!
     * \return Protocol version of hokuyo.
     */
    std::string getProtocolVersion();

    //! Tries to determine if intensity mode is supported
    /*!
     * \return True if intensity mode is supported, false if it is not.
     * hokuyo::Exception if no scan could be read even out of intensity
     * mode.
     */
    bool isIntensitySupported();

    //! Call VV and store the returned version information
    //! Use other methods to read the information.
    void queryVersionInformation();
  
    //! Wrapper around fputs
    int laserWrite(const char* msg);

    //! Read a full line from the hokuyo using fgets
    int laserReadline(char *buf, int len, int timeout = -1);

    //! Wrapper around tcflush
    int laserFlush();

    //! Read in a time value
    uint64_t readTime(int timeout = -1);

  private:
    //! Use the TM command to determine the offset between the PC clock and
    //the Hokuyo clock. Returns the median of reps measurements.
    long long int getHokuyoClockOffset(int reps, int timeout = -1);

    //! Get reps scans and returns the median difference between the system
    // timestamp (PC) and the scan timestamp (Hokuyo).
    long long int getHokuyoScanStampToSystemStampOffset(bool intensity, double min_ang, double max_ang, int clustering, int skip, int reps, int timeout = -1);
     
    //! Query the sensor configuration of the hokuyo
    void querySensorConfig();

    //! Search for a particular sequence and then read the rest of the line
    char* laserReadlineAfter(char *buf, int len, const char *str, int timeout = -1);

    //! Compute the checksum of a given buffer
    bool checkSum(const char* buf, int buf_len);

    //! Read in a scan
    void readData(LaserScan& scan, bool has_intensity, int timout = -1);

    int dmin_;
    int dmax_;
    int ares_;
    int amin_;
    int amax_;
    int afrt_;
    int rate_;

    int wrapped_;

    unsigned int last_time_;

    unsigned int time_repeat_count_;

    long long offset_;

    int laser_fd_;

    std::string vendor_name_;
    std::string product_name_;
    std::string serial_number_;
    std::string protocol_version_;
    std::string firmware_version_;

    char read_buf[256];
    int read_buf_start;
    int read_buf_end;
  };

}

#endif

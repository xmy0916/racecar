^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hokuyo_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.7.8 (2014-01-15)
------------------
* Merge pull request `#8 <https://github.com/ros-drivers/hokuyo_node/issues/8>`_ from trainman419/hydro-devel
  Use rosconsole to set logger levels. Fixes `#7 <https://github.com/ros-drivers/hokuyo_node/issues/7>`_
* Use rosconsole to set logger levels. Fixes `#7 <https://github.com/ros-drivers/hokuyo_node/issues/7>`_
* Added changelog
* Contributors: Chad Rockey, chadrockey, trainman419

1.7.7 (2013-07-17)
------------------
* fixing value interpretation for smaller lasers (04LX)

1.7.6 (2013-04-25)
------------------
* Removed REP 117 Tick/Tock for Hydro.

1.7.5 (2013-03-26)
------------------
* Added install targets for various files.
* Added install targets.
* Fixed two compile errors on OS X with clang
* Fixed dynamic_reconfigure build bug and refactor
* Missing include file.
* Catkinizing
* Tock on REP 117.  Will now default to True.  Warns if not set or false.
* Had to actually add boost directors and link to boost::thread in the CMakeLists
* Removed call to directory with no CMakeLists.txt
* explicitly link log4cxx
* Updated hokuyo node to add the use_rep_117 parameter and output error cases accordingly.
* Added a launch file to help test whether hokuyo unplug causes SIGHUP.
* Now only polls when there is no buffered data. Otherwise could lock up. Now only checks timeout once in read loop. Now detects EOF. Verified that no sighup on unplug. Now just need to test on Mac. Should fix `#3826 <https://github.com/ros-drivers/hokuyo_node/issues/3826>`_.
* Tweaked the tty settings to try to get hokuyo_node working on Mac. Seems to be working. Now need to test that I haven't broken anything.
* Now IO to hokuyo is purely file descriptor based. Should help with `#3826 <https://github.com/ros-drivers/hokuyo_node/issues/3826>`_.
* Changed open with fopen in the hokuyo library. This allow the NO_CTTY and O_NONBLOCK flags to be set at open time, which simplifies the code a bit. Still need to verify that CTTY actually works.
* Added Ubuntu platform tags to manifest
* Fixed linker export in manifest.
* Now blocks SIGHUP if there isn't a handler for it.
* Updated warning message when angular range is being limited for safety to give more data on why the problem is happening.
* Increased the max_safe_angular_range for 1.16.01 firmware.
* Correcting console output in getFirmwareVersion
* Added error code to diagnostic summary message as requested by `#3705 <https://github.com/ros-drivers/hokuyo_node/issues/3705>`_.
* Added getFirmwareVersion tool.
* Improved angle limiting on 30LX so that any sufficiently limited angular rance will work. Also reduced the range to 95 degrees/cluster, because that's what it takes to get good data.
* Made the allow_unsafe_settings warning message appear at more appropriate times.
* Added an allow_unsafe_settings option to bypass the UTM-30LX angular range limits.
* Added detection of 30LX, and automatic angle limiting in that case to prevent laser crashes.
* Added optional logging of hokuyo transactions. Have a small test that crashes the laser.
* Had add algorithm include in wrong place.
* Added missing include of algorithm
* Replaced the time calibration method. The new one is much faster, and much less sensitive to the massive clock error on the new hokuyo firmware. Added a test program that can be used for exploring hokuyo timing issues.
* Added code to detect the new hokuyo firmware version, and use a hard-coded latency for it. Now calls calibration code each time the device_id changes, instead of only once at startup.
* Made some more methods public so that other programs can do relatively low-level access.
* Added queryVersionInformation method to allow the version information to be explicitely reread at a time later than the open call.
* Fixed typo in previous commit.
* Made hokuyo export libhokuyo. Enhances external API (but it is an experimental API).
* Added ability to clear the latency set by calcLatency. Added currently used latency and time offset to diagnostics.
* Added time_offset parameter to allow tweaking of time offset.
* Took out compile twice hack
* preparing laser_drivers 1.0.0. Updated manifest descriptions slightly
* Changed diagnostic level messages.
* Previous fix to clear_window was broken. Moved to postOpenHook instead.
* Moved clear_window for frequency diagnostics to doStart so that the diagnostics get reset on any start event. (Was only in reconfigure events.)
* Tweaked a message. (Calibration will take up to a minute.)
* Cleaned up messages. Now says when it has started streaming. Made reads non-blocking to avoid lockup if concurrent access to the port happens. Added workaround for limited precision with which parameters are stored on the parameter server. Merged Connection Status diagnostic into Driver Status diagnostic.
* Added TODO list of tests
* Made turning off laser more robust. Previously there were errors if the turn off command happened just as a scan was arriving.
* Rearranged stop message to be after the scanning actually stops.
* Now only checks angle ranges and intensity support during config_update in the open state. Checking those in the running state was wreaking havoc, as it should.
* Got rid of all output to stderr when getID is in quiet mode.
* Made the serial number scheme compatible with how it used to be (always H in front), but compatible with long serial numbers (don't start with H when returned by device).
* Made getID retry a few times as the hokuyo takes a while before it can be talked to after it is plugged in.
* Made exceptions during startup TM2 command be ignored. Indeed, if the laser is scanning then TM2 fails, but the subsequent command successfully resets the state. Made the subsequent RS instead of QT as it does a more thorough reset.
* Increased limit on hokuyo exception messages.
* Added detection of intensity capability. Node now automatically sets intensity to false if it is unsupported.
* hokuyo_node now reports firmware version, vendor name, product name and firmware version in diagnostics. (Requested by `#3505 <https://github.com/ros-drivers/hokuyo_node/issues/3505>`_)
* Added graceful handling of out-of-range scan range.
* Made hokuyo range go from -pi to pi again because different hokuyos have different ranges, and we can't hard-code some particular range.
* Took race condition out of port locking, and removed unnecessary unlock before close.
* Tweaked package description. Added a debug message.
* Updated licenses.
* Fixed up example launch files.
* Reworked messages so that the driver should never spew when it is failing to reconnect.
* Removed no longer necessary tf stuff from hokuyo example launch file.
* Removed one unnecessary startup message.
* Took node documentation out of doxygen.
* staging laser_drivers into tick-tock

/**
 * @file LoggedImageListSource.cpp
 * @author Anqi Xu
 *
 * WARNING: The contents of ProcerusHeader.h and all other code that depend
 *          or use the Procerus' communication protocol are issued under a
 *          non-disclosure agreement (NDA) between McGill University - Mobile
 *          Robotics Lab and Procerus Inc. In addition, the communication
 *          specifications used to communicate to the Procerus Unicorn unit,
 *          the commbox unit, and the Virtual Cockpit software is controlled
 *          by ITAR export document. As such, it is imperative that projects
 *          using this code be restricted to usage by McGill MRL members only.
 *
 *          > DO NOT DISTRIBUTE WITHOUT CONSENT FROM PROF. GREGORY DUDEK. <
 */


#include "LoggedImageListSource.h"
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <iostream>


using namespace std;
using namespace input;


const string LoggedImageListSource::LOGFILE_EXTENSION = ".log";


LoggedImageListSource::LoggedImageListSource( \
    const std::string& firstImageFilename, bool isTimeSynched) : \
    InputSource(isTimeSynched ? 1 : 0), inputFilename(firstImageFilename), \
    numDigitsInFilename(0), fileID(0), fileIDOffset(0), line(), \
    firstImageID(-1), lastImageID(-1) {
  type = LOGGED_IMAGE_LIST_SOURCE;
};


LoggedImageListSource::~LoggedImageListSource() {
  stopSource();
};


void LoggedImageListSource::initSource() throw (const std::string&) {
  // Clean up any previously-opened sources
  stopSource();

  // Parse user-specified filename for header & numbering format & extension
  InputSource::parseImageFileHeader(inputFilename, fileHeader, \
      fileExtension, fileIDOffset, numDigitsInFilename);

  // Form log filename and open log file
  ostringstream logFilename;
  logFilename << fileHeader << setfill('0') << \
      setw(numDigitsInFilename) << fileIDOffset << LOGFILE_EXTENSION;
  if (logFile.is_open()) { logFile.close(); }
  logFile.open(logFilename.str().c_str(), ios::in);
  if (!logFile.is_open()) {
    ostringstream err;
    err << "Could not find log file: " << logFilename.str();
    throw err.str();
  }

  // Parse time counts inside timing file
  timeIndicesUSEC.clear();
  int currImageID, prevImageID = -1;
  string currTimeString;
  ptime currTime;
  time_duration td;
  hasStartTime = false;
  firstImageID = -1;
  lastImageID = -1;
  while (!logFile.eof()) {
    // Skip blank lines or lines with comments
    getline(logFile, line);
    boost::trim(line);
    if (line.length() <= 0 || line[0] == '%') {
      continue;
    }

    // Parse image ID and time index
    istringstream lineStream(line);
    lineStream >> currImageID >> currTimeString;

    if (hasStartTime) {
      // Make sure imageID is contiguous
      if (currImageID - prevImageID != 1) {
        if (logFile.is_open()) { logFile.close(); }
        alive = false;
        ostringstream err;
        err << "Found non-contiguous imageID sequence between # " << \
            prevImageID << " & " << currImageID;
        throw err.str();
      }
      prevImageID = currImageID;
      lastImageID = currImageID;

      // Compute and store elapsed duration
      currTime = from_iso_string(currTimeString);
      td = currTime - startTime;
      timeIndicesUSEC.push_back((long) td.total_microseconds());
    } else { // First entry: make sure imageID = file's image ID
      if (currImageID != fileIDOffset) {
        if (logFile.is_open()) { logFile.close(); }
        ostringstream err;
        err << "Image ID in filename (" << fileIDOffset << \
            ") does not correspond to first image ID in log (" << \
            currImageID << ")";
        throw err.str();
      }
      prevImageID = currImageID;
      startTime = from_iso_string(currTimeString);
      timeIndicesUSEC.push_back(0);
      hasStartTime = true;
      firstImageID = currImageID;
      lastImageID = currImageID;
    }
  } // while (!logFile.eof())

  // Rewind file to beginning in preparations for reading telemetry
  logFile.clear();
  logFile.seekg(0, ios::beg);
  line.clear();

  // Update status
  fileID = -1; // Not 0, since we start getFrame() by incrementing fileID
  alive = true;
  hasStartTime = false; // NOTE: startTime's usage in THIS function is different!
};


void LoggedImageListSource::stopSource() {
  timeIndicesUSEC.clear();

  if (logFile.is_open()) {
    logFile.close();
  }

  alive = false;
  timeMultiplier = (timeMultiplier > 0) ? 1 : 0;
  hasStartTime = false;
  firstImageID = -1;
  lastImageID = -1;
  line.clear();
};


bool LoggedImageListSource::getFrame(cv::Mat& userBuf) {
  if (timeMultiplier != 0) {
    if (fileID < 0) { // First getFrame() call
      fileID = 0;
      startTime = microsec_clock::local_time();
      hasStartTime = true;
      prevTime = startTime;
      elapsedTime = seconds(0);
    } else {
      // If no more frames, then return false
      if ((unsigned int) fileID + 1 >= timeIndicesUSEC.size()) {
        return false;
      }

      // Compute elapsed time since first getFrame() call
      ptime currTime = microsec_clock::local_time();
      time_duration td = currTime - prevTime;
      elapsedTime += td * timeMultiplier;
      prevTime = currTime;

      // If next frame is not available yet, then sleep a bit
      long timeGap = \
          (long) (timeIndicesUSEC[fileID+1] - elapsedTime.total_microseconds());
      if (timeGap > 0) {
        boost::this_thread::sleep(boost::posix_time::microseconds(timeGap));
        fileID++;
      } else { // Else skip over frames until time gap is longer than elapsed time
        fileID++;
        while ((unsigned int) fileID < timeIndicesUSEC.size() && \
            (timeIndicesUSEC[fileID] < elapsedTime.total_microseconds())) {
          fileID++;
        }

        // If no more frames, then return false
        if ((unsigned int) fileID >= timeIndicesUSEC.size()) {
          return false;
        }
      }
    }
  } else {
    fileID++;
  }

  // Increment numDigits if necessary
  unsigned int newDigitCount = \
      (unsigned int) floor(log10((double) fileID)) + 1;
  if (newDigitCount > numDigitsInFilename) {
    numDigitsInFilename = newDigitCount;
  }

  // Load image and set dimensions
  ostringstream imageFilename;
  imageFilename << fileHeader << setfill('0') << \
      setw(numDigitsInFilename) << fileIDOffset + fileID << fileExtension;
  cv::Mat localBuf = cv::imread(imageFilename.str()); // Assume format is BGR
  if (localBuf.empty()) { // Image failed to load
    return false;
  }
  userBuf = localBuf.clone();
  width = userBuf.rows;
  height = userBuf.cols;

  return true;
};


bool LoggedImageListSource::getTelem(logTelem& buf) {
  try {
    seekForFileID();
  } catch(const std::string& err) {
    cout << "ERROR > " << err << endl;
    return false;
  }

  string iso_string;
  istringstream lineStream(line);
  lineStream >> buf.image_ID >> iso_string >> \

      buf.UTC_year >> buf.UTC_month >> buf.UTC_day >> buf.UTC_hour >> \
      buf.UTC_minute >> buf.UTC_sec >> buf.UTC_millisec >> \

      buf.UAV_altitude_HAL >> buf.UAV_altitude_HAL_desired >> \
      buf.UAV_altitude_MSL >> buf.UAV_altitude_MSL_GPS >> \
      buf.Home_altitude_MSL >> \

      buf.UAV_velocity >> buf.UAV_velocity_GPS >> buf.UAV_velocity_desired >> \

      buf.UAV_latitude >> buf.UAV_longitude >> buf.UAV_latitude_desired >> \
      buf.UAV_longitude_desired >> buf.Home_latitude >> buf.Home_longitude >> \

      buf.time_to_target >> buf.distance_to_target >> buf.heading_to_target >> \

      buf.roll >> buf.roll_desired >> buf.pitch >> buf.pitch_desired >> \

      buf.roll_rate >> buf.pitch_rate >> buf.yaw_rate >> buf.turn_rate >> \
      buf.turn_rate_desired >> buf.climb_rate >> buf.climb_rate_desired >> \

      buf.heading >> buf.heading_magnetometer >> buf.heading_GPS >> \
      buf.heading_desired >> \

      buf.gimbal_azimuth >> buf.gimbal_elevation >> buf.camera_horiz_fov >> \

      buf.aileron_angle >> buf.elevator_angle >> buf.rudder_angle >> \

      buf.GPS_num_satellites >> buf.GPS_RSSI >> buf.throttle >> \

      buf.engineSpeed >> buf.aux_1_servo_pos >> buf.aux_2_servo_pos >> \

      buf.temperature >> buf.wind_heading >> buf.wind_speed >> \

      buf.current_draw >> buf.battery_voltage >> buf.servo_voltage >> \
      buf.alt_voltage >> \

      buf.UAV_mode >> buf.alt_tracker_mode >> buf.current_command >> \
      buf.nav_state >> \

      buf.airborne_time >> buf.home_lock_timer >> buf.takeoff_timer >> \

      buf.system_status >> buf.failsafe_status_1 >> buf.system_flags_1 >> \
      buf.system_flags_2 >> buf.system_flags_3 >> buf.system_flags_4 >> \
      buf.system_flags_5 >> buf.navigation_FLC;

  // Check for error flags prior to last entry
  if (!lineStream.good()) { // NOTE: can use good() here since don't expect EOF yet
    return false;
  }

  // Parse last entry and then check for error flags
  lineStream >> buf.user_IO_pins;
  if ((!lineStream.rdstate() & (ifstream::failbit | ifstream::badbit)) != 0) {
    return false;
  }

  // Parse system time
  buf.sys_time = from_iso_string(iso_string);

  return true;
};


void LoggedImageListSource::setImageID(int desiredID) throw (const std::string&) {
  // Check for boundary conditions
  if (!alive) {
    throw string("Logged image list source has not been initiated.");
  }
  if (desiredID < firstImageID || desiredID > lastImageID) {
    ostringstream err;
    err << "Specified Image ID (" << desiredID << ") is out of bounds (" << \
        firstImageID << " - " << lastImageID << ")";
    throw err.str();
  }

  if (desiredID == fileIDOffset + fileID) { // Already at current position
    return;
  }

  // Set desired file ID and re-seek for telemetry entry
  int newFileID = desiredID - fileIDOffset;
  // Rewind file if necessary
  if (newFileID < fileID) {
    logFile.clear();
    logFile.seekg(0, ios::beg);
  }
  fileID = newFileID;
  seekForFileID();

  // Re-synchronize elapsed duration
  hasStartTime = true;
  startTime = microsec_clock::local_time();
  prevTime = startTime;
  elapsedTime = microseconds(timeIndicesUSEC[fileID]);
};


void LoggedImageListSource::seekForFileID() throw (const std::string&) {
  // Seek for log entry
  int currImageID;

  if (line.length() > 0) {
    istringstream lineStream(line);
    lineStream >> currImageID;
    if (currImageID == fileIDOffset + fileID) {
      return;
    } else if (currImageID > fileIDOffset + fileID) {
      stopSource();
      ostringstream err;
      err << "Image ID in log (" << currImageID << \
          ") is larger than expected image ID (" << \
          fileIDOffset + fileID << ")";
      throw err.str();
    }
  }

  while (!logFile.eof()) {
    // Skip blank lines or lines with comments
    getline(logFile, line);
    boost::trim(line);
    if (line.length() <= 0 || line[0] == '%') {
      continue;
    }

    istringstream lineStream(line);
    lineStream >> currImageID;
    if (currImageID == fileIDOffset + fileID) {
      return;
    } else if (currImageID > fileIDOffset + fileID) {
      stopSource();
      ostringstream err;
      err << "Image ID in log (" << currImageID << \
          ") is larger than expected image ID (" << \
          fileIDOffset + fileID << ")";
      throw err.str();
    }
  }

  // If execution arrives here then throw error
  stopSource();
  ostringstream err;
  err << "Could not find image ID (" << fileIDOffset + fileID << \
        ") in log file.";
  throw err.str();
};

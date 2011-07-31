/**
 * @file LoggedImageListSource.h
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


#ifndef LOGGEDIMAGESOURCE_HPP_
#define LOGGEDIMAGESOURCE_HPP_


#include "InputSource.hpp"
#include <vector>
#include <fstream>
#include <utility>

namespace input {

struct logTelem {
  int image_ID;
  ptime sys_time;
  int UTC_year;
  int UTC_month;
  int UTC_day;
  int UTC_hour;
  int UTC_minute;
  int UTC_sec;
  int UTC_millisec;

  float UAV_altitude_HAL; // m
  float UAV_altitude_HAL_desired; // m
  float UAV_altitude_MSL; // m
  float UAV_altitude_MSL_GPS; // m
  float Home_altitude_MSL; // m

  float UAV_velocity; // m/s
  float UAV_velocity_GPS; // m/s
  float UAV_velocity_desired; // m/s

  float UAV_latitude; // deg
  float UAV_longitude; // deg
  float UAV_latitude_desired; // deg
  float UAV_longitude_desired; // deg
  float Home_latitude; // deg
  float Home_longitude; // deg

  float time_to_target; // sec
  float distance_to_target; // m
  float heading_to_target; // deg

  float roll; // deg
  float roll_desired; // deg
  float pitch; // deg
  float pitch_desired; // deg

  float roll_rate; // deg/s
  float pitch_rate; // deg/s
  float yaw_rate; // deg/s
  float turn_rate; // deg/s
  float turn_rate_desired; // deg/s
  float climb_rate; // deg/s
  float climb_rate_desired; // deg/s

  float heading; // deg
  float heading_magnetometer; // deg
  float heading_GPS; // deg
  float heading_desired; // deg

  float gimbal_azimuth; // deg
  float gimbal_elevation; // deg
  float camera_horiz_fov; // deg

  float aileron_angle; // deg
  float elevator_angle; // deg
  float rudder_angle; // deg

  int GPS_num_satellites; // #
  int GPS_RSSI; // dB
  int throttle; // %

  float engineSpeed; // rev/min
  float aux_1_servo_pos; // deg
  float aux_2_servo_pos; // deg

  float temperature; // 'C
  float wind_heading; // deg
  float wind_speed; // m/s

  float current_draw; // A
  float battery_voltage; // V
  float servo_voltage; // V
  float alt_voltage; // V

  int UAV_mode;
  int alt_tracker_mode;
  int current_command;
  int nav_state;

  float airborne_time; // sec
  float home_lock_timer; // sec
  float takeoff_timer; // sec

  int system_status;
  int failsafe_status_1;
  int system_flags_1;
  int system_flags_2;
  int system_flags_3;
  int system_flags_4;
  int system_flags_5;
  int navigation_FLC;
  int user_IO_pins;
};


class LoggedImageListSource : public InputSource {
public:
  LoggedImageListSource(const std::string& firstImageFilename, \
      bool isTimeSynched = true);
  ~LoggedImageListSource();

  void initSource() throw (const std::string&);
  void initSource(const std::string& newFirstImageFilename) \
      throw (const std::string&) \
      { inputFilename = newFirstImageFilename; initSource(); };
  void stopSource();
  // NOTE: If this function returns a false value, then it is recommended
  //       to proceed by calling stopSource() and
  //       initSource(newFirstImageFilename)
  bool getFrame(cv::Mat& userBuf);

  bool getTelem(logTelem& buf);

  std::pair<int, int> getImageRange() {
    if (!alive) { firstImageID = -1; lastImageID = -1; }
    return std::make_pair(firstImageID, lastImageID);
  };
  void setImageID(int desiredID) throw (const std::string&);
  int getImageID() { return fileIDOffset + fileID; };

  const static std::string LOGFILE_EXTENSION; // See LoggedImageListSource.cpp for declaration

private:
  void seekForFileID() throw (const std::string&);

  std::string inputFilename;
  std::string fileHeader; // File path excluding # & extension
  std::string fileExtension;
  unsigned int numDigitsInFilename;
  int fileID;
  // NOTE: The numbering string in the file is formed by formatting
  //       (fileID + fileIDOffset) using numDigitsInFilename
  int fileIDOffset; // NOTE: This is used if list does not start at 0
  std::string line;
  std::vector<long> timeIndicesUSEC;
  std::ifstream logFile;
  int firstImageID, lastImageID;
};

} // namespace input

#endif /* LOGGEDIMAGESOURCE_HPP_ */

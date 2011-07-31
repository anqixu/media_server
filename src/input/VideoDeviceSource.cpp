/**
 * @file VideoDeviceSource.cpp
 * @author Anqi Xu
 */


#include "VideoDeviceSource.h"
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <limits>
#define FLOAT_PRECISION (std::numeric_limits< float >::digits10 + 1)


using namespace std;
namespace fs = boost::filesystem;
using namespace uav;
using namespace input;


const string VideoDeviceSource::IMAGE_EXTENSION = ".jpg";
const string VideoDeviceSource::LOGFILE_EXTENSION = ".log";
const string VideoDeviceSource::DEFAULT_FILEPATH_HEADER = "./vidlog/image";


VideoDeviceSource::VideoDeviceSource(int device, bool enableDeinterlace, \
    unsigned int multipleGrabs, bool poll, \
    boost::function<void (ImageData d)> cbFn, \
    double framesPerSec, unsigned int imageQualityPercent, bool log, \
    const std::string& filepathHeader, VCDriver* c) \
    throw (const std::string&) : InputSource(1), videoDeviceID(device), cap(), \
    logMode(log && poll), pollMode(poll), isPollerActive(false), \
    callbackFn(cbFn), conn(c), frameBuf(), poller(), bufferMutex(), \
    fileHeader(filepathHeader), imageIDDigits(DEFAULT_IMAGE_ID_DIGITS), \
    imageID(0), \
    frameDelayUSEC(framesPerSec > 0 ? (long) (1000000.0/framesPerSec) : 0), \
    imgQuality(min(imageQualityPercent, (unsigned int) 100)), \
    deinterlace(enableDeinterlace), \
    multigrab(max(multipleGrabs, (unsigned int) 1)) {
  type = VIDEO_DEVICE_SOURCE;
};


VideoDeviceSource::~VideoDeviceSource() {
  conn = NULL;
  callbackFn = NULL;
  stopSource();
};


void VideoDeviceSource::initSource() throw (const std::string&) {
  // Close previously-opened device
  stopSource();

  // Attempt to open device
  cap.open(videoDeviceID);
  if (!cap.isOpened()) {
    throw string("Could not open specified video capture device");
  }

  // Update status
  width = (unsigned int) cap.get(CV_CAP_PROP_FRAME_WIDTH);
  height = (unsigned int) cap.get(CV_CAP_PROP_FRAME_HEIGHT);
  hasStartTime = false;
  // NOTE: We rely on startTime in the non-timesynched mode to measure
  //       the duration between consecutive calls to getFrame(), which
  //       will help us judge whether we should flush VideoCapture's
  //       internal buffer or not (see comment in getFrame()).
  alive = true;

  // Setup log files
  if (logMode) {
    setupFiles();
  }

  // Start poller thread and grab 1st image into buffer
  if (pollMode) {
    if (!cap.grab()) {
      throw string("Unable to grab frame from device");
    }
    cap.retrieve(frameBuf);
    isPollerActive = false;
    poller = boost::thread(boost::bind( \
        &VideoDeviceSource::runPollerWrapper, this));
  }
};


void VideoDeviceSource::stopSource() {
  if (alive) {
    // Remove callbacks
    updateCallbackFn();

    // Close poller thread
    if (isPollerActive) {
      isPollerActive = false;
      poller.timed_join(boost::posix_time::milliseconds( \
            VideoDeviceSource::MAX_JOIN_TIME_MSEC));
    }

    // Close source
    if (cap.isOpened()) {
      cap.release();
    }

    // Update status
    alive = false;
    hasStartTime = false;
  }

  timeMultiplier = 1;
};


bool VideoDeviceSource::getFrame(cv::Mat& userBuf) {
  // NOTE: Video device is always time-synched

  if (isPollerActive) {
    // Return internal image buffer, which SHOULD contain latest frame
    bufferMutex.lock();
    userBuf = frameBuf.clone();
    bufferMutex.unlock();
  } else {
    // NOTE: If the time between consecutive getFrame() calls is sufficiently
    //       large (i.e. slower than 30/5 FPS), then VideoCapture will return
    //       an outdated frame from its internal buffer when cap.grab() is
    //       called. To prevent returning outdated frames, we flush
    //       VideoCapture's internal buffer by calling grab() N times.
    //
    //       In this function, the variables 'startTime' and 'hasStartTime'
    //       is actually being used as "previous time" and "has previous time"
    if (hasStartTime) {
      ptime currTime = microsec_clock::local_time();
      time_duration td = currTime - startTime;
      startTime = currTime;

      if (td.total_microseconds() >= DEVICE_FRAME_DELAY_USEC) {
        // Need to flush internal buffer to obtain up-to-date frame
        bool grabResult = true;
        for (unsigned int i = 0; i < multigrab; i++) {
          grabResult = grabResult && cap.grab();
        }
        if (grabResult) {
          cap.retrieve(userBuf);
          userBuf = userBuf.clone(); // (Probably) to create new local buffer with own reference count
        } else {
          return false;
        }
      } else { // Consecutive getFrame() called before next frame is available
        if (cap.grab()) { // grab() will block until next frame is available
          cap.retrieve(userBuf);
          userBuf = userBuf.clone(); // (Probably) to create new local buffer with own reference count
        } else {
          return false;
        }
      }
    } else { // For the first getFrame() call, return the first grab()
      if (cap.grab()) {
        ptime currTime = microsec_clock::local_time();
        hasStartTime = true;
        prevTime = currTime;
        startTime = currTime;
        elapsedTime = seconds(0);
        cap.retrieve(userBuf);
        userBuf = userBuf.clone(); // (Probably) to create new local buffer with own reference count
      } else {
        return false;
      }
    }

    // Deinterlace image if required
    if (deinterlace) {
      cv::Mat tempBuf;
      cv::resize(userBuf, tempBuf, cv::Size(), 1, 0.5, cv::INTER_NEAREST);
      cv::resize(tempBuf, userBuf, cv::Size(), 1, 2, cv::INTER_LINEAR);
    }
  }

  return true;
};


void VideoDeviceSource::runPoller() {
  unsigned int newDigitCount;
  ofstream logFile;
  ostringstream tempStr;
  sStdTelemPacket telemBuf;
  sNavigationPacket navBuf;
  memset(&telemBuf, 0x00, sizeof(sStdTelemPacket));
  memset(&navBuf, 0x00, sizeof(sNavigationPacket));
  ptime prevTimeTic, currTimeToc;
  time_duration td;
  long timeGapUSEC;
  cv::Mat tempBuf, deinterlacedBuf;

  // Setup image saving parameters
  vector<int> imwriteParams = vector<int> ();
  imwriteParams.push_back(CV_IMWRITE_JPEG_QUALITY);
  imwriteParams.push_back(imgQuality);

  try {
    // Setup log file
    if (logMode) {
      tempStr << fileHeader << "_" << setfill('0') << \
          setw(imageIDDigits) << imageID << LOGFILE_EXTENSION;
      logFile.open(tempStr.str().c_str(), ios::out | ios::app);
      if (!logFile.is_open()) {
        ostringstream err;
        err << "Unable to open log file: " << tempStr;
        throw err.str();
      }
      writeHeaderToFile(logFile);
    }

    // Main loop
    isPollerActive = true;
    bool hasTelems = false;
    while (isPollerActive) {
      // Tick
      prevTimeTic = microsec_clock::local_time();

      // Capture latest frame
      // NOTE: grab() will block until new frame is available
      if (cap.grab()) {
        if (!isPollerActive) { break; }

        // Obtain latest telemetry
        hasTelems = false;
        if (logMode && conn != NULL && conn->isConnected()) {
          conn->getLatestStdTelemPkt(&telemBuf);
          conn->getLatestNavPkt(&navBuf);
          hasTelems = true;
        }

        // Save latest frame into internal memory
        bufferMutex.lock();
        cap.retrieve(frameBuf);
        bufferMutex.unlock();
        if (!isPollerActive) { break; }

        // Callback
        if (callbackFn != NULL) {
          if (hasTelems) {
            callbackFn(ImageData(&frameBuf, &telemBuf, &navBuf));
          } else {
            callbackFn(ImageData(&frameBuf, NULL, NULL));
          }
        }
        if (!isPollerActive) { break; }

        // Deinterlace image if required
        if (deinterlace) {
          cv::resize(frameBuf, tempBuf, cv::Size(), 1, 0.5, cv::INTER_NEAREST);
          cv::resize(tempBuf, frameBuf, cv::Size(), 1, 2, cv::INTER_LINEAR);
        }

        // Construct image filename
        tempStr.clear();
        tempStr.str("");
        tempStr << fileHeader << "_" << setfill('0') << \
            setw(imageIDDigits) << imageID << IMAGE_EXTENSION;

        if (logMode && isPollerActive) {
          // Save latest frame as image file
  #ifdef DISABLE_SAVE_IMAGES
          cout << "IMWRITE: " << tempStr.str() << endl;
  #else
          cv::imwrite(tempStr.str(), frameBuf, imwriteParams);
  #endif

          // Save latest telemetry into log file
          if (!logFile.is_open()) {
            throw string("Log file closed unexpectedly");
          }
          if (hasTelems) {
            writeTelemToFile(logFile, imageID, telemBuf, navBuf);
          } else {
            writeTimeToFile(logFile, imageID);
          }

          // Increment image ID counter
          imageID++;
          if (imageID > 0) {
            newDigitCount = \
                (unsigned int) floor(log10((double) imageID)) + 1;
            if (newDigitCount > imageIDDigits) {
              imageIDDigits = newDigitCount;
            }
          }
        }
      } else { // Grab failed, so terminate thread
        throw string("Could not grab frame from device");
        break;
      }

      // If user has set specified FPS, then wait a bit (if needed)
      // a.k.a. Toc
      currTimeToc = microsec_clock::local_time();
      td = currTimeToc - prevTimeTic;
      timeGapUSEC = (long) (frameDelayUSEC - td.total_microseconds());
      if (timeGapUSEC > 0) {
        boost::this_thread::sleep(boost::posix_time::microseconds( \
          timeGapUSEC));
      }
    }
  } catch (const string& err) {
    cout << endl << "Logger thread terminated: " << err << endl;
    // NOTE: do NOT (re-)throw an exception since this function is supposed
    //       to be inside of a separate thread, and thus throwing an
    //       exception would terminate the thread (and possibly the entire
    //       program)
  }

  // Close logfile
  if (logFile.is_open()) {
    logFile << endl;
    logFile.close();
  }

  isPollerActive = false;
};


void VideoDeviceSource::setupFiles() throw (const std::string&) {
  string tempFileString;
  size_t tempBeginPos, tempSize;
  int scannedID;

  try {
    fs::path headerPath = fs::system_complete(fs::path(fileHeader));
    fs::path parentPath;
    fileHeader = headerPath.string();

    // If parent folder (not root) does not exist, then attempt to create it
    if (headerPath.has_parent_path()) {
      parentPath = headerPath.parent_path();
      if (!fs::exists(parentPath)) {
        if (!fs::create_directory(parentPath)) {
          ostringstream err;
          err << "Unable to create folder: " << parentPath.string();
          throw err.str();
        }
      } else if (!fs::is_directory(parentPath)) {
        ostringstream err;
        err << "Parent path is not a directory: " << parentPath.string();
        throw err.str();
      }
    } else {
      ostringstream err;
      err << "Unable to identify parent folder in path: " << headerPath.string();
      throw err.str();
    }

    // Setup image ID count
    imageIDDigits = DEFAULT_IMAGE_ID_DIGITS;
    imageID = 0;

    // Scan for existing images inside folder and then determine next free ID
    fs::directory_iterator dirIt(parentPath), endDirIt;
    for (; dirIt != endDirIt; dirIt++) {
      // Check for flatfile
      if (fs::is_regular_file(dirIt->status())) {
        // Check for image file extension
        if (boost::iequals(dirIt->path().extension(), IMAGE_EXTENSION)) {
          tempFileString = dirIt->path().leaf();

          // Check for file header
          if (boost::iequals(headerPath.leaf(), \
              tempFileString.substr(0, headerPath.leaf().length()))) {
            tempBeginPos = headerPath.leaf().length() + 1; // 1 = "_" delimiter
            tempSize = tempFileString.length() - tempBeginPos - \
                IMAGE_EXTENSION.length();
            scannedID = atoi(tempFileString.substr(tempBeginPos, tempSize).c_str());
            if (scannedID >= (int) imageID) {
              imageID = scannedID + 1;
            }
          }
        }
      }
    }
  } catch (const std::exception& ex) {
    ostringstream err;
    err << "Could not setup logging folder: " << ex.what();
    throw err.str();
  }
};


void VideoDeviceSource::writeTelemToFile(ofstream& logFile, \
    unsigned int imageID, sStdTelemPacket& telemBuf, \
    sNavigationPacket& navBuf) {
  ptime currTime = microsec_clock::local_time();
  logFile << std::setprecision(FLOAT_PRECISION) << imageID << '\t' << \

      to_iso_string(currTime) << '\t' << \

      (unsigned short) telemBuf.UTCYear + 1900 << '\t' << \
      (unsigned short) telemBuf.UTCMonth << '\t' << \
      (unsigned short) telemBuf.UTCDay << '\t' << \
      (unsigned short) telemBuf.UTCHour << '\t' << \
      (unsigned short) telemBuf.UTCMinute << '\t' << \
      telemBuf.getUTCSec() << '\t' << \
      telemBuf.getUTCMSec() << '\t' << \

      telemBuf.getAltitudeHAL() << '\t' << \
      telemBuf.getDesiredAltitude() << '\t' << \
      telemBuf.getAltitudeMSL() << '\t' << \
      navBuf.getGPSAltitudeMSL() << '\t' << \
      navBuf.getHomeAltitudeMSL() << '\t' << \

      telemBuf.getVelocity() << '\t' << \
      navBuf.getGPSVelocity() << '\t' << \
      telemBuf.getDesiredVelocity() << '\t' << \

      navBuf.GPSLatitude << '\t' << \
      navBuf.GPSLongitude << '\t' << \
      navBuf.DesiredLatitude << '\t' << \
      navBuf.DesiredLongitude << '\t' << \
      navBuf.GPSLatHomePosition << '\t' << \
      navBuf.GPSLonHomePosition << '\t' << \

      navBuf.TimeTargetOrLoiter << '\t' << \
      navBuf.DistanceFromTarget << '\t' << \
      navBuf.getHeadingToTarget() << '\t' << \

      telemBuf.getRoll() << '\t' << \
      telemBuf.getDesiredRoll() << '\t' << \
      telemBuf.getPitch() << '\t' << \
      telemBuf.getDesiredPitch() << '\t' << \

      telemBuf.getRollRate() << '\t' << \
      telemBuf.getPitchRate() << '\t' << \
      telemBuf.getYawRate() << '\t' << \
      telemBuf.getTurnRate() << '\t' << \
      telemBuf.getDesiredTurnRate() << '\t' << \
      telemBuf.getClimbRate() << '\t' << \
      telemBuf.getDesiredClimbRate() << '\t' << \

      telemBuf.getHeading() << '\t' << \
      telemBuf.getMagnetometerHeading() << '\t' << \
      navBuf.getGPSHeading() << '\t' << \
      telemBuf.getDesiredHeading() << '\t' << \

      telemBuf.getGimbalAzimuth() << '\t' << \
      telemBuf.getGimbalElevation() << '\t' << \
      telemBuf.getCameraHorizFOV() << '\t' << \

      telemBuf.getAileronAngle() << '\t' << \
      telemBuf.getElevatorAngle() << '\t' << \
      telemBuf.getRudderAngle() << '\t' << \

      (unsigned short) telemBuf.GPSNumSats << '\t' << \
      (unsigned short) telemBuf.RSSI << '\t' << \
      (unsigned short) telemBuf.Throttle << '\t' << \

      navBuf.getEngineRPM() << '\t' << \
      navBuf.getAux1ServoPos() << '\t' << \
      navBuf.getAux2ServoPos() << '\t' << \

      navBuf.getTemperature() << '\t' << \
      navBuf.getWindHeading() << '\t' << \
      navBuf.getWindSpeed() << '\t' << \

      telemBuf.getCurrentDraw() << '\t' << \
      telemBuf.getBatteryVoltage() << '\t' << \
      navBuf.getServoVoltage() << '\t' << \
      navBuf.getAlternateVoltage() << '\t' << \

      (unsigned short) telemBuf.UAVMode << '\t' << \
      (unsigned short) telemBuf.AltTrackerMode << '\t' << \
      (unsigned short) navBuf.CurrentCommand << '\t' << \
      (unsigned short) navBuf.NavState << '\t' << \

      telemBuf.getAirborneTime() << '\t' << \
      navBuf.getHomeLockTimer() << '\t' << \
      navBuf.getTakeoffTimer() << '\t' << \

      telemBuf.SystemStatus << '\t' << \
      telemBuf.FailsafeStatus1 << '\t' << \
      telemBuf.SystemFlags1 << '\t' << \
      navBuf.SysFlags2 << '\t' << \
      navBuf.SysFlags3 << '\t' << \
      navBuf.SysFlags4 << '\t' << \
      navBuf.SysFlags5 << '\t' << \
      navBuf.FLC << '\t' << \
      navBuf.UserIOPins << '\t' << \
      endl;
};

void VideoDeviceSource::writeTimeToFile(ofstream& logFile, \
    unsigned int imageID) {
  ptime currTime = microsec_clock::local_time();
  logFile << std::setprecision(FLOAT_PRECISION) << imageID << '\t' << \
      to_iso_string(currTime) << '\t' << \
      "1970\t1\t1\t0\t0\t0\t1\t" << \
      "0\t0\t0\t0\t0\t" << \
      "0\t0\t0\t" << \
      "0\t0\t0\t0\t0\t0\t" << \
      "0\t0\t0\t" << \
      "0\t0\t0\t0\t" << \
      "0\t0\t0\t0\t0\t0\t0\t" << \
      "0\t0\t0\t0\t" << \
      "0\t0\t0\t" << \
      "0\t0\t0\t" << \
      "0\t0\t0\t" << \
      "0\t0\t0\t" << \
      "0\t0\t0\t" << \
      "0\t0\t0\t0\t" << \
      "0\t0\t0\t0\t" << \
      "0\t0\t0\t" << \
      "0\t0\t0\t0\t0\t0\t0\t0\t0\t" << endl;
};


void VideoDeviceSource::writeHeaderToFile(ofstream& logFile) {
  ptime currTime = microsec_clock::local_time();
  logFile << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl << \
      "% LOG START TIME: " << to_simple_string(currTime) << endl << \
      "% LOG VERSION: Revision 117" << endl << \
      "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl << \
      "% FORMATTING:" << endl << \
      "%" << endl << \
      "% 01. Image ID" << endl << \
      "% 02. Sys Time" << endl << \
      "% 03. UAV UTC Year" << endl << \
      "% 04. UAV UTC Month" << endl << \
      "% 05. UAV UTC Day" << endl << \
      "% 06. UAV UTC Hour" << endl << \
      "% 07. UAV UTC Minute" << endl << \
      "% 08. UAV UTC Sec" << endl << \
      "% 09. UAV UTC MilliSec" << endl << \
      "%" << endl << \
      "% 10. Altitude HAL (m)" << endl << \
      "% 11. Desired Altitude HAL (m)" << endl << \
      "% 12. Altitude MSL (m)" << endl << \
      "% 13. GPS Altitude MSL (m)" << endl << \
      "% 14. Home Altitude MSL (m)" << endl << \
      "%" << endl << \
      "% 15. Velocity (m/s)" << endl << \
      "% 16. GPS Velocity (m/s)" << endl << \
      "% 17. Desired Velocity (m/s)" << endl << \
      "%" << endl << \
      "% 18. UAV Latitude (deg)" << endl << \
      "% 19. UAV Longitude (deg)" << endl << \
      "% 20. Desired Latitude (deg)" << endl << \
      "% 21. Desired Longitude (deg)" << endl << \
      "% 22. Home Latitude (deg)" << endl << \
      "% 23. Home Longitude (deg)" << endl << \
      "%" << endl << \
      "% 24. Time to Target / Loiter Time (sec)" << endl << \
      "% 25. Distance from Target (m)" << endl << \
      "% 26. Heading to Target (deg)" << endl << \
      "%" << endl << \
      "% 27. Roll (deg)" << endl << \
      "% 28. Desired Roll (deg)" << endl << \
      "% 29. Pitch (deg)" << endl << \
      "% 30. Desired Pitch (deg)" << endl << \
      "%" << endl << \
      "% 31. Roll Rate (deg/s)" << endl << \
      "% 32. Pitch Rate (deg/s)" << endl << \
      "% 33. Yaw Rate (deg/s)" << endl << \
      "% 34. Turn Rate (deg/s)" << endl << \
      "% 35. Desired Turn Rate (deg/s)" << endl << \
      "% 36. Climb Rate (deg/s)" << endl << \
      "% 37. Desired Climb Rate (deg/s)" << endl << \
      "%" << endl << \
      "% 38. Heading (deg)" << endl << \
      "% 39. Magnetometer Heading (deg)" << endl << \
      "% 40. GPS Heading (deg)" << endl << \
      "% 41. Desired Heading (deg)" << endl << \
      "%" << endl << \
      "% 42. Gimbal Azimuth (deg)" << endl << \
      "% 43. Gimbal Elevation (deg)" << endl << \
      "% 44. Camera Horiz. FOV (deg)" << endl << \
      "%" << endl << \
      "% 45. Aileron Angle (deg)" << endl << \
      "% 46. Elevator Angle (deg)" << endl << \
      "% 47. Rudder Angle (deg)" << endl << \
      "%" << endl << \
      "% 48. GPS Satellite #" << endl << \
      "% 49. GPS RSSI (dB)" << endl << \
      "% 50. Throttle (%)" << endl << \
      "%" << endl << \
      "% 51. Engine Speed (rev/min)" << endl << \
      "% 52. Aux 1 Servo Pos" << endl << \
      "% 53. Aux 2 Servo Pos" << endl << \
      "%" << endl << \
      "% 54. Temperature ('C)" << endl << \
      "% 55. Wind Heading (deg)" << endl << \
      "% 56. Wind Speed (m/s)" << endl << \
      "%" << endl << \
      "% 57. Current Draw (A)" << endl << \
      "% 58. Battery Voltage (V)" << endl << \
      "% 59. Servo Voltage (V)" << endl << \
      "% 60. Alt. Voltage (V)" << endl << \
      "%" << endl << \
      "% 61. UAV Mode" << endl << \
      "% 62. Alt. Tracker Mode" << endl << \
      "% 63. Current Command" << endl << \
      "% 64. Nav State" << endl << \
      "%" << endl << \
      "% 65. Airborne Time (sec)" << endl << \
      "% 66. Home Lock Timer (sec)" << endl << \
      "% 67. Takeoff Timer (sec)" << endl << \
      "%" << endl << \
      "% 68. System Status" << endl << \
      "% 69. Failsafe Status 1" << endl << \
      "% 70. System Flags 1" << endl << \
      "% 71. System Flags 2" << endl << \
      "% 72. System Flags 3" << endl << \
      "% 73. System Flags 4" << endl << \
      "% 74. System Flags 5" << endl << \
      "% 75. Navigation FLC" << endl << \
      "% 76. User IO Pins" << endl << \
      "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
};

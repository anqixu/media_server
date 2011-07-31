/**
 * @file VideoDeviceSource.cpp
 * @author Anqi Xu
 */


#include "VideoDeviceSource.hpp"
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
using namespace input;


const string VideoDeviceSource::IMAGE_EXTENSION = ".jpg";
const string VideoDeviceSource::LOGFILE_EXTENSION = ".log";
const string VideoDeviceSource::DEFAULT_FILEPATH_HEADER = "./vidlog/image";


VideoDeviceSource::VideoDeviceSource(int device, bool enableDeinterlace, \
    unsigned int multipleGrabs, bool poll, \
    boost::function<void (ImageData* d)> cbFn, \
    double framesPerSec, unsigned int imageQualityPercent, bool log, \
    const std::string& filepathHeader) \
    throw (const std::string&) : InputSource(1), videoDeviceID(device), cap(), \
    logMode(log && poll), poller(), pollMode(poll), isPollerActive(false), \
    callbackFn(poll ? cbFn : NULL), bufferMutex(), \
    fileHeader(poll ? filepathHeader : ""), \
    imageIDDigits(DEFAULT_IMAGE_ID_DIGITS), \
    imageID(0), \
    frameDelayUSEC((poll && framesPerSec > 0) ? (long) (1000000.0/framesPerSec) : 0), \
    imgQuality(poll ? std::min(std::max((int) imageQualityPercent, 0), 100) : 0), \
    deinterlace(enableDeinterlace), \
    multigrab(std::max(multipleGrabs, (unsigned int) 1)) {
  type = VIDEO_DEVICE_SOURCE;
};


VideoDeviceSource::~VideoDeviceSource() {
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

  // Start poller thread and grab first image into buffer
  if (pollMode) {
    if (!cap.grab()) {
      throw string("Unable to grab frame from device");
    }
    bufferMutex.lock();
    cap.retrieve(imageBuf);
    bufferMutex.unlock();
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

  timeMultiplier = 1; // Although timeMultiplier means nothing here, reset for consistency
};


bool VideoDeviceSource::getFrame(cv::Mat& userBuf) {
  // NOTE: Video device is always time-synched with timeMultiplier == 1 assumed
  cv::Mat localBuf;

  if (isPollerActive) {
    // Return internal image buffer, which SHOULD contain latest frame
    bufferMutex.lock();
    imageBuf.copyTo(userBuf);
    bufferMutex.unlock();
  } else {
    // NOTE: If the time between consecutive getFrame() calls is sufficiently
    //       large (i.e. slower than 30/5 FPS), then VideoCapture will return
    //       an outdated frame from its internal buffer when cap.grab() is
    //       called. To prevent returning outdated frames, we flush
    //       VideoCapture's internal buffer by calling grab() N times.
    //
    // NOTE: In this function, the variables 'startTime' and 'hasStartTime'
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
          // The following retrieve-clone procedure is necessary, as indicated
          // by the documentation of VideoCapture::retrieve(), which uses
          // the C function cvRetrieveFrame() internally
          cap.retrieve(localBuf);
          if (!localBuf.empty()) {
            localBuf.copyTo(userBuf);
          } else {
            return false;
          }
        } else {
          return false;
        }
      } else { // Consecutive getFrame() called before next frame is available
        if (cap.grab()) { // grab() will block until next frame is available
          // The following retrieve-clone procedure is necessary, as indicated
          // by the documentation of VideoCapture::retrieve(), which uses
          // the C function cvRetrieveFrame() internally
          cap.retrieve(localBuf);
          if (!localBuf.empty()) {
            localBuf.copyTo(userBuf);
          } else {
            return false;
          }
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

        // The following retrieve-clone procedure is necessary, as indicated
        // by the documentation of VideoCapture::retrieve(), which uses
        // the C function cvRetrieveFrame() internally
        cap.retrieve(localBuf);
        if (!localBuf.empty()) {
          localBuf.copyTo(userBuf);
        } else {
          return false;
        }
      } else {
        return false;
      }
    }

    // Deinterlace image if required
    if (deinterlace) {
      cv::resize(userBuf, localBuf, cv::Size(), 1, 0.5, cv::INTER_NEAREST);
      cv::resize(localBuf, userBuf, cv::Size(), 1, 2, cv::INTER_LINEAR);
    }
  }

  return true;
};


void VideoDeviceSource::runPoller() {
  unsigned int newDigitCount;
  ofstream logFile;
  ostringstream tempStr;
  ptime prevTimeTic, currTimeToc;
  time_duration td;
  long timeGapUSEC;
  cv::Mat localBuf;

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
      if (frameDelayUSEC > 0) {
        prevTimeTic = microsec_clock::local_time();
      }

      // Capture latest frame
      // NOTE: grab() will block until new frame is available
      if (cap.grab()) {
        if (!isPollerActive) { break; }

        // Obtain latest telemetry
        hasTelems = updateTelems();
        if (!isPollerActive) { break; }

        // Quick-save latest frame into internal memory (without cloning, thus cannot modify contents!)
        bufferMutex.lock();
        cap.retrieve(imageBuf);
        bufferMutex.unlock();
        if (!isPollerActive) { break; }

        // Trigger user-specified callback
        triggerCallbackFn();
        if (!isPollerActive) { break; }

        // Deinterlace image if required
        if (deinterlace) {
          cv::resize(imageBuf, localBuf, cv::Size(), 1, 0.5, cv::INTER_NEAREST);
          cv::resize(localBuf, imageBuf, cv::Size(), 1, 2, cv::INTER_LINEAR);
        }
        if (!isPollerActive) { break; }

        if (logMode && isPollerActive) {
          // Construct image filename
          // if (imageID < 0) { imageID = 0; } // Not needed since imageID is unsigned int
          tempStr.clear();
          tempStr.str("");
          tempStr << fileHeader << "_" << setfill('0') << \
              setw(imageIDDigits) << imageID << IMAGE_EXTENSION;

          // Save latest frame as image file
  #ifdef DISABLE_SAVE_IMAGES
          cout << "IMWRITE: " << tempStr.str() << endl;
  #else
          cv::imwrite(tempStr.str(), imageBuf, imwriteParams);
  #endif

          // Save latest telemetry into log file
          if (!logFile.is_open()) {
            throw string("Log file closed unexpectedly");
          }
          if (hasTelems) {
            writeTelemToFile(logFile, imageID);
          } else {
            writeTimeToFile(logFile, imageID);
          }

          // Increment image ID counter
          imageID++;
          newDigitCount = \
              (unsigned int) floor(log10((double) imageID)) + 1;
          if (newDigitCount > imageIDDigits) {
            imageIDDigits = newDigitCount;
          }
        } // if (logMode && isPollerActive)
      } else { // Grab failed, so terminate thread
        throw string("Could not grab frame from device");
        break;
      }

      // If user has provided a specific FPS value, then wait a bit (if needed)
      // a.k.a. Tock
      if (frameDelayUSEC > 0) {
        currTimeToc = microsec_clock::local_time();
        td = currTimeToc - prevTimeTic;
        timeGapUSEC = (long) (frameDelayUSEC - td.total_microseconds());
        if (timeGapUSEC > 0) {
          boost::this_thread::sleep(boost::posix_time::microseconds( \
            timeGapUSEC));
        }
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
        if (!fs::create_directory(parentPath)) { // TODO: LATER upgrade to create_directories() after upgrading libboost (> 1.40.0)
          ostringstream err;
          err << "Unable to create folder: " << parentPath.string();
          throw err.str();
        }
      } else if (!fs::is_directory(parentPath)) {
        ostringstream err;
        err << "Parent path is not a directory: " << parentPath.string();
        throw err.str();
      } // else parentPath is an existing directory, so a-ok
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

          // Find the smallest imageID that does not exist already
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
    } // for (; dirIt != endDirIt; dirIt++)
  } catch (const std::exception& ex) {
    ostringstream err;
    err << "Could not setup logging folder: " << ex.what();
    throw err.str();
  }
};


void VideoDeviceSource::writeTimeToFile(ofstream& logFile, \
    unsigned int imageID) {
  ptime currTime = microsec_clock::local_time();
#ifdef _DO_NOT_USE__DEPRECATED_OLD_UAV_FORMAT_
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
      "0\t0\t0\t0\t0\t0\t0\t0\t0" << endl;
#else
  logFile << std::setprecision(FLOAT_PRECISION) << imageID << '\t' << \
      to_iso_string(currTime) << endl;
#endif
};


void VideoDeviceSource::writeHeaderToFile(ofstream& logFile) {
  ptime currTime = microsec_clock::local_time();
  logFile << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl << \
      "% LOG START TIME: " << to_simple_string(currTime) << endl << \
      "% LOG VERSION: media_server Revision 5" << endl << \
      "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl << \
      "% FORMATTING:" << endl << \
      "%" << endl << \
      "% 01. Image ID" << endl << \
      "% 02. Sys Time" << endl << \
      "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
};

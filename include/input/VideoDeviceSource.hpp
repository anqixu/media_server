/**
 * @file VideoDeviceSource.hpp
 * @author Anqi Xu
 *
 * WARNING: It appears that the V4L2 driver does not work properly for certain
 *          TV tuner / composite capture cards. If this occurs, you should run
 *          the following command to manually force-load the V4L1 compatibility
 *          drivers:
 *
 *          > LD_PRELOAD=/usr/lib/libv4l/v4l1compat.so [BINARY_NAME]
 */


#ifndef VIDEODEVICESOURCE_HPP_
#define VIDEODEVICESOURCE_HPP_


//#define DISABLE_SAVE_IMAGES


#include "InputSource.hpp"
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/function.hpp>
#include <fstream>
#include <algorithm>


namespace input {


struct ImageData {
  cv::Mat* image;
  bool* alive;
  boost::posix_time::ptime time;

  ImageData(cv::Mat* i = NULL, \
      bool* a = NULL, \
      boost::posix_time::ptime tm = boost::posix_time::microsec_clock::local_time()) : \
      image(i), alive(a), time(tm) {};
};


class VideoDeviceSource : public InputSource {
public:
  // TODO: 9 ensure that runPoller() does not segfault w or w/o callback during destructor

  // NOTE: If framesPerSec == 0, then system will grab as fast as possible.
  //       Also, keep in mind that the system can only grab as fast as there
  //       are new frames available, thus the framesPerSec value serves
  //       only as a suggestion of the desired FPS whereas the true FPS value
  //       may be smaller than this suggested value.
  //
  // device = index + driver_type, where:
  // - if device == 0, then the first video capture source in /dev/video# will be selected
  //
  // - index is a number between 0 to 99 indicating the system ID of the video capture source
  //
  // - driver_type is defined in highgui_c.h as:
  // CV_CAP_ANY      =0,     // autodetect
  //
  // CV_CAP_MIL      =100,   // MIL proprietary drivers
  //
  // CV_CAP_VFW      =200,   // platform native
  // CV_CAP_V4L      =200,
  // CV_CAP_V4L2     =200,
  //
  // CV_CAP_FIREWARE =300,   // IEEE 1394 drivers
  // CV_CAP_FIREWIRE =300,
  // CV_CAP_IEEE1394 =300,
  // CV_CAP_DC1394   =300,
  // CV_CAP_CMU1394  =300,
  //
  // CV_CAP_STEREO   =400,   // TYZX proprietary drivers
  // CV_CAP_TYZX     =400,
  // CV_TYZX_LEFT    =400,
  // CV_TYZX_RIGHT   =401,
  // CV_TYZX_COLOR   =402,
  // CV_TYZX_Z       =403,
  //
  // CV_CAP_QT       =500,   // QuickTime
  //
  // CV_CAP_UNICAP   =600,   // Unicap drivers
  //
  // CV_CAP_DSHOW    =700,   // DirectShow (via videoInput)
  //
  // CV_CAP_PVAPI    =800   // PvAPI, Prosilica GigE SDK
  //
  // NOTE: Setting multipleGrabs > 1 allows the user to manually specify
  //       number of grab()s that are performed during each getFrame().
  //       This is used to flush the internal camera buffers (if present)
  //       of out-dated previous frames.
  //
  // NOTE: options below 'poll' are only relevant if poll == true
  //
  // if log is enabled, then the polled frames will be saved as image files.
  // WARNING: this logging feature is only activated when poll == true
  //          AND log == true; this design choice avoids polluting getFrame()
  //          with logging-related overhead
  //
  // WARNING: The callback function MUST clone() / copyTo() the image matrix
  //          before writing to it! The callback function is also responsible
  //          for checking if the Boolean argument a (a.k.a. alive) is true
  //          during the execution of the callback; if the flag turns false,
  //          then the callback should terminate ASAP.
  VideoDeviceSource(int device = CV_CAP_ANY, bool enableDeinterlace = false, \
      unsigned int multipleGrabs = 1, bool poll = false, \
      boost::function<void (ImageData* d)> cbFn = NULL, \
      double framesPerSec = 0, \
      unsigned int imageQualityPercent = DEFAULT_IMAGE_QUALITY_PERCENT, \
      bool log = false, \
      const std::string& logPathHeader = DEFAULT_FILEPATH_HEADER) \
      throw (const std::string&);
  virtual ~VideoDeviceSource();

  void initSource() throw (const std::string&);
  virtual void initSource(int newDevice, bool enableDeinterlace = false, \
      unsigned int multipleGrabs = 1, bool poll = false, \
      boost::function<void (ImageData* d)> cbFn = NULL, \
      double framesPerSec = 0, \
      unsigned int imageQualityPercent = DEFAULT_IMAGE_QUALITY_PERCENT, \
      bool log = false, \
      const std::string& filepathHeader = DEFAULT_FILEPATH_HEADER) \
      throw (const std::string&) {
    videoDeviceID = newDevice;
    deinterlace = enableDeinterlace;
    multigrab = std::max(multipleGrabs, (unsigned int) 1);
    pollMode = poll;
    callbackFn = poll ? cbFn : NULL;
    frameDelayUSEC = (framesPerSec > 0 && poll) ? \
        (long) (1000000.0/framesPerSec) : 0;
    imgQuality = poll ? std::min(std::max((int) imageQualityPercent, 0), 100) : 0;
    logMode = log && poll;
    fileHeader = poll ? filepathHeader : "";
    initSource();
  };
  void stopSource();
  bool getFrame(cv::Mat& userBuf);

  void deinterlaceImages(bool enabled) {
    deinterlace = enabled;
  };

  /**
   * WARNING: callback function is responsible for cloning the Mat image matrix
   *          if it will take a long time to process the image.
   */
  void updateCallbackFn(
      boost::function<void (ImageData* d)> cbFn = NULL) {
    callbackFn = pollMode ? cbFn : NULL;
  };

  const static unsigned int MAX_JOIN_TIME_MSEC = 500;
  const static long DEVICE_FRAME_DELAY_USEC = 30000; // i.e. ~0.03 sec
  const static std::string IMAGE_EXTENSION; // See VideoDeviceSource.cpp for declaration
  const static std::string LOGFILE_EXTENSION; // See VideoDeviceSource.cpp for declaration
  const static std::string DEFAULT_FILEPATH_HEADER; // See VideoDeviceSource.cpp for declaration
  const static unsigned int DEFAULT_IMAGE_QUALITY_PERCENT = 95;
  const static unsigned int DEFAULT_IMAGE_ID_DIGITS = 6;


protected:
  // Main function for poller thread
  void runPoller();

  // Wrapper function for runPoller()
  static void runPollerWrapper(VideoDeviceSource* me) { me->runPoller(); };

  // Overload the following functions to implement a child class
  // NOTE: these functions are only used in poll mode
  virtual void triggerCallbackFn() {
    if (callbackFn != NULL) {
      ImageData d(&imageBuf, &isPollerActive);
      callbackFn(&d);
    }
  };
  virtual bool updateTelems() { return false; };
  virtual void writeHeaderToFile(std::ofstream& logFile);
  virtual void writeTelemToFile(std::ofstream& logFile, \
      unsigned int imageID) { writeTimeToFile(logFile, imageID); };

  static void writeTimeToFile(std::ofstream& logFile, \
        unsigned int imageID);

  // Validate file names and folder structures for logging purposes
  void setupFiles() throw (const std::string&);

  int videoDeviceID;
  cv::VideoCapture cap;

  bool logMode;

  boost::thread poller;
  bool pollMode;
  bool isPollerActive;
  boost::function<void (ImageData* d)> callbackFn;

  boost::mutex bufferMutex;

  std::string fileHeader;
  unsigned int imageIDDigits;
  unsigned int imageID;

  long frameDelayUSEC;

  unsigned int imgQuality;

  bool deinterlace;

  unsigned int multigrab; // Number of times to call grab()
  // NOTE: use multigrab > 1 to clear internal hardware buffer when using
  //       tv capture card (through trial-and-error)
};

} // namespace input

#endif /* VIDEODEVICESOURCE_HPP_ */

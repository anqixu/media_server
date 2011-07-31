/**
 * @file VideoDeviceSource.hpp
 * @author Anqi Xu
 *
 * WARNING: Currently OpenCV's VideoCapture (C++) class only supports
 *          opening the FIRST video input source under /dev/video*, so
 *          to open different sources you must MANUALLY move video sources
 *          around. For example, if you have /dev/video0 and /dev/video1,
 *          and you wish to open /dev/video1, then you must execute the following:
 *
 *          > sudo mv /dev/video1 /dev/video2
 *          > sudo mv /dev/video0 /dev/video1
 *          > sudo mv /dev/video2 /dev/video0
 *
 *          Please remember to move the files back to their original places
 *          once done.
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

//#include "VCDriver.h"
//using namespace uav;
#define VCDriver void
#define sStdTelemPacket void
#define sNavigationPacket void

namespace input {

struct ImageData {
  cv::Mat* image;
  sStdTelemPacket* telem;
  sNavigationPacket* nav;
  boost::posix_time::ptime time;

  ImageData() : image(NULL), telem(NULL), nav(NULL), \
      time(boost::posix_time::microsec_clock::local_time()) {};
  ImageData(cv::Mat* i, sStdTelemPacket* t, sNavigationPacket* n, \
      boost::posix_time::ptime tm) : image(i), telem(t), nav(n), time(tm) {};
  ImageData(cv::Mat* i, sStdTelemPacket* t, sNavigationPacket* n) :
    image(i), telem(t), nav(n), \
    time(boost::posix_time::microsec_clock::local_time()) {};
};

class VideoDeviceSource : public InputSource {
public:
  // TODO: LATER 0 ensure that runPoller() does not segfault w or w/o callback during destructor

  // NOTE: If framesPerSec == 0, then system will grab as fast as possible.
  //       Also, keep in mind that the system can only grab as fast as there
  //       are new frames available, thus a large framesPerSec value will
  //       be ignored.
  //
  // Values for 'device':
  // CV_CAP_ANY, CV_CAP_MIL, CV_CAP_VFW, CV_CAP_V4L, CV_CAP_V4L2,
  // CV_CAP_FIREWIRE, CV_CAP_IEEE1394, CV_CAP_DC1394, CV_CAP_CMU1394
  VideoDeviceSource(int device = CV_CAP_ANY, bool enableDeinterlace = false, \
      unsigned int multipleGrabs = 1, bool poll = false, \
      boost::function<void (ImageData d)> cbFn = NULL, \
      double framesPerSec = 0, \
      unsigned int imageQualityPercent = DEFAULT_IMAGE_QUALITY_PERCENT, \
      bool log = false, \
      const std::string& filepathHeader = DEFAULT_FILEPATH_HEADER, \
      VCDriver* c = NULL) throw (const std::string&);
  ~VideoDeviceSource();

  void initSource() throw (const std::string&);
  void initSource(int newDevice, bool enableDeinterlace = false, \
      unsigned int multipleGrabs = 1, bool poll = false, \
      boost::function<void (ImageData d)> cbFn = NULL, \
      double framesPerSec = 0, \
      unsigned int imageQualityPercent = DEFAULT_IMAGE_QUALITY_PERCENT, \
      bool log = false, \
      const std::string& filepathHeader = DEFAULT_FILEPATH_HEADER, \
      VCDriver* c = NULL) throw (const std::string&) {
    videoDeviceID = newDevice;
    deinterlace = enableDeinterlace;
    multigrab = multipleGrabs; if (multigrab <= 0) { multigrab = 1; }
    pollMode = poll;
    callbackFn = cbFn;
    if (framesPerSec > 0) { frameDelayUSEC = (long) (1000000.0/framesPerSec); }
    else { frameDelayUSEC = 0; }
    imgQuality = imageQualityPercent;
    if (imgQuality < 0) { imgQuality = 0; }
    else if (imgQuality > 100) { imgQuality = 100; }
    logMode = log && poll;
    fileHeader = filepathHeader;
    conn = c;
    initSource();
  };
  void stopSource();
  bool getFrame(cv::Mat& userBuf);

  void deinterlaceImages(bool enabled) {
    deinterlace = enabled;
  };

  void updateVCDriver(VCDriver* newVC = NULL) { conn = newVC; };

  /**
   * WARNING: callback function is responsible for cloning the matrix if it
   *          will take a long time to process the image.
   */
  void updateCallbackFn(
      boost::function<void (ImageData d)> cbFn = NULL) {
    callbackFn = cbFn;
  };

  const static unsigned int MAX_JOIN_TIME_MSEC = 500;
  const static long DEVICE_FRAME_DELAY_USEC = 30000; // i.e. ~0.03 sec
  const static std::string IMAGE_EXTENSION; // See VideoDeviceSource.cpp for declaration
  const static std::string LOGFILE_EXTENSION; // See VideoDeviceSource.cpp for declaration
  const static std::string DEFAULT_FILEPATH_HEADER; // See VideoDeviceSource.cpp for declaration
  const static unsigned int DEFAULT_IMAGE_QUALITY_PERCENT = 95;
  const static unsigned int DEFAULT_IMAGE_ID_DIGITS = 6;


private:
  // Main function for poller thread
  void runPoller();

  // Wrapper function for runPoller()
  static void runPollerWrapper(VideoDeviceSource* me) { me->runPoller(); };

  // Helper functions with no error checking - hence private
  static void writeTelemToFile(std::ofstream& logFile, \
      unsigned int imageID, sStdTelemPacket& telemBuf, \
      sNavigationPacket& navBuf);
  static void writeTimeToFile(std::ofstream& logFile, \
      unsigned int imageID);
  static void writeHeaderToFile(std::ofstream& logFile);

  // Validate file names and folder structures for logging purposes
  void setupFiles() throw (const std::string&);

  int videoDeviceID;
  cv::VideoCapture cap;
  bool logMode;
  bool pollMode;
  bool isPollerActive;
  boost::function<void (ImageData d)> callbackFn;
  VCDriver* conn;
  cv::Mat frameBuf;
  boost::thread poller;
  boost::mutex bufferMutex;
  std::string fileHeader;
  unsigned int imageIDDigits;
  unsigned int imageID;
  long frameDelayUSEC;
  unsigned int imgQuality;
  bool deinterlace;
  unsigned int multigrab; // Number of times to call grab()
  // NOTE: use multigrab > 1 to clear buffer when using tv capture card
};

} // namespace input

#endif /* VIDEODEVICESOURCE_HPP_ */

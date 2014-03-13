/**
 * @file InputSource.hpp
 * @author Anqi Xu
 */


#ifndef INPUTSOURCE_HPP_
#define INPUTSOURCE_HPP_


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem/path.hpp>

using namespace boost::posix_time;

namespace input {

// NOTE: These classes are designed to block at getFrame() until the next
//       frame is synchronized (e.g. available in case of capture device,
//       past corresponding timing in case of video or synchronized image file).
//       This feature can be disabled by calling setTimeSync(false) when
//       desired (e.g. immediately after construction).
class InputSource {
public:
  typedef enum {UNKNOWN_SOURCE, STUB_SOURCE, IMAGE_LIST_SOURCE, \
    LOGGED_IMAGE_LIST_SOURCE, \
    VIDEO_DEVICE_SOURCE, VIDEO_FILE_SOURCE, \
    UAV_LOGGED_IMAGE_LIST_SOURCE, UAV_VIDEO_DEVICE_SOURCE, \
  } SourceType;

  // NOTE: Children constructors should NOT load internal buffers, but instead
  //       should delay loading to initSource().
  //       This would allow multiple initSource()/stopSource() function calls.
  InputSource(double timeMult) : type(UNKNOWN_SOURCE), imageBuf(), \
      width(-1), height(-1), alive(false), \
      startTime(), hasStartTime(false), prevTime(), elapsedTime(), \
      timeMultiplier(timeMult <= 0 ? 0 : timeMult) {};
  virtual ~InputSource() {};

  // NOTE: This function is responsible for setting width, height, alive, timeSync
  // NOTE: For restartable media sources (e.g. image lists, video files),
  //       re-calling initSource() should restart the media sequence
  virtual void initSource() throw (const std::string&) = 0;

  // NOTE: This function is responsible for resetting alive
  //       In addition, by convention this function should also reset
  //       timeMultiplier = (timeMultiplier != 0) ? 1 : 0
  virtual void stopSource() = 0;

  // NOTE: If no frame is available, return false BUT DO NOT CALL stopSource()
  // NOTE: All derived implementations MUST provide blocking functionality if
  //       timeSync is enabled and when time between consecutive frames is
  //       longer than wall time between consecutive getFrame() calls
  virtual bool getFrame(cv::Mat& userBuf) = 0;

  virtual std::pair<int, int> getIndexRange() {
    return std::make_pair(-1, -1);
  };

  virtual double getFPS() { return 0.0; };

  virtual std::string getName() = 0;

  // For seekable (and thus bounded) input sources, ratio = 0 means
  // seek to beginning and ratio = 1 means seek to end
  //
  // NOTE: For time-synchronized sources, calling this function will begin
  //       "ticking" the clock immediately (as opposed to waiting for getFrame()
  //       to be called later)
  virtual bool seek(double ratio) { return false; };

  int getWidth() { return width; };
  int getHeight() { return height; };
  double getAspectRatio() { return (double) width / height; };
  bool isAlive() { return alive; };
  double getTimeMultiplier() { return timeMultiplier; };
  // NOTE: Source will be reset to initial position when toggling time synchronization
  virtual double setTimeMultiplier(double newMult);
  SourceType getType() { return type; };

  // This function is used by *ImageListSource child classes
  static void parseImageFileHeader(const std::string& firstImageFilename, \
      std::string& headerBuf, std::string& extensionBuf, \
      int& firstImageIDBuf, unsigned int& numDigitsBuf) \
      throw (const std::string&);

  // Recursive directory creation
  //
  // From http://www.mail-archive.com/boost@lists.boost.org/msg02228.html
  static bool createDirectories(const boost::filesystem::path& ph);

protected:
  SourceType type;
  cv::Mat imageBuf; // NOTE: child classes may or may not store loaded images in here
  int width;
  int height;
  bool alive;

  // NOTE: For time synchronization, update the following 3 members together
  ptime startTime;
  bool hasStartTime;
  ptime prevTime;

  time_duration elapsedTime;
  double timeMultiplier; // frame-time = wall-time * timeMultiplier

private:
  // Disable copy constructor and copy assignment operator
  InputSource(const InputSource& rhs) {};
  InputSource& operator=(const InputSource& rhs) {return *this; };
};

} // namespace input


#endif /* INPUTSOURCE_HPP_ */

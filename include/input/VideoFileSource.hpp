/**
 * @file VideoFileSource.hpp
 * @author Anqi Xu
 */


#ifndef VIDEOFILESOURCE_HPP_
#define VIDEOFILESOURCE_HPP_


#include "InputSource.h"


namespace input {

class VideoFileSource : public InputSource {
public:
  VideoFileSource(const std::string& videoFile, bool isTimeSynched);
  ~VideoFileSource();

  void initSource() throw (const std::string&);
  void initSource(const std::string& newVideoFile) throw (const std::string&) \
      { sourceFile = newVideoFile; initSource(); };
  void stopSource();
  bool getFrame(cv::Mat& userBuf);

  unsigned int getFPS() {
    return (vid.isOpened() ? (int) vid.get(CV_CAP_PROP_FPS) : 0);
  };
  unsigned int getNumFrames() {
    return (vid.isOpened() ? (int) vid.get(CV_CAP_PROP_FRAME_COUNT) : 0);
  };
  double getCVFileProperty(int propID) {
    return (vid.isOpened() ? vid.get(propID) : 0);
  };

private:
  // Disable copy constructor and copy assignment operator
  VideoFileSource(const VideoFileSource& rhs) : InputSource(false) {};
  VideoFileSource& operator=(const VideoFileSource& rhs) {return *this; };

  std::string sourceFile;
  cv::VideoCapture vid;
};

} // namespace input


#endif /* VIDEOFILESOURCE_HPP_ */

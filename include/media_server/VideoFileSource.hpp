/**
 * @file VideoFileSource.hpp
 * @author Anqi Xu
 */


#ifndef VIDEOFILESOURCE_HPP_
#define VIDEOFILESOURCE_HPP_


#include "media_server/InputSource.hpp"


namespace input {

class VideoFileSource : public InputSource {
public:
  VideoFileSource(const std::string& videoFile, double timeMult = 1.0);
  ~VideoFileSource();

  void initSource() throw (const std::string&);
  void initSource(const std::string& newVideoFile, \
      double newTimeMultiplier = 1.0) throw (const std::string&) {
    sourceFile = newVideoFile;
    timeMultiplier = (newTimeMultiplier > 0) ? newTimeMultiplier : 0;
    initSource();
  };
  void stopSource();
  bool getFrame(cv::Mat& userBuf);

  virtual std::pair<int, int> getIndexRange() {
    return (vid.isOpened() ? \
        std::make_pair(0, (int) (vid.get(CV_CAP_PROP_FRAME_COUNT)) - 1) : \
        std::make_pair(-1, -1));
  };

  bool seek(double ratio);

  double getFPS() {
    return (vid.isOpened() ? vid.get(CV_CAP_PROP_FPS) : 0.0);
  };
  unsigned int getNumFrames() {
    return (vid.isOpened() ? (int) vid.get(CV_CAP_PROP_FRAME_COUNT) : 0);
  };
  double getCVFileProperty(int propID) {
    return (vid.isOpened() ? vid.get(propID) : 0);
  };

  std::string getVideoFilename() { return sourceFile; };
  std::string getName() { return sourceFile; };

private:
  // Disable copy constructor and copy assignment operator
  VideoFileSource(const VideoFileSource& rhs) : InputSource(false) {};
  VideoFileSource& operator=(const VideoFileSource& rhs) {return *this; };

  std::string sourceFile;
  cv::VideoCapture vid;
};

} // namespace input


#endif /* VIDEOFILESOURCE_HPP_ */

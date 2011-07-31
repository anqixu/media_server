/**
 * @file ImageListSource.hpp
 * @author Anqi Xu
 */


#ifndef IMAGELISTSOURCE_HPP_
#define IMAGELISTSOURCE_HPP_


#include "InputSource.hpp"


namespace input {

class ImageListSource : public InputSource {
public:
  // NOTE: If FPS <= 0, then time sychronization will be disabled
  ImageListSource(const std::string& firstImage, double FPS = DEFAULT_FPS);
  ~ImageListSource();

  void initSource() throw (const std::string&);
  void initSource(const std::string& newFirstImage, \
      double newFPS = 15) throw (const std::string&) \
      { inputFilename = newFirstImage; framesPerSec = newFPS; initSource(); };
  void stopSource();
  // NOTE: If this function returns a false value, then it is recommended
  //       to proceed by calling stopSource() and initSource(newFirstImage)
  bool getFrame(cv::Mat& userBuf);

  const static double DEFAULT_FPS = 15;

private:
  bool parseFileHeader(const std::string& imageFilename) \
      throw (const std::string&);

  std::string inputFilename;
  std::string fileHeader; // File path excluding # & extension
  std::string fileExtension;
  double framesPerSec;
  int fileID, firstFileID;
  unsigned int numDigitsInFilename;
};

} // namespace input


#endif /* IMAGELISTSOURCE_HPP_ */

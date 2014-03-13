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
      double newFPS = DEFAULT_FPS) throw (const std::string&) {
    inputFilename = newFirstImage;
    framesPerSec = (newFPS > 0) ? newFPS : 0;
    initSource();
  };
  void stopSource();
  // NOTE: If this function returns a false value, then it is recommended
  //       to proceed by calling stopSource() and initSource(newFirstImage)
  bool getFrame(cv::Mat& userBuf);

  std::pair<int, int> getIndexRange() {
    if (!alive) { firstFileID = -1; lastFileID = -1; }
    return std::make_pair(firstFileID, lastFileID);
  };

  int getImageID() { return alive ? fileID : -1; };

  bool seek(double ratio);

  std::string getFirstImageFilename() { return inputFilename; };
  std::string getName() { return inputFilename; };

  double getFPS() { return alive ? framesPerSec : 0; };

  constexpr static double DEFAULT_FPS = 15.;

private:
  bool parseFileHeader(const std::string& imageFilename) \
      throw (const std::string&);

  static int scanForLastFileID(const std::string& header, \
      const std::string& extension, int firstID, int numDigits);

  std::string inputFilename;
  std::string fileHeader; // File path excluding # & extension
  std::string fileExtension;
  double framesPerSec;
  int fileID, firstFileID, lastFileID;
  unsigned int numDigitsInFilename;
};

} // namespace input


#endif /* IMAGELISTSOURCE_HPP_ */

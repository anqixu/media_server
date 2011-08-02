/**
 * @file LoggedImageListSource.hpp
 * @author Anqi Xu
 */


#ifndef LOGGEDIMAGELISTSOURCE_HPP_
#define LOGGEDIMAGELISTSOURCE_HPP_


#include "InputSource.hpp"
#include <vector>
#include <fstream>
#include <utility>

namespace input {

struct logTelem {
  int image_ID;
  ptime sys_time;
};


// This is a generic logged image list loader, to load a sequence of image
// of the format [HEADER]_[IMAGE_ID].[EXT], accompanied with the log file
// [HEADER]_[FIRST_ID].[LOGFILE_EXTENSION]. The log file is assumed to
// contain space-deliminated telemetry entries for each of the images in
// the sequence, of the format:
//
// % This is a comment line, where % is the escape character
// [IMAGE_ID] [PTIME_ISO_STRING]
//
// Children of this class are expected to share the same log file format,
// but may contain additional space-deliminated entries
class LoggedImageListSource : public InputSource {
public:
  LoggedImageListSource(const std::string& firstImageFilename, \
      double timeMult = 1.0);
  virtual ~LoggedImageListSource();

  void initSource() throw (const std::string&);
  void initSource(const std::string& newFirstImageFilename,
      double newTimeMultiplier = 1.0) \
      throw (const std::string&) {
    inputFilename = newFirstImageFilename;
    timeMultiplier = (newTimeMultiplier > 0) ? newTimeMultiplier : 0;
    initSource();
  };
  void stopSource();

  // NOTE: If this function returns a false value, then it is recommended
  //       to proceed by calling stopSource() and
  //       initSource(newFirstImageFilename)
  bool getFrame(cv::Mat& userBuf);

  virtual bool getTelem(logTelem* buf);

  std::pair<int, int> getIndexRange() {
    if (!alive) { firstImageID = -1; lastImageID = -1; }
    return std::make_pair(firstImageID, lastImageID);
  };

  bool seek(double ratio);

  void setImageID(int desiredID) throw (const std::string&);
  int getImageID() { return fileIDOffset + fileID; };

  const static std::string LOGFILE_EXTENSION; // See LoggedImageListSource.cpp for declaration

protected:
  // Seek for the line in the log file starting with the image ID = fileID + fileIDOffset
  //
  // Will throw exception if the seeked image ID is either smaller than the
  // value in the next valid line inside the log file, or cannot be found
  // in the remainder of the log file.
  //
  // WARNING: if an exception is thrown, the user is still responsible for
  //          resetting the input source by calling stopSource() manually
  void seekForFileID() throw (const std::string&);

  std::string inputFilename;
  std::string fileHeader; // File path excluding # & extension
  std::string fileExtension;
  unsigned int numDigitsInFilename;
  int fileID;
  // NOTE: The numbering string in the file is formed by formatting
  //       (fileID + fileIDOffset) using numDigitsInFilename
  int fileIDOffset; // NOTE: This is useful if list does not start at 0
  std::string line;
  std::vector<long> timeIndicesUSEC;
  std::ifstream logFile;
  int firstImageID, lastImageID;
};

} // namespace input

#endif /* LOGGEDIMAGELISTSOURCE_HPP_ */

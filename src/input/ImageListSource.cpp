/**
 * @file ImageListSource.cpp
 * @author Anqi Xu
 */


#include "ImageListSource.hpp"
#include <boost/thread/thread.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>


using namespace std;
using namespace input;
namespace fs = boost::filesystem;


ImageListSource::ImageListSource(const std::string& firstImage, \
    double FPS) : InputSource((FPS <= 0) ? 0 : 1), inputFilename(firstImage), \
    framesPerSec((FPS > 0) ? FPS : 0), \
    fileID(0), firstFileID(0), lastFileID(0), numDigitsInFilename(0) {
  type = IMAGE_LIST_SOURCE;
};


ImageListSource::~ImageListSource() {
  stopSource();
};


void ImageListSource::initSource() throw (const std::string&) {
  // Clean up any previously-opened sources
  stopSource();

  // Parse user-specified filename for header & numbering format & extension
  InputSource::parseImageFileHeader(inputFilename, fileHeader, \
      fileExtension, firstFileID, numDigitsInFilename);
  fileID = firstFileID;
  lastFileID = ImageListSource::scanForLastFileID(fileHeader,
      fileExtension, firstFileID, numDigitsInFilename);

  // Validate FPS (and change time synchronization mode if necessary)
  if (framesPerSec <= 0) {
    timeMultiplier = 0;
    framesPerSec = 0;
  } else {
    timeMultiplier = 1;
  }

  // Update status
  alive = true;
  hasStartTime = false;
};


void ImageListSource::stopSource() {
  alive = false;
  timeMultiplier = (timeMultiplier > 0) ? 1 : 0;
};


bool ImageListSource::getFrame(cv::Mat& userBuf) {
  if (timeMultiplier > 0) {
    if (!hasStartTime) { // First getFrame() call
      startTime = microsec_clock::local_time();
      prevTime = startTime;
      hasStartTime = true;
      elapsedTime = seconds(0);
      fileID = firstFileID;
    } else {
      // Compute elapsed time since first (not a typo) getFrame() call
      ptime currTime = microsec_clock::local_time();
      time_duration td = currTime - prevTime;
      // NOTE: following expression can't be simplified due to cast in time_duration::operator*(int)
      elapsedTime += microseconds((long) ((float) td.total_microseconds() * timeMultiplier)); // Note the += ...
      prevTime = currTime; // (thus duration computed from first getFrame() call)
      // NOTE: elapsedTime is within the image-list-time frame, thus
      //       any updates to this variable need to consider timeMultiplier

      // Compute expected fileID from elapsed time
      int nextID = \
          (int) floor(elapsedTime.total_microseconds() * framesPerSec / 1000000.0) +
          firstFileID;
      if (nextID <= fileID) { // If next frame is not available yet, then sleep
        fileID++; // ... eventually load next file in sequence
        if (framesPerSec > 0) {
          long sleepTimeUSEC = (long) (floor((fileID - \
              firstFileID)*1000000/framesPerSec) - \
              elapsedTime.total_microseconds());
          // NOTE: the previous duration is computed in frame-time and not
          //       wall-time, thus we need to factor out timeMultiplier
          //       when using it in real space (a.k.a. wall time)
          sleepTimeUSEC = (long) floor(sleepTimeUSEC / timeMultiplier);

          if (sleepTimeUSEC > 0) {
            boost::this_thread::sleep(boost::posix_time::microseconds( \
                sleepTimeUSEC));
            // NOTE: no need to update elapsedTime since prevTime was not updated
          }
        }
      } else {
        fileID = nextID;
      }
    }
  } else { // not time-synchronized
    fileID++;
  }

  // Increment numDigits if necessary
  unsigned int newDigitCount = \
      (unsigned int) floor(log10((double) fileID)) + 1;
  if (newDigitCount > numDigitsInFilename) {
    numDigitsInFilename = newDigitCount;
  }

  // Load image directly into caller's buffer, and then update image dimensions
  ostringstream imageFilename;
  imageFilename << fileHeader << setfill('0') << \
      setw(numDigitsInFilename) << fileID << fileExtension;
  userBuf = cv::imread(imageFilename.str()); // Assume format is BGR
  if (userBuf.empty()) { // Image failed to load
    return false;
  }
  width = userBuf.rows;
  height = userBuf.cols;

  return true;
};


int ImageListSource::scanForLastFileID(const std::string& header, \
    const std::string& extension, int firstID, int numDigits) {
  int scanID = firstID;
  int nDigits = numDigits, newDigitCount;

  while (1) {
    newDigitCount = (int) floor(log10((double) scanID)) + 1;
    if (newDigitCount > nDigits) {
      nDigits = newDigitCount;
    }

    ostringstream imageFilename;
    imageFilename << header << setfill('0') << \
        setw(nDigits) << scanID << extension;

    if (!fs::exists(fs::path(imageFilename.str()))) {
      break;
    }

    scanID++;
  }

  return scanID;
};


bool ImageListSource::seek(double ratio) {
  ratio = std::min(std::max(ratio, 0.0), 1.0);
  bool result = false;
  if (alive) {
    result = true;
    fileID = firstFileID + (int) round((lastFileID - firstFileID)*ratio);
    if (fileID < firstFileID || fileID > lastFileID) { fileID = firstFileID; }
    if (timeMultiplier > 0) {
      prevTime = microsec_clock::local_time();
      elapsedTime = microseconds((fileID - firstFileID) * 1000000.0 / framesPerSec);
      hasStartTime = true;
      startTime = prevTime - elapsedTime;
    }
  }
  return result;
};

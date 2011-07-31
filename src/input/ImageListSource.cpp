/**
 * @file ImageListSource.cpp
 * @author Anqi Xu
 */


#include "ImageListSource.h"
#include <boost/thread/thread.hpp>


using namespace std;
using namespace input;


ImageListSource::ImageListSource(const std::string& firstImage, \
    double FPS) : InputSource((FPS <= 0) ? 0 : 1), inputFilename(firstImage), \
    framesPerSec(FPS), fileID(0), firstFileID(0), numDigitsInFilename(0) {
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
  if (timeMultiplier != 0) {
    if (!hasStartTime) { // First getFrame() call
      startTime = microsec_clock::local_time();
      prevTime = startTime;
      hasStartTime = true;
      elapsedTime = seconds(0);
    } else {
      // Compute elapsed time since first getFrame() call
      ptime currTime = microsec_clock::local_time();
      time_duration td = currTime - prevTime;
      elapsedTime += td * timeMultiplier;
      prevTime = currTime;

      // Compute expected fileID from elapsed time
      int nextID = \
          (int) floor(elapsedTime.total_microseconds() * framesPerSec / 1000000.0);
      if (nextID <= fileID) { // If next frame is not available yet, then sleep
        fileID++;
        long sleepTimeUSEC = (long) (floor((fileID - \
            firstFileID)*1000000/framesPerSec) - \
            elapsedTime.total_microseconds());

        if (sleepTimeUSEC > 0) {
          boost::this_thread::sleep(boost::posix_time::microseconds( \
              sleepTimeUSEC));
        }

        fileID++;
      } else {
        fileID = nextID;
      }
    }
  } else {
    fileID++;
  }

  // Increment numDigits if necessary
  unsigned int newDigitCount = \
      (unsigned int) floor(log10((double) fileID)) + 1;
  if (newDigitCount > numDigitsInFilename) {
    numDigitsInFilename = newDigitCount;
  }

  // Load image and set dimensions
  ostringstream imageFilename;
  imageFilename << fileHeader << setfill('0') << \
      setw(numDigitsInFilename) << fileID << fileExtension;
  cv::Mat localBuf = cv::imread(imageFilename.str()); // Assume format is BGR
  if (localBuf.empty()) { // Image failed to load
    return false;
  }
  userBuf = localBuf.clone();
  width = userBuf.rows;
  height = userBuf.cols;

  return true;
};

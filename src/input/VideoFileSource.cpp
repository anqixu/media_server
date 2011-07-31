/**
 * @file VideoFileSource.cpp
 * @author Anqi Xu
 */


#include "VideoFileSource.hpp"
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/thread/thread.hpp> // Needed for sleep()


using namespace std;
namespace fs = boost::filesystem;
using namespace input;


VideoFileSource::VideoFileSource(const std::string& videoFile, \
    bool isTimeSynched) : InputSource(isTimeSynched ? 1 : 0), \
    sourceFile(videoFile), vid() {
  type = VIDEO_FILE_SOURCE;
};


VideoFileSource::~VideoFileSource() {
  stopSource();
};


void VideoFileSource::initSource() throw (const std::string&) {
  // Close existing source if applicable
  stopSource();

  // Attempt to open new source
  fs::path sourceFilepath = fs::complete(fs::path(sourceFile));
  if (!vid.open(sourceFilepath.string())) {
    ostringstream err;
	err << "Could not find or load video file: " << sourceFilepath.string();
    throw err.str();
  }

  // Update status
  width = (unsigned int) vid.get(CV_CAP_PROP_FRAME_WIDTH);
  height = (unsigned int) vid.get(CV_CAP_PROP_FRAME_HEIGHT);
  alive = true;
  hasStartTime = false;
};


void VideoFileSource::stopSource() {
  // Close source
  if (vid.isOpened()) {
    vid.release();
  }

  // Update status
  alive = false;
  hasStartTime = false;
  timeMultiplier = (timeMultiplier > 0) ? 1 : 0;
};


bool VideoFileSource::getFrame(cv::Mat& userBuf) {
  // Ensure that source is loaded and ready to spit out frames
  if (vid.isOpened()) {
    if (timeMultiplier > 0) {
      if (hasStartTime) {
        ptime currTime;
        time_duration td;
        long currVidUSEC, diffUSEC;

        while (1) {
          // Continuously grab...
          if (!vid.grab()) {
            return false;
          }

          // ... and compute time differences, both for wall time and for
          // frame time...
          currVidUSEC = (long) (vid.get(CV_CAP_PROP_POS_MSEC) * 1000);
          currTime = microsec_clock::local_time();
          td = currTime - prevTime;
          elapsedTime += td * timeMultiplier;
          prevTime = currTime;

          // ... until wall time between queries is shorter than
          // frame gap
          diffUSEC = currVidUSEC - (long) elapsedTime.total_microseconds();
          if (diffUSEC >= 0) { // Local frame is valid; return to user
            // The following retrieve-clone procedure is necessary, as indicated
            // by the documentation of VideoCapture::retrieve(), which uses
            // the C function cvRetrieveFrame() internally
            vid.retrieve(imageBuf);
            if (imageBuf.empty()) { // If something went wrong in vid.retrieve()
              return false;
            } else {
              imageBuf.copyTo(userBuf);
            }

            // Sleep a bit to synchronize wall time with video time
            currTime = microsec_clock::local_time();
            td = currTime - prevTime;
            elapsedTime += td * timeMultiplier;
            prevTime = currTime;
            // NOTE: the division of timeMultiplier in the following duration
            //       calculation is needed to convert from frame-time to wall-time
            diffUSEC = (long) floor( (currVidUSEC - \
                elapsedTime.total_microseconds()) / timeMultiplier );
            if (diffUSEC > 0) {
              boost::this_thread::sleep(boost::posix_time::microseconds( \
                  diffUSEC));
            }
            return true;
          }
        }
      } else { // First request: return next frame and record start time
        if (vid.grab()) {
          startTime = microsec_clock::local_time();
          hasStartTime = true;
          prevTime = startTime;
          elapsedTime = seconds(0);

          // The following retrieve-clone procedure is necessary, as indicated
          // by the documentation of VideoCapture::retrieve(), which uses
          // the C function cvRetrieveFrame() internally
          vid.retrieve(imageBuf);
          if (imageBuf.empty()) {
            return false;
          } else {
            imageBuf.copyTo(userBuf);
            return true;
          }
        }
      }
    } else { // Not time-synchronized
      if (vid.grab()) {
        // The following retrieve-clone procedure is necessary, as indicated
        // by the documentation of VideoCapture::retrieve(), which uses
        // the C function cvRetrieveFrame() internally
        vid.retrieve(imageBuf);
        if (imageBuf.empty()) {
          return false;
        } else {
          imageBuf.copyTo(userBuf);
          return true;
        }
      }
    }
  }

  return false;
};

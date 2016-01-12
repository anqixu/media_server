#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "media_server/StubSource.hpp"
#include "media_server/ImageListSource.hpp"
#include "media_server/LoggedImageListSource.hpp"
#include "media_server/VideoFileSource.hpp"
#include "media_server/VideoDeviceSource.hpp"
#include "media_server/LoadImageList.h"
#include "media_server/LoadLoggedImageList.h"
#include "media_server/LoadVideoFile.h"
#include "media_server/LoadVideoDevice.h"
#include "media_server/GetMediaStatus.h"
#include "media_server/SeekIndex.h"
#include "media_server/SetTimeMultiplier.h"
// TODO: 1 debug why if we can't load video device; or loads video file that's not there, ROS node crashes and service call does not return

using namespace std;
using namespace input;


class MediaServer {
public:
  MediaServer(ros::NodeHandle& nh, \
      bool idle, double customStartSec) : \
      imageSeqID(0), source(NULL), \
      idleEnabled(idle), inIdleMode(true), \
      streamMode(false), repeatMode(false) {
    // Set start time for publishing video frames
    if (customStartSec >= 0) {
      customStartTime = ros::Time(customStartSec);
    }

    // Setup image topic
    std::string image_topic = nh.resolveName("image");
    image_transport::ImageTransport it(nh);
    imagePub = it.advertise(image_topic, 1);

    // Setup services
    loadImageListSrv = nh.advertiseService("load_image_list", \
        &MediaServer::loadImageListCB, (MediaServer*) this);
    loadLoggedImageListSrv = nh.advertiseService("load_logged_image_list", \
        &MediaServer::loadLoggedImageListCB, this);
    loadVideoFileSrv = nh.advertiseService("load_video_file", \
        &MediaServer::loadVideoFileCB, this);
    loadVideoDeviceSrv = nh.advertiseService("load_video_device", \
        &MediaServer::loadVideoDeviceCB, this);
    getMediaStatusSrv = nh.advertiseService("get_media_status", \
        &MediaServer::getMediaStatusCB, this);
    seekIndexSrv = nh.advertiseService("seek_index", \
        &MediaServer::seekIndexCB, this);
    setTimeMultiplierSrv = nh.advertiseService("set_time_multiplier", \
        &MediaServer::setTimeMultiplierCB, this);
    pollFrameSrv = nh.advertiseService("poll_frame", \
        &MediaServer::pollFrameCB, this);

    ROS_INFO_STREAM("MEDIA_SERVER: initiated idle server on image topic " <<
        image_topic << " with idle mode " << (idleEnabled ? "ENABLED" : "DISABLED"));
  };

  void spin() {
    // Main loop body
    ros::Rate sleepRate(200); // 200Hz should be sufficient for regular usage of poll mode
    while (ros::ok()) {
      // REMINDER: streamed video device uses callback fn
      // REMINDER: getFrame() will block if time-synchronized
      if (!inIdleMode && source != NULL && source->isAlive() && streamMode && \
          (source->getType() != InputSource::VIDEO_DEVICE_SOURCE)) {
        inIdleMode = false;
        pushNextFrame();
      } else {
        sleepRate.sleep();
      }
      ros::spinOnce();
    }

    // Clean up
    if (source != NULL) {
      source_mutex.lock();
      delete source;
      source = NULL;
      source_mutex.unlock();
    }
    ROS_INFO("MEDIA_SERVER: terminated server");
    terminateMediaServer();
    return;
  };

  bool loadImageListCB( \
      media_server::LoadImageList::Request& request, \
      media_server::LoadImageList::Response& response) {
    try {
      updateSettings(request.framesPerSecond > 0 ? request.streamMode : false, \
          request.repeatMode);
      swapSource(new ImageListSource(request.firstImageFilename, \
          request.framesPerSecond));
      response.error = "";
    } catch (const std::string& err) {
      ROS_ERROR_STREAM("MEDIA_SERVER: unable to load image list @ " << \
          request.firstImageFilename << " - " << err);
      response.error = err;
    }
    ROS_INFO_STREAM("MEDIA_SERVER: image list loaded @ " << \
        request.firstImageFilename);

    return true;
  };

  bool loadLoggedImageListCB( \
      media_server::LoadLoggedImageList::Request& request, \
      media_server::LoadLoggedImageList::Response& response) {
    try {
      updateSettings(request.timeMultiplier > 0 ? request.streamMode : false, \
          request.repeatMode);
      swapSource(new LoggedImageListSource(request.firstImageFilename, \
          request.timeMultiplier));
      response.error = "";
    } catch (const std::string& err) {
      ROS_ERROR_STREAM("MEDIA_SERVER: unable to load logged image list @ " << \
          request.firstImageFilename << " - " << err);
      response.error = err;
    }
    ROS_INFO_STREAM("MEDIA_SERVER: logged image list loaded @ " << \
        request.firstImageFilename);

    return true;
  };

  bool loadVideoFileCB( \
      media_server::LoadVideoFile::Request& request, \
      media_server::LoadVideoFile::Response& response) {
    try {
      updateSettings(request.timeMultiplier > 0 ? request.streamMode : false, \
          request.repeatMode);
      swapSource(new VideoFileSource(request.videoFilename, \
          request.timeMultiplier));
      response.error = "";
    } catch (const std::string& err) {
      ROS_ERROR_STREAM("MEDIA_SERVER: unable to load video file @ " << \
          request.videoFilename << " - " << err);
      response.error = err;
    }
    ROS_INFO_STREAM("MEDIA_SERVER: video file loaded @ " << \
        request.videoFilename);

    return true;
  };

  bool loadVideoDeviceCB( \
      media_server::LoadVideoDevice::Request& request, \
      media_server::LoadVideoDevice::Response& response) {
    try {
      updateSettings(false, false); // NOTE: even if streamMode is true, we must allow video device's callback to handle the image polls
      swapSource(new VideoDeviceSource(request.device, \
          request.enableDeinterlace, \
          (unsigned int) request.multipleGrabs, \
          request.streamMode, \
          std::bind1st(std::mem_fun(&MediaServer::streamedVideoDeviceCB), this), \
          request.framesPerSecond, \
          request.imageQualityPercent, \
          request.logImages, \
          request.logPathHeader));
      response.error = "";
    } catch (const std::string& err) {
      ROS_ERROR_STREAM("MEDIA_SERVER: unable to load video device @ " << \
          request.device << " - " << err);
      response.error = err;
    }
    ROS_INFO_STREAM("MEDIA_SERVER: video device loaded @ " << \
        request.device);

    return true;
  };

  bool getMediaStatusCB( \
      media_server::GetMediaStatus::Request& request, \
      media_server::GetMediaStatus::Response& response) {
    response.isActive = (source != NULL && source->isAlive());
    response.streamMode = streamMode;
    response.currImageID = -1;
    if (response.isActive) {
      response.timeMultiplier = source->getTimeMultiplier();
      std::pair<int, int> imageRange = source->getIndexRange();
      response.firstImageID = imageRange.first;
      response.lastImageID = imageRange.second;
      response.framesPerSecond = source->getFPS();
      response.sourceName = source->getName();

      if (source != NULL) {
        switch (source->getType()) {
        case InputSource::IMAGE_LIST_SOURCE:
          response.currImageID = ((ImageListSource*) source)->getImageID();
          break;
        case InputSource::LOGGED_IMAGE_LIST_SOURCE:
          response.currImageID = ((LoggedImageListSource*) source)->getImageID();
          break;
        case InputSource::VIDEO_FILE_SOURCE:
          response.currImageID = (int) ((VideoFileSource*) source)->getCVFileProperty(CV_CAP_PROP_POS_FRAMES);
          break;
        default:
          response.currImageID = -1;
          break;
        }
      }
    } else {
      response.timeMultiplier = 0;
      response.firstImageID = -1;
      response.lastImageID = -1;
      response.framesPerSecond = 0;
      response.sourceName = "undefined";
    }

    return true;
  };

  bool seekIndexCB( \
      media_server::SeekIndex::Request& request, \
      media_server::SeekIndex::Response& response) {
    response.result = false;
    if (source != NULL && source->isAlive()) {
      if (source->seek(request.ratio)) {
        response.result = true;
        inIdleMode = false;
        ROS_INFO_STREAM("MEDIA_SERVER: succeeded seeking to " << \
            request.ratio*100 << " %");
      } else {
        ROS_WARN("MEDIA_SERVER: seek_index failed because source is unseekable");
      }
    } else {
      ROS_WARN("MEDIA_SERVER: seek_index failed because source is inactive");
    }

    return true;
  };

  bool setTimeMultiplierCB( \
      media_server::SetTimeMultiplier::Request& request, \
      media_server::SetTimeMultiplier::Response& response) {
    response.resultTimeMultiplier = 0;
    if (source != NULL && source->isAlive()) {
      double prevTimeMultiplier = source->getTimeMultiplier();
      response.resultTimeMultiplier = source->setTimeMultiplier(request.newTimeMultiplier);
      if (prevTimeMultiplier == 0 && response.resultTimeMultiplier > 0 && \
          source->getType() != InputSource::VIDEO_DEVICE_SOURCE) {
        streamMode = true;
      } else if (prevTimeMultiplier > 0 && response.resultTimeMultiplier == 0) {
        streamMode = false;
      }
      ROS_INFO_STREAM("MEDIA_SERVER: succeeded setting time multiplier to " << \
            response.resultTimeMultiplier);
    } else {
      ROS_WARN("MEDIA_SERVER: set_time_multiplier failed because source is inactive");
    }

    return true;
  };

  bool pollFrameCB( \
      std_srvs::Empty::Request& request, \
      std_srvs::Empty::Response& response) {
    if (streamMode) {
      ROS_WARN("MEDIA_SERVER: poll_frame failed because server is in stream mode");
      return false;
    } else {
      if (!pushNextFrame()) {
        ROS_WARN("MEDIA_SERVER: could not poll frame from source");
        return false;
      }
    }
    return true;
  };

  void swapSource(InputSource* newSource) throw (const std::string&) {
    // Lock source access
    source_mutex.lock();

    // Get rid of previous input source
    if (source != NULL) {
      source->stopSource();
      delete source;
      source = NULL;
    }

    // Copy over new input source
    source = newSource;
    try {
      source->initSource();
      inIdleMode = false;
    } catch (const std::string& err) {
      // Release source access
      source_mutex.unlock();

      // Stop media server in case of error
      terminateMediaServer();

      // Re-throw error
      throw err;
    }

    // Release source access
    source_mutex.unlock();
  };

  void updateSettings(bool newStreamMode, bool newRepeatMode) {
    streamMode = newStreamMode;
    repeatMode = newRepeatMode;
  }

  void terminateMediaServer() {
    streamMode = false;
    ros::shutdown();
  };


private:
  bool pushNextFrame() {
    // If currently streaming from video device, then terminate immediately
    // to prevent clashing with callback
    if (source != NULL && source->getType() == InputSource::VIDEO_DEVICE_SOURCE && \
        ((VideoDeviceSource*) source)->isStreaming()) {
      return false;
    }

    bool hasFrame = false;
    bool result = false;

    if (source != NULL && source->isAlive()) {
      // REMINDER: getFrame() will block if time-synchronized
      image_mutex.lock();
      hasFrame = source->getFrame(imageBuf.image) && !imageBuf.image.empty();
      image_mutex.unlock();
      if (hasFrame) {
        image_mutex.lock();
        imageBuf.header.seq = imageSeqID++;
        double customDeltaSec = \
          (source->getType() == InputSource::VIDEO_FILE_SOURCE) ? \
          ((VideoFileSource*) source)->getCurrFrameTime() : \
          -1;
        if (customDeltaSec >= 0) {
          imageBuf.header.stamp = customStartTime + ros::Duration(customDeltaSec);
        } else {
          imageBuf.header.stamp = ros::Time::now();
        }
        getEncodingString(imageBuf.image, imageBuf.encoding);
        image_mutex.unlock();
      } else { // Something went wrong with getFrame
        // If repeat mode is on and seekable source depleted, then try rewind
        if (repeatMode) {
          if (source->seek(0.0)) {
            image_mutex.lock();
            hasFrame = source->getFrame(imageBuf.image) && !imageBuf.image.empty();
            image_mutex.unlock();
            if (hasFrame) {
              image_mutex.lock();
              double customDeltaSec = \
                (source->getType() == InputSource::VIDEO_FILE_SOURCE) ? \
                ((VideoFileSource*) source)->getCurrFrameTime() : \
                -1;
              if (customDeltaSec >= 0) {
                imageBuf.header.stamp = customStartTime + ros::Duration(customDeltaSec);
              } else {
                imageBuf.header.stamp = ros::Time::now();
              }
              imageBuf.header.seq = imageSeqID++;
              getEncodingString(imageBuf.image, imageBuf.encoding);
              image_mutex.unlock();
            } else {
              ROS_ERROR_STREAM("MEDIA_SERVER: could not get frame after rewinding media source " << \
                  source->getName());
            }
          } else {
            ROS_ERROR_STREAM("MEDIA_SERVER: could not rewind media source " << \
                source->getName());
          }
        } else if (!idleEnabled) { // Signal media server to terminate
          ROS_INFO("MEDIA_SERVER: source has no more frames; server terminating");
          terminateMediaServer();
        } else {
          if (!inIdleMode) {
            ROS_INFO("MEDIA_SERVER: source has no more frames; server is now in idle mode");
            inIdleMode = true;
          }
          result = true;
        }
      }
    } // if (source != NULL && source->isAlive())

    // Publish image
    if (hasFrame) {
      result = true;
      imagePub.publish(imageBuf.toImageMsg());
    }

    return result;
  };

  void streamedVideoDeviceCB(ImageData* d) {
    if (d == NULL || d->alive == NULL || !*(d->alive) || d->image == NULL || \
        d->image->empty()) {
      ROS_ERROR("MEDIA_SERVER: video device source callback failed");
    } else {
      image_mutex.lock();
      d->image->copyTo(imageBuf.image);
      imageBuf.header.seq = imageSeqID++;
      imageBuf.header.stamp = ros::Time::now();
      getEncodingString(imageBuf.image, imageBuf.encoding);
      image_mutex.unlock();

      // Only publish (which might take time) if source is still alive;
      // otherwise callback might drag on thread for too long
      if (d->alive) {
        imagePub.publish(imageBuf.toImageMsg());
      }
    }
  };

  // NOTE: Assume that all OpenCV Mat color images are encoded as BGR
  static void getEncodingString(cv::Mat image, std::string& encodingBuf) {
    switch (image.type()) {
    case CV_8UC1:
      encodingBuf = sensor_msgs::image_encodings::MONO8;
      break;
    case CV_16UC1:
      encodingBuf = sensor_msgs::image_encodings::MONO16;
      break;
    case CV_8UC3:
      encodingBuf = sensor_msgs::image_encodings::BGR8;
      break;
    case CV_8UC4:
      encodingBuf = sensor_msgs::image_encodings::BGRA8;
      break;
    default:
      encodingBuf = "";
      break;
    }
  };

  image_transport::Publisher imagePub;
  ros::ServiceServer loadImageListSrv;
  ros::ServiceServer loadLoggedImageListSrv;
  ros::ServiceServer loadVideoFileSrv;
  ros::ServiceServer loadVideoDeviceSrv;
  ros::ServiceServer getMediaStatusSrv;
  ros::ServiceServer seekIndexSrv;
  ros::ServiceServer setTimeMultiplierSrv;
  ros::ServiceServer pollFrameSrv;

  cv_bridge::CvImage imageBuf;
  boost::mutex image_mutex;
  unsigned int imageSeqID;
  InputSource* source;
  boost::mutex source_mutex;
  bool idleEnabled; // If false, node will terminate after failing to fetch frame
  bool inIdleMode;
  bool streamMode;
  bool repeatMode;
  
  ros::Time customStartTime;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "media_server", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  if (nh.resolveName("image") == "/image") {
    ROS_WARN_STREAM("Usage: " << argv[0] << \
        " image:=<image topic> [idleAfterSourceDone customStartTime]");
  }

  bool idleAfterSourceDone = (argc > 1) ? argv[1][0] != '0' : false;
  double customStartSec = (argc > 2) ? atof(argv[2]) : -1.0;
  MediaServer mediaServer(nh, idleAfterSourceDone, customStartSec);
  mediaServer.spin();

  return 0;
};

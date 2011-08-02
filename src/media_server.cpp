#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include "input/StubSource.hpp"
#include "input/ImageListSource.hpp"
#include "input/LoggedImageListSource.hpp"
#include "input/VideoFileSource.hpp"
#include "input/VideoDeviceSource.hpp"
#include "media_server/LoadImageList.h"
#include "media_server/LoadLoggedImageList.h"
#include "media_server/LoadVideoFile.h"
#include "media_server/LoadVideoDevice.h"
#include "media_server/GetMediaStatus.h"
#include "media_server/SeekIndex.h"
#include "media_server/SetTimeMultiplier.h"


using namespace std;
using namespace input;


class MediaServer {
public:
  MediaServer(ros::NodeHandle& nh, \
      const std::string& transport, \
      bool idle) : \
      source(NULL), transportString(transport), \
      idleEnabled(idle), streamMode(false), isStreamAlive(false) {
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

    ROS_INFO("Initialized idle media_server on image topic %s", \
        image_topic.c_str());
  };

  void spin() {
    // Main loop body
    ros::Rate sleepRate(200); // 200Hz should be sufficient for regular usage of poll mode
    while (ros::ok()) {
      if (streamMode && isStreamAlive) {
        // TODO: getFrame (which should block) [UNLESS logged video device mode)
      } else {
        isStreamAlive = false;
        sleepRate.sleep();
        ros::spinOnce();
      }
    }

    // Clean up
    if (source != NULL) {
      delete source;
      source = NULL;
    }
    ROS_INFO("Terminated media_server");
    ros::shutdown();
    return;
  };

  bool loadImageListCB( \
      media_server::LoadImageList::Request& request, \
      media_server::LoadImageList::Response& response) {
    // TODO: swap input source, then potentially start stream loop
    // TODO: ROS_INFO warn user of result
    return true;
  };

  bool loadLoggedImageListCB( \
      media_server::LoadLoggedImageList::Request& request, \
      media_server::LoadLoggedImageList::Response& response) {
    // TODO: swap input source, then potentially start stream loop
    // TODO: ROS_INFO warn user of result
    return true;
  };

  bool loadVideoFileCB( \
      media_server::LoadVideoFile::Request& request, \
      media_server::LoadVideoFile::Response& response) {
    // TODO: swap input source, then potentially start stream loop
    // TODO: ROS_INFO warn user of result
    return true;
  };

  bool loadVideoDeviceCB( \
      media_server::LoadVideoDevice::Request& request, \
      media_server::LoadVideoDevice::Response& response) {
    // TODO: swap input source, then potentially start stream loop
    // TODO: ROS_INFO warn user of result
    return true;
  };

  bool getMediaStatusCB( \
      media_server::GetMediaStatus::Request& request, \
      media_server::GetMediaStatus::Response& response) {
    // TODO: compile response (with default vars; then fill in when appropriate using switch)
    return true;
  };

  bool seekIndexCB( \
      media_server::SeekIndex::Request& request, \
      media_server::SeekIndex::Response& response) {
    // TODO: if source is alive, seek index
    // TODO: ROS_INFO warn user of result
    return true;
  };

  bool setTimeMultiplierCB( \
      media_server::SetTimeMultiplier::Request& request, \
      media_server::SetTimeMultiplier::Response& response) {
    // TODO: if source is alive, change its multiplier
    // TODO: ROS_INFO warn user of result
    return true;
  };

  bool pollFrameCB( \
      std_srvs::Empty::Request& request, \
      std_srvs::Empty::Response& response) {
    if (streamMode) {
      ROS_WARN("poll_frame failed because media_server is in stream mode");
      return false;
    } else {
      return pushNextFrame();
    }
  };

private:
  bool pushNextFrame() {
    // TODO: if source is alive, getFrame() and push to topic. If failed, consider terminating ros node
    return false;
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

  InputSource* source;
  string transportString;
  bool idleEnabled; // If false, node will terminate after failing to fetch frame
  bool streamMode;
  bool isStreamAlive;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "media_server", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  if (nh.resolveName("image") == "/image") {
    ROS_WARN_STREAM("Usage: " << argv[0] << " image:=<image topic> [transport] [idle]");
  }

  MediaServer mediaServer(nh, \
      (argc > 1) ? argv[1] : "raw", \
      (argc > 2) ? argv[2][0] != '0' : false);
  mediaServer.spin();

  return 0;
};

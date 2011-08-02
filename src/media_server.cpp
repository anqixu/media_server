#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include "input/StubSource.hpp"
#include "input/ImageListSource.hpp"
#include "input/LoggedImageListSource.hpp"
#include "input/VideoFileSource.hpp"
#include "input/VideoDeviceSource.hpp"


using namespace std;
using namespace input;


class MediaServer {
public:
  MediaServer(const ros::NodeHandle& nh, const std::string& transport) : \
      source(NULL) {
    std::string topic = nh.resolveName("image");
    ros::NodeHandle local_nh("~");

    ROS_INFO("The transport is %s", transport.c_str());
    ROS_INFO("The topic is %s", topic.c_str());
    ROS_INFO("The current time is %d", ros::Time::now().sec);
    ros::shutdown();
  };

private:
  InputSource* source;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "media_server", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  if (n.resolveName("image") == "/image") {
    ROS_WARN_STREAM("Usage: " << argv[0] << " image:=<image topic> [transport]");
  }

  MediaServer media(n, (argc > 1) ? argv[1] : "raw");

  ros::spin();

  return 0;
};

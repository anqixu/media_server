# media_server

An OpenCV-based ROS node for serving images from different sources, including:
- sequential list of images (format: `[HEADER]_[IMAGE_ID].[EXTENSION]`)
- sequential list of images with time-synchronization log file (log file format: `[HEADER]_[FIRST_IMAGE_ID].log`)
- video file
- video device (must be supported by OpenCV)


## KEY FEATURES

* either have frames streamed on a ROS topic automatically (while time-synchronized),
* or poll individual frames manually using a ROS service call
* in poll mode, frames can either be polled while time-synchronized or on a frame-per-frame basis (not applicable for VideoDeviceSource)
* dynamically adjust the time synchronization multiplier using a ROS service call
* seek to specific position-ratio (for seekable media sources only)
* set the media source to repeat (not applicable for VideoDeviceSource)
* create a logged sequence of images while streaming from a video device source


## DEPENDENCIES
libboost-date-time-dev, libboost-filesystem-dev, libboost-system-dev, libboost-thread-dev, libopencv-dev, libexiv2-dev


## TEST INSTRUCTIONS
Here are brief instructions for viewing the sample video file using media_server
and image_view. On 4 separate terminals, run:

* TERM_1 > `roscore`
* TERM_2 > `rosrun rqt_image_view rqt_image_view` and select `/source/image_raw`
* TERM_3 > `rosrun media_server media_server image:=/source/image_raw`
      (optional parameters: ['raw'|'theora'|'compressed'] [ROS_NODE_IDLE_AFTER_SOURCE_IS_DONE])
* TERM_4 > `rosservice call load_video_file true [PATH_TO_media_server]/input/sample.avi 1.0 1`
      (the parameters for LoadVideoFile.srv are: [STREAM_MODE] [VIDEO_FILE] [TIME_MULTIPLIER] [REPEAT])

Info on other available services can be found by running:
* \> `rosservice list`
* \> `rosservice type [SERVICE_NAME] | rossrv show`

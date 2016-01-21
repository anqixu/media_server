#! /usr/bin/env python

import sys
import rospy
from media_server.srv import *

def main():
  if len(sys.argv) < 2:
    print 'Usage: %s VIDEO_PATH (MULTIPLIER)' % sys.argv[0]
    return -1
  
  streamMode = True
  videoFilename = sys.argv[1]
  timeMultiplier = 1.0
  repeatMode = False
  if len(sys.argv) > 2:
    timeMultiplier = float(sys.argv[2])
  
  rospy.wait_for_service('/load_video_file')
  try:
    loadVideoFileCln = rospy.ServiceProxy('/load_video_file', LoadVideoFile)
    resp = loadVideoFileCln(streamMode, videoFilename, timeMultiplier, repeatMode)
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e
  return 0

if __name__ == "__main__":
  sys.exit(main())

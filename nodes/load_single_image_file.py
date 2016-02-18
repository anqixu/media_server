#! /usr/bin/env python

import sys
import rospy
import os
import shutil
from media_server.srv import *

def main():
  if len(sys.argv) < 2:
    print 'Usage: %s IMAGE_PATH (FPS)' % sys.argv[0]
    return -1
  
  sourceFilename = sys.argv[1]
  streamMode = True
  firstImageFilename = '/tmp/image_001' + sourceFilename[sourceFilename.rfind('.'):] # node expects format: [folder]/[header]_[image_ID].[extension]
  shutil.copy(sourceFilename, firstImageFilename)
  framesPerSecond = 10.0
  repeatMode = True
  if len(sys.argv) > 2:
    framesPerSecond = float(sys.argv[2])
  
  rospy.wait_for_service('/load_image_list')
  try:
    loadImageListCln = rospy.ServiceProxy('/load_image_list', LoadImageList)
    resp = loadImageListCln(streamMode, firstImageFilename, framesPerSecond, repeatMode)
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e
  return 0

if __name__ == "__main__":
  sys.exit(main())

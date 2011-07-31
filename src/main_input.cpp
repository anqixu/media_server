#include <iostream>
#include "StubSource.hpp"
#include "VideoFileSource.hpp"
#include "VideoDeviceSource.hpp"
#include "LoggedImageListSource.hpp"
#include "ImageListSource.hpp"


using namespace std;
using namespace input;


class CallbackClass {
public:
  void callback(ImageData d) {
    unsigned int count = 0;
    double value = 0;
    cv::MatIterator_<unsigned char> it;
    for (it = d.image->begin<unsigned char>(); \
    it != d.image->end<unsigned char>(); it++) {
      value += *it;
      count++;
    }
    cout << "callback() - w=" << d.image->cols << " h=" << d.image->rows << \
        " nChan=" << d.image->channels() << " Avg Value: " << \
        ((count == 0) ? 0 : value/count) << endl;
    if (d.telem != NULL) {
      cout << "... (with telemetry packet)" << endl;
    }
    if (d.nav != NULL) {
      cout << "... (with navigation packet)" << endl;
    }
  };
};


int main (int argc, char **argv) {
  ///////////////// BEGIN USER-EDITABLE SETTINGS //////////////
  // TEST_MODE VALUES:
  // 0: Stub Source (i.e. always returns black image) [DEFAULT]
  // 1: Video File Source
  // 2: Video Capture Source, logger disabled
  // 3: Video Capture Source, logger enabled
  // 4: Logged Image List Source
  //    [MUST HAVE IMAGES AND LOG FILE]
  // 5. Image List Source
  //    [MUST HAVE IMAGES]
  //
  // USAGE SCENARIOS:
  // > UAV_INPUT 0
  // > UAV_INPUT 1 [isTimeSynched] [videoFile]
  // > UAV_INPUT 2 [isTimeSynched] [videoDevice]
  // > UAV_INPUT 3 [videoDeviceFPS] [videoDevice] [logLocationHeader]
  // > UAV_INPUT 4 [isTimeSynched] [firstImageFilename]
  // > UAV_INPUT 5 [imageListFPS] [firstImageFilename]
  int testMode = 0;

  // NOTE: isTimeSynched does nothing if source is video capture or image list
  bool isTimeSynched = true;
  double imageListFPS = 15.0; // if = 0, time sync will be disabled

  string videoFile = "./sample.avi";

#ifdef __MSVC__ /* Microsoft Visual Studio */
  int videoDeviceType = CV_CAP_ANY;
#else
  int videoDeviceType = CV_CAP_V4L2;
#endif
  string logLocationHeader = "./log/log"; // = filename without _#####.jpg
  double videoDeviceFPS = 15.0;
  unsigned int imageQualityPercentage = 95;
  bool deinterlaceImage = true;

  string firstImageFilename = "./log/log_00000.jpg";
  ///////////////// END USER-EDITABLE SETTINGS ////////////////

  // Process command line arguments
  if (argc > 1) {
    if (argv[1][0] == '-' && argv[1][1] == 'h') {
      cout << "Sources:" << endl;
      cout << "0: Stub Source (i.e. always returns black image) [DEFAULT]" << endl;
      cout << "1: Video File Source" << endl;
      cout << "2: Video Capture Source, logger disabled" << endl;
      cout << "3: Video Capture Source, logger enabled" << endl;
      cout << "4: Logged Image List Source" << endl;
      cout << "   [MUST HAVE IMAGES AND LOG FILE]" << endl;
      cout << "5. Image List Source" << endl;
      cout << "   [MUST HAVE IMAGES]" << endl;
      cout << endl;
      cout << "Usage:" << endl;
      cout << argv[0] << " 0" << endl;
      cout << argv[0] << " 1 [isTimeSynched] [videoFile]" << endl;
      cout << argv[0] << " 2 [isTimeSynched] [videoDeviceID]" << endl;
      cout << argv[0] << " 3 [videoDeviceFPS] [videoDeviceID] [logLocationHeader]" << endl;
      cout << argv[0] << " 4 [isTimeSynched] [firstImageFilename]" << endl;
      cout << argv[0] << " 5 [imageListFPS] [firstImageFilename]" << endl;
      return 0;
    } else {
      testMode = atoi(argv[1]);
      cout << ". testMode manually set to: " << \
          testMode << endl;
    }
  }
  if (argc > 2) {
    double userFPS = atof(argv[2]);
    if (userFPS >= 0) {
      switch (testMode) {
      case 3:
        videoDeviceFPS = userFPS;
        isTimeSynched = true;
        cout << ". videoDeviceFPS manually set to: " << \
            videoDeviceFPS << endl;
        break;
      case 5:
        imageListFPS = userFPS;
        isTimeSynched = (imageListFPS != 0.0);
        cout << ". imageListFPS manually set to: " << \
            imageListFPS << endl;
        break;
      default:
        isTimeSynched = (userFPS != 0.0);
        cout << ". isTimeSynched manually set to: " << \
            isTimeSynched << endl;
        break;
      }
    }
  }
  if (argc > 3) {
    string userString = string(argv[3]);
    switch (testMode) {
    case 1:
      videoFile = userString;
      cout << ". videoFile manually set to: " << videoFile << endl;
      break;
    case 2:
    case 3:
      videoDeviceType = atoi(argv[3]);
      cout << ". videoDeviceType manually set to: " << \
          videoDeviceType << endl;
      break;
    case 4:
    case 5:
      firstImageFilename = userString;
      cout << ". firstImageFilename manually set to: " << \
          firstImageFilename << endl;
      break;
    }
  }
  if (argc > 4) {
    switch (testMode) {
    case 3:
      logLocationHeader = argv[4];
      cout << ". logLocationHeader manually set to: " << \
          logLocationHeader << endl;
      break;
    }
  }

  // Define local variables
  InputSource* src = NULL;
  cv::Mat imgBuf;
  int key;
  VCDriver* VC = NULL;
  int keyPressDelay = isTimeSynched ? 30 : 0;
  logTelem telemBuf;
  CallbackClass cbc;


  try {
    // Initialize input source
    switch(testMode) {
    case 1:
      src = new VideoFileSource(videoFile, isTimeSynched);
      break;
    case 2:
      src = new VideoDeviceSource(videoDeviceType, false);
      break;
    case 3:
      src = new VideoDeviceSource(videoDeviceType, deinterlaceImage, 1, true, \
          std::bind1st(std::mem_fun(&CallbackClass::callback), &cbc), \
          videoDeviceFPS, imageQualityPercentage, true, logLocationHeader, VC);
      break;
    case 4:
      src = new LoggedImageListSource(firstImageFilename, isTimeSynched);
      break;
    case 5:
      src = new ImageListSource(firstImageFilename, imageListFPS);
      break;
    default:
      cout << "! Unrecognized testMode: " << testMode << endl;
      src = new StubSource();
      break;
    }

    if (src == NULL) {
      throw string("src is NULL! Check code in main.cpp");
    }
    src->initSource();
    cout << ". input source initialized." << endl;
    cout << ". keypress interactions:" << endl;
    cout << ". 'X'  : exit" << endl;
    cout << ". '+'  : time multiplier * 1.1" << endl;
    cout << ". '-'  : time multiplier / 1.1" << endl;
    cout << ". '#'  : manually set time multiplier (# = 0-9)" << endl;
    cout << ". NOTE: all key presses except for 'X' will grab next frame" << endl;
    cout << flush;

    // Print image range if logged image list source
    if (src->getType() == InputSource::LOGGED_IMAGE_LIST_SOURCE) {
      LoggedImageListSource* s = (LoggedImageListSource*) src;
      pair<int, int> range = s->getImageRange();
      cout << ". Logged image range: " << range.first << " to " << \
          range.second << endl;
      //s->setImageID(500);
    }

    while(1) {
      // Query and display latest frame
      if (!src->getFrame(imgBuf)) {
        throw string("Unable to obtain frame");
      }
      cv::imshow("Frame", imgBuf);

      // Print telemetry if available
      if (src->getType() == InputSource::LOGGED_IMAGE_LIST_SOURCE) {
        LoggedImageListSource* s = (LoggedImageListSource*) src;
	// TODO: print image ID
      }

      // Wait for user to press key, either to close app or to grab frame
      key = (cv::waitKey(keyPressDelay) & 0xFF);
      if (key == 'x' || key == 'X' || key == 'q' || key == 'Q') {
        break;
      } else if (key >= '0' && key <= '9') {
        src->setTimeMultiplier(key - '0');
        cout << ". Multiplier: " << src->getTimeMultiplier() << endl;
      } else if (key == '+') {
        src->setTimeMultiplier(src->getTimeMultiplier()*1.1);
        cout << ". Multiplier: " << src->getTimeMultiplier() << endl;
      } else if (key == '-') {
        src->setTimeMultiplier(src->getTimeMultiplier()/1.1);
        cout << ". Multiplier: " << src->getTimeMultiplier() << endl;
      }
    }
  } catch (const std::string& err) {
    cout << "ERROR > " << err << endl;
  }

  if (VC != NULL) {
    delete VC;
  }

  if (src != NULL) {
    delete src;
  }

  return 0;
};


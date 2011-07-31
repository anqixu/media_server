/**
 * @file StubSource.cpp
 * @author Anqi Xu
 */


#include "StubSource.h"


using namespace std;
using namespace input;


StubSource::StubSource(unsigned int imgWidth, unsigned int imgHeight) : \
    InputSource(0) {
  type = STUB_SOURCE;

  if (imgWidth > 0) {
    width = imgWidth;
  } else {
    width = DEFAULT_IMG_WIDTH;
  }

  if (imgHeight > 0) {
    height = imgHeight;
  } else {
    height = DEFAULT_IMG_HEIGHT;
  }

  stubImg.create(height, width, DEFAULT_IMG_TYPE);
};


StubSource::~StubSource() {
};


void StubSource::initSource() throw (const std::string&) {
  alive = true;
};


void StubSource::stopSource() {
  alive = false;
};


bool StubSource::getFrame(cv::Mat& userBuf) {
  userBuf = stubImg.clone();
  return true;
};


void StubSource::setWidth(unsigned int newImgWidth) {
  if (newImgWidth > 0) {
    width = newImgWidth;
  }

  stubImg.create(height, width, DEFAULT_IMG_TYPE);
};


void StubSource::setHeight(unsigned int newImgHeight) {
  if (newImgHeight > 0) {
    height = newImgHeight;
  }

  stubImg.create(height, width, DEFAULT_IMG_TYPE);
};

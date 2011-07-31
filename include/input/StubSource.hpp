/**
 * @file StubSource.hpp
 * @author Anqi Xu
 */


#ifndef STUBSOURCE_HPP_
#define STUBSOURCE_HPP_


#include "InputSource.hpp"

namespace input {

class StubSource : public InputSource {
public:
  StubSource(unsigned int imgWidth = DEFAULT_IMG_WIDTH, \
      unsigned int imgHeight = DEFAULT_IMG_HEIGHT);
  ~StubSource();

  void initSource() throw (const std::string&);
  void stopSource();
  bool getFrame(cv::Mat& userBuf);
  void setWidth(unsigned int newImgWidth);
  void setHeight(unsigned int newImgHeight);

  const static unsigned int DEFAULT_IMG_WIDTH = 640;
  const static unsigned int DEFAULT_IMG_HEIGHT = 480;
  const static int DEFAULT_IMG_TYPE = CV_8UC3;

private:
  cv::Mat stubImg;
};

} // namespace input


#endif /* STUBSOURCE_HPP_ */

/**
 * @file InputSource.cpp
 * @author Anqi Xu
 */


#include "InputSource.hpp"
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/lexical_cast.hpp>


using namespace std;
namespace fs = boost::filesystem;
using namespace input;


void InputSource::parseImageFileHeader(const string& firstImageFilename, \
    string& headerBuf, string& extensionBuf, int& firstImageIDBuf, \
    unsigned int& numDigitsBuf) throw (const std::string&) {
  int tempInt, charIndex;

  try {
    // Obtain absolute path and file extension
    fs::path filepath = fs::system_complete(fs::path(firstImageFilename));
    extensionBuf = filepath.extension();

    // Determine filepath header location by skipping extension & numbers
    headerBuf = filepath.string();
    charIndex = headerBuf.length() - 1 - extensionBuf.length();
    numDigitsBuf = 0;
    while (charIndex >= 0 && headerBuf[charIndex] >= '0' && \
        headerBuf[charIndex] <= '9') {
      charIndex--;
      numDigitsBuf++;
    }
    if (charIndex < 0 || numDigitsBuf <= 0) {
      throw string("Could not parse image ID");
    }
    charIndex++;

    // Parse image ID and set filepath header
    //tempInt = atoi(headerBuf.substr(charIndex, numDigitsBuf).c_str());
    tempInt = boost::lexical_cast<int>(headerBuf.substr(charIndex, numDigitsBuf));
    if (tempInt < 0) {
      throw string("Found negative image ID");
    }
    firstImageIDBuf = (unsigned int) tempInt;
    headerBuf = headerBuf.substr(0, charIndex);
  } catch (const std::exception& ex) {
    ostringstream err;
    err << "Could not parse image filename: " << ex.what();
    throw err.str();
  }
};


void InputSource::setTimeMultiplier(double newMult) {
  // Update elapsed run-time till current instant using previous multiplier
  if (alive && hasStartTime && timeMultiplier > 0) {
    ptime currTime = microsec_clock::local_time();
    time_duration td = currTime - prevTime;
    // NOTE: following expression can't be simplified due to cast in time_duration::operator*(int)
    elapsedTime += microseconds((long) ((float) td.total_microseconds() * timeMultiplier));
    prevTime = currTime;
  }

  // Update multiplier
  timeMultiplier = (newMult > 0) ? newMult : 0;
};

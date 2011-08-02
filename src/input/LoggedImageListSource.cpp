/**
 * @file LoggedImageListSource.cpp
 * @author Anqi Xu
 */


#include "LoggedImageListSource.hpp"
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <iostream>


using namespace std;
using namespace input;


const string LoggedImageListSource::LOGFILE_EXTENSION = ".log";


LoggedImageListSource::LoggedImageListSource( \
    const std::string& firstImageFilename, double timeMult) : \
    InputSource(timeMult > 0 ? timeMult : 0), \
    inputFilename(firstImageFilename), \
    numDigitsInFilename(0), fileID(0), line(), \
    firstImageID(-1), lastImageID(-1) {
  type = LOGGED_IMAGE_LIST_SOURCE;
};


LoggedImageListSource::~LoggedImageListSource() {
  stopSource();
};


void LoggedImageListSource::initSource() throw (const std::string&) {
  // Clean up any previously-opened sources
  stopSource();

  // Parse user-specified filename for header & numbering format & extension
  InputSource::parseImageFileHeader(inputFilename, fileHeader, \
      fileExtension, firstImageID, numDigitsInFilename);

  // Form log filename and open log file
  ostringstream logFilename;
  logFilename << fileHeader << setfill('0') << \
      setw(numDigitsInFilename) << firstImageID << LOGFILE_EXTENSION;
  if (logFile.is_open()) { logFile.close(); }
  logFile.open(logFilename.str().c_str(), ios::in);
  if (!logFile.is_open()) {
    ostringstream err;
    err << "Could not find log file: " << logFilename.str();
    throw err.str();
  }

  // Parse time counts inside timing file
  timeIndicesUSEC.clear();
  int currImageID, prevImageID = -1;
  string currTimeString;
  ptime currTime;
  time_duration td;
  hasStartTime = false;
  lastImageID = -1;
  while (!logFile.eof()) {
    // Skip blank lines or lines with comments
    getline(logFile, line);
    boost::trim(line);
    if (line.length() <= 0 || line[0] == '%') {
      continue;
    }

    // Parse image ID and time index
    istringstream lineStream(line);
    lineStream >> currImageID >> currTimeString;

    // NOTE: during loading, the variables hasStartTime, startTime, currTime
    //       are used temporarily with different meanings; for instance
    //       hasStartTime represents the start time in the image list within
    //       the log file, and thus should NOT be confused with its usage in
    //       getFrame() (see end of this function)
    if (hasStartTime) {
      // Make sure imageID is contiguous
      if (currImageID - prevImageID != 1) {
        if (logFile.is_open()) { logFile.close(); }
        alive = false;
        ostringstream err;
        err << "Found non-contiguous imageID sequence between # " << \
            prevImageID << " & " << currImageID << " in log file";
        throw err.str();
      }
      prevImageID = currImageID;
      lastImageID = currImageID;

      // Compute and store elapsed duration
      currTime = from_iso_string(currTimeString);
      td = currTime - startTime;
      timeIndicesUSEC.push_back((long) td.total_microseconds());
    } else { // First entry: make sure imageID = file's image ID
      if (currImageID != firstImageID) {
        if (logFile.is_open()) { logFile.close(); }
        ostringstream err;
        err << "Image ID in filename (" << firstImageID << \
            ") does not correspond to first image ID in log (" << \
            currImageID << ")";
        throw err.str();
      }
      prevImageID = currImageID;
      startTime = from_iso_string(currTimeString);
      timeIndicesUSEC.push_back(0);
      hasStartTime = true;
      lastImageID = currImageID;
    }
  } // while (!logFile.eof())

  // Rewind file to beginning in preparations for reading telemetry
  logFile.clear(); // This clears the internal flags
  logFile.seekg(0, ios::beg);
  line.clear();

  // Update status
  fileID = -1; // Not 0, since we start getFrame() by incrementing fileID
  alive = true;
  hasStartTime = false; // NOTE: startTime's usage in THIS function is different!
};


void LoggedImageListSource::stopSource() {
  timeIndicesUSEC.clear();

  if (logFile.is_open()) {
    logFile.close();
  }

  alive = false;
  if (timeMultiplier < 0) { timeMultiplier = 0; }
  hasStartTime = false;
  firstImageID = -1;
  lastImageID = -1;
  fileID = -1;
  line.clear();
};


bool LoggedImageListSource::getFrame(cv::Mat& userBuf) {
  if (timeMultiplier > 0) {
    if (!hasStartTime || fileID < 0) { // First getFrame() call
      fileID = 0; // Recall that current image ID = fileID + firstImageID
      startTime = microsec_clock::local_time();
      hasStartTime = true;
      prevTime = startTime;
      elapsedTime = seconds(0);
    } else {
      // If no more frames, then terminate immediately
      if ((unsigned int) fileID + 1 >= timeIndicesUSEC.size()) {
        return false;
      }

      // Compute elapsed time since first (not a typo) getFrame() call
      ptime currTime = microsec_clock::local_time();
      time_duration td = currTime - prevTime;
      // NOTE: following expression can't be simplified due to cast in time_duration::operator*(int)
      elapsedTime += microseconds((long) ((float) td.total_microseconds() * timeMultiplier)); // Note the += ...
      prevTime = currTime; // (thus duration computed from first getFrame() call)
      // NOTE: elapsedTime is within the image-list-time frame, thus
      //       any updates to this variable need to consider timeMultiplier

      fileID++; // Update file index to next frame
      long timeGapUSEC = \
          (long) (timeIndicesUSEC[fileID] - elapsedTime.total_microseconds());
      if (timeGapUSEC > 0) { // If next frame is not available yet, then sleep a bit
        // NOTE: the previous duration is computed in frame-time and not
        //       wall-time, thus we need to factor out timeMultiplier
        //       when using it in real space (a.k.a. wall time)
        timeGapUSEC = (long) floor(timeGapUSEC / timeMultiplier);

        boost::this_thread::sleep(boost::posix_time::microseconds(timeGapUSEC));
      } else { // Else skip over frames until time gap is longer than elapsed time
        while ((unsigned int) fileID < timeIndicesUSEC.size() && \
            (timeIndicesUSEC[fileID] < elapsedTime.total_microseconds())) {
          fileID++;
        }

        // If no more frames, then return false
        if ((unsigned int) fileID >= timeIndicesUSEC.size()) {
          return false;
        } else { // Sleep until it's time to display next image
          // Begin by updating time
          // (difference should be insignificant, but be cautious nevertheless)
          currTime = microsec_clock::local_time();
          time_duration td = currTime - prevTime;
          // NOTE: following expression can't be simplified due to cast in time_duration::operator*(int)
          elapsedTime += microseconds((long) ((float) td.total_microseconds() * timeMultiplier));
          prevTime = currTime;

          timeGapUSEC = \
              (long) floor( (timeIndicesUSEC[fileID] - \
                  elapsedTime.total_microseconds()) / timeMultiplier );
          boost::this_thread::sleep(boost::posix_time::microseconds(timeGapUSEC));
        }
      }
    }
  } else {
    fileID++;
  }

  // Increment numDigits if necessary
  unsigned int newDigitCount = \
      (unsigned int) floor(log10((double) firstImageID + fileID)) + 1;
  if (newDigitCount > numDigitsInFilename) {
    numDigitsInFilename = newDigitCount;
  }

  // Load image directly into caller's buffer, and then update image dimensions
  ostringstream imageFilename;
  imageFilename << fileHeader << setfill('0') << \
      setw(numDigitsInFilename) << firstImageID + fileID << fileExtension;
  userBuf = cv::imread(imageFilename.str()); // Assume format is BGR
  if (userBuf.empty()) { // Image failed to load
    return false;
  }
  width = userBuf.rows;
  height = userBuf.cols;

  return true;
};


bool LoggedImageListSource::getTelem(logTelem* buf) {
  if (buf == NULL) {
    return false;
  }

  try {
    seekForFileID();
  } catch(const std::string& err) {
    //cout << "ERROR > " << err << endl;
    return false;
  }

  string iso_string;
  istringstream lineStream(line);
  lineStream >> buf->image_ID;

  // Check for error flags prior to last entry
  if (!lineStream.good()) { // NOTE: can use good() here since don't expect EOF yet
    return false;
  }

  // Parse last entry and then check for error flags
  lineStream >> iso_string;
  if ((!lineStream.rdstate() & (ifstream::failbit | ifstream::badbit)) != 0) {
    return false;
  }

  // Parse system time into numeric format
  buf->sys_time = from_iso_string(iso_string);

  return true;
};


void LoggedImageListSource::setImageID(int desiredID) throw (const std::string&) {
  // Check for boundary conditions
  if (!alive) {
    throw string("Logged image list source has not been initiated.");
  }
  if (desiredID < firstImageID || desiredID > lastImageID) {
    ostringstream err;
    err << "Specified Image ID (" << desiredID << ") is out of bounds (" << \
        firstImageID << " - " << lastImageID << ")";
    throw err.str();
  }

  if (desiredID == firstImageID + fileID) { // Already at current position
    return;
  }

  // Set desired file ID and re-seek for telemetry entry
  int newFileID = desiredID - firstImageID;
  // Rewind file if necessary
  if (newFileID < fileID) {
    logFile.clear();
    logFile.seekg(0, ios::beg);
  }
  fileID = newFileID;
  seekForFileID();

  // Re-synchronize elapsed duration
  hasStartTime = true;
  startTime = microsec_clock::local_time();
  prevTime = startTime;
  elapsedTime = microseconds(timeIndicesUSEC[fileID]);
};


bool LoggedImageListSource::seek(double ratio) {
  ratio = std::min(std::max(ratio, 0.0), 1.0);
  bool result = false;
  if (alive) {
    result = true;
    int newFileID = (int) round((lastImageID - firstImageID)*ratio);
    if (newFileID < 0 || \
        newFileID > lastImageID - firstImageID) {
      newFileID = 0;
    }

    // Rewind file if necessary
    if (newFileID < fileID) {
      logFile.clear();
      logFile.seekg(0, ios::beg);
    }
    fileID = newFileID;
    seekForFileID();

    if (timeMultiplier > 0) {
      prevTime = microsec_clock::local_time();
      elapsedTime = microseconds(timeIndicesUSEC[fileID]);
      hasStartTime = true;
      startTime = prevTime - elapsedTime;
    }
  }
  return result;
};


void LoggedImageListSource::seekForFileID() throw (const std::string&) {
  int currImageID;

  // Check if seeked entry is either equals or smaller than the current
  // line in the log file
  if (line.length() > 0 && line[0] != '%') {
    istringstream lineStream(line);
    lineStream >> currImageID;
    if (currImageID == firstImageID + fileID) {
      return;
    } else if (currImageID > firstImageID + fileID) {
      stopSource();
      ostringstream err;
      err << "Image ID in log (" << currImageID << \
          ") is larger than expected image ID (" << \
          firstImageID + fileID << ")";
      throw err.str();
    }
  }

  // The seeked entry is larger than the current line in the log file, hence
  // iteratively seek for the correct line in the log file
  while (!logFile.eof()) {
    // Skip blank lines or lines with comments
    getline(logFile, line);
    boost::trim(line);
    if (line.length() <= 0 || line[0] == '%') {
      continue;
    }

    istringstream lineStream(line);
    lineStream >> currImageID;
    if (currImageID == firstImageID + fileID) {
      return;
    } else if (currImageID > firstImageID + fileID) {
      ostringstream err;
      err << "Image ID in log (" << currImageID << \
          ") is larger than expected image ID (" << \
          firstImageID + fileID << ")";
      throw err.str();
    }
  }

  // If execution arrives here then throw error
  ostringstream err;
  err << "Could not find image ID (" << firstImageID + fileID << \
        ") in log file.";
  throw err.str();
};

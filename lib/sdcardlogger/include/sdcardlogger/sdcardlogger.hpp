#ifndef SDCARDLOGGER_HPP
#define SDCARDLOGGER_HPP

#include "SdFat.h"
#include "RingBuf.h"
#include <Arduino.h>
#include <vector>
#include <string>

// Use Teensy SDIO
#define SD_CONFIG  SdioConfig(FIFO_SDIO)

// Interval between points for 25 ksps.
#define LOG_INTERVAL_USEC 40

// Size to log 10 byte lines at 25 kHz for more than ten minutes.
#define LOG_FILE_SIZE 10*25000*600  // 150,000,000 bytes.

// Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
#define RING_BUF_CAPACITY 400*512

class SDcardLogger
{
  public:
  SDcardLogger(const char* filename);
  void sdcard_setup();
  bool checkMaxSize();
  void dump2sdcard(std::vector<std::string> data);
  void dump2sdcard(std::vector<double> data);
  void sdcard_close();

  private:
  SdFs sd;
  FsFile file;
  // RingBuf for File type FsFile.
  RingBuf<FsFile, RING_BUF_CAPACITY> rb;
  const char* filename_;
};

#endif
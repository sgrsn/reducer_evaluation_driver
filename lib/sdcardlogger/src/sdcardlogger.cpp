#include "sdcardlogger/sdcardlogger.hpp"

SDcardLogger::SDcardLogger(const char* filename) : filename_(filename)
{
}

void SDcardLogger::sdcard_setup()
{
  // Initialize the SD.
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
  }
  // Open or create file - truncate existing file.
  if (!file.open(filename_, O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println("open failed\n");
    return;
  }
  // File must be pre-allocated to avoid huge
  // delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE)) {
     Serial.println("preAllocate failed\n");
     file.close();
     return;
  }
  // initialize the RingBuf.
  rb.begin(&file);
  Serial.println("Setup complete\n");
}

bool SDcardLogger::checkMaxSize()
{
  // Max RingBuf used bytes. Useful to understand RingBuf overrun.
  size_t maxUsed = 0;
  size_t n = rb.bytesUsed();
  if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20)) {
    Serial.println("File full - quitting.");
    return false;
  }
  if (n > maxUsed) {
    maxUsed = n;
  }
  if (n >= 512 && !file.isBusy()) {
    // Not busy only allows one sector before possible busy wait.
    // Write one sector from RingBuf to file.
    if (512 != rb.writeOut(512)) {
      Serial.println("writeOut failed");
      return false;
    }
  }
  return true;
}

void SDcardLogger::dump2sdcard(std::vector<std::string> data)
{
  if(!checkMaxSize()) return;

  for(auto d : data)
  {
    rb.print(d.c_str());
    rb.write(',');
  }
  rb.println("");
  if (rb.getWriteError()) {
    // Error caused by too few free bytes in RingBuf.
    Serial.println("WriteError");
    return;
  }
}

void SDcardLogger::dump2sdcard(std::vector<double> data)
{
  if(!checkMaxSize()) return;

  for(auto d : data)
  {
    char buf[] = {};
    dtostrf(d, 10, 5, buf);
    rb.write(buf);
    rb.write(',');
  }
  rb.println("");
  if (rb.getWriteError()) {
    // Error caused by too few free bytes in RingBuf.
    Serial.println("WriteError");
    return;
  }
  //Serial.println("Dump complete\n");
}


void SDcardLogger::sdcard_close()
{
  // Write any RingBuf data to file.
  rb.sync();
  file.truncate();
  file.rewind();
  file.close();
  Serial.println("Close complete\n");
}
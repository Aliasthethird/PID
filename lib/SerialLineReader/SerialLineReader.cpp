#include <Arduino.h>
#include "SerialLineReader.h"



  /**
 * @brief Reads characters from the serial buffer until a newline character is received.
 *
 * This method should be called repeatedly (e.g., inside the loop function).
 * It accumulates incoming serial characters into an internal buffer.
 * When a newline character ('\\n') is received, it null-terminates the string
 * and returns `true` to indicate a complete line has been read.
 *
 * @return true if a full line ending with '\n' was received, false otherwise.
 */
  [[nodiscard]] bool SerialLineReader::read()
  {
  static uint8_t idx = 0;

  while (Serial.available() > 0)
  {
    // // read the incoming byte:
    input_[idx] = Serial.read();
    if (input_[idx] == '\n')
    {
      input_[idx + 1] = '\0';
      idx = 0;
      return true;
    }
    else
      idx++;
  }
  return false;
}

 /**
   * @brief Returns a pointer to the most recently read line.
   */
const char* SerialLineReader::getLine() const
  {
    return input_;
  }


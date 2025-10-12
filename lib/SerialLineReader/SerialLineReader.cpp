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
  [[nodiscard]] bool SerialLineReader::newLineAvailable()
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


KeyVal parseSerialData(const char *input)
{
  static char key[8]; // persistent buffer (so we can return pointer)
  const char *sep = strchr(input, ':');
  if (sep == nullptr)
  {
    Serial.println("Unable to parse string, no \":\" in string");
    key[0] = '\0';
    return {"", NAN};
  }

  size_t keyLen = sep - input;

  if (keyLen >= sizeof(key))
    keyLen = sizeof(key) - 1;

  strncpy(key, input, keyLen);
  key[keyLen] = '\0';

  const char *valueStart = sep + 1; // jump to number
  float value = atof(valueStart);

  return {key, value};
}

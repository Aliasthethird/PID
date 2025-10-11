#include <Arduino.h>

/**
 * @brief Reads characters from the serial port until a newline ('\\n') is received.
 *
 * Call `read()` repeatedly. Once a newline is encountered, the full line
 * (including the newline) is placed in `serialData`, null-terminated.
 */
class SerialLineReader
{
public:
  SerialLineReader() {}

  /**
   * @brief Reads characters from the serial buffer until a newline character is received.
   *
   * This method should be called repeatedly (e.g., inside the loop function).
   * It accumulates incoming serial characters into an internal buffer.
   * When a newline character ('\\n') is received, it null-terminates the string
   * and returns `true` to indicate a complete line has been read.
   * 
   * Once `true` is returned, call `getLine()` to access the received line.
   *
   * @return true if a full line ending with '\n' was received, false otherwise.
   */
  [[nodiscard]] bool newLineAvailable();

  /**
   * @brief Returns a pointer to the most recently read line.
   */
  const char *getLine() const;

private:
  static constexpr size_t kBufferSize_ = 128;
  char input_[kBufferSize_] = {'\0'};
};
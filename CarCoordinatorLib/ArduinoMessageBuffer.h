#ifndef ARDUINO_MESSAGE_BUFFER_H_
#define ARDUINO_MESSAGE_BUFFER_H_

#include <RingBuf.h>
#include "standardTypes.h"

/**
 * @brief A MessageBuffer implementation following the MUML PIM message buffer specification.
 * 
 * @details The MessageBuffer is implemented for Arduino specifically, as the old message buffer couldn't get to work.
 * Thus, it uses the RingBuf library for Arduino: https://www.arduino.cc/reference/en/libraries/ringbuf/
 * 
 */
typedef struct ArduinoMessageBuffer{
    RingBuf* ringBuffer; // A ring buffer using the RingBuf library for Arduino
    size_t messageSize; // The size of the messages stored in the buffer
	bool_t replaceOldMessages;  // The mode of a MessageBuffer - false: discard new incoming message; true: replace oldest message
}ArduinoMessageBuffer;

/**
  * @brief Creates a new ArduinoMessageBuffer
  * @details Memory for a ArduninoMessageBuffer and its buffer are allocated and the ArduinoMessageBuffer is initialized
  * 
  * @param size the size of the buffer for this ArduinoMessageBuffer
  * @param mode description
  * @return the pointer to the allocated ArduinoMessageBuffer
  */
ArduinoMessageBuffer* ArduinoMessageBuffer_create(size_t capacity, size_t elementSize, bool_t mode);


size_t ArduinoMessageBuffer_getSize(ArduinoMessageBuffer* buffer);

bool_t ArduinoMessageBuffer_push(ArduinoMessageBuffer* buffer, const void* message);

bool_t ArduinoMessageBuffer_pop(ArduinoMessageBuffer* buffer, void* returnedMessage);

bool_t ArduinoMessageBuffer_doesMessageExist(ArduinoMessageBuffer* buffer);

bool_t ArduinoMessageBuffer_destroy(ArduinoMessageBuffer* buffer);

#endif /* ARDUINO_MESSAGE_BUFFER_H_ */
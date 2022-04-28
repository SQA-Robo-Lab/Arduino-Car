#include "ArduinoMessageBuffer.h"

ArduinoMessageBuffer* ArduinoMessageBuffer_create(size_t capacity, size_t messageSize, bool_t mode){
    ArduinoMessageBuffer* newAMB = (ArduinoMessageBuffer*) malloc(sizeof(ArduinoMessageBuffer));
    if (newAMB != NULL){
        newAMB->replaceOldMessages = mode;
        newAMB->ringBuffer = RingBuf_new(messageSize, capacity);
        newAMB->messageSize = messageSize;
    }
    return newAMB;
}

size_t ArduinoMessageBuffer_getSize(ArduinoMessageBuffer* buffer){
    return buffer->ringBuffer->numElements(buffer->ringBuffer);
}

bool_t ArduinoMessageBuffer_push(ArduinoMessageBuffer* buffer, const void* message){
    if (!(buffer->ringBuffer->isFull(buffer->ringBuffer))){ // if the buffer is not full:
        buffer->ringBuffer->add(buffer->ringBuffer, message); // add the message
        return true;
    } else if (buffer->replaceOldMessages) { // if the buffer is full, but old messages will be replaced:
        void* messageToReplace = malloc(sizeof(buffer->messageSize));
        buffer->ringBuffer->pull(buffer->ringBuffer, messageToReplace);
        free(messageToReplace); // remove the oldest message from the buffer
        buffer->ringBuffer->add(buffer->ringBuffer, message); // only then, add the message
        return true;
    } else { //if the buffer is full and the strategy is discard new incoming messages:
        return false; // and do nothing
    }
}

bool_t ArduinoMessageBuffer_pop(ArduinoMessageBuffer* buffer, void* returnedMessage){
    if (buffer->ringBuffer->isEmpty(buffer->ringBuffer)){
        return false;
    } else {
        buffer->ringBuffer->pull(buffer->ringBuffer, returnedMessage);
        return true;
    }
}

bool_t ArduinoMessageBuffer_doesMessageExist(ArduinoMessageBuffer* buffer){
    return !(buffer->ringBuffer->isEmpty(buffer->ringBuffer));
}
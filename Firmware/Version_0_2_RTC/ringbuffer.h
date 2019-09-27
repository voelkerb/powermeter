/***************************************************
 Library implementing a simple ringbuffer.
 Functions like available, write and read are implemented

 Feel free to use the code as it is.

 Benjamin Völker, voelkerb@me.com
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#ifndef RING_BUFFER_h
#define RING_BUFFER_h

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class RingBuffer {
  public:
    RingBuffer(uint32_t size);
    RingBuffer(uint32_t size, bool usePSRAM);
    bool init();
    bool write(uint8_t * data, uint32_t size);
    bool read(uint8_t * data, uint32_t size);
    void reset();
    bool inPSRAM();
    size_t getSize();
    uint32_t available();
    uint32_t availableForWrite();
  private:

    uint32_t _rwDistance();
    uint32_t _wrDistance();
    // Size of the buffer
    uint32_t _ring_buffer_size;
    uint32_t _readPtr;
    uint32_t _writePtr;
    // Buffer pointer
    uint8_t* _buffer;
    // esp_himem_handle_t _memoryHandle;
    // esp_himem_rangehandle_t _rangeHandle;
    // If ps ram should be used for the buffer
    bool _psram;
};

#endif

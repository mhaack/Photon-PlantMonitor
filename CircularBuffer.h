#ifndef CIRCULARBUFFER_h
#define CIRCULARBUFFER_h
#include <inttypes.h>

template <typename T, uint16_t Size>
class CircularBuffer {
public:
  void push(T value) {
    cBufferLast_++;
    if(cBufferLast_ >= Size) {
        cBufferLast_ = 0;
    }

    cBufferData[cBufferLast_] = value;

    if(cBufferSize_ < Size) {
        cBufferSize_++;
    }
  }

  T get(uint16_t index) {
    int16_t virtualIndex;
    virtualIndex = cBufferLast_ - index;
    if( virtualIndex < 0 ) {
        virtualIndex += Size;
    }
    return cBufferData[virtualIndex];
  }

  uint16_t size() {
    return cBufferSize_;
  }

private:
  T cBufferData[Size];
  int16_t cBufferLast_;
  uint8_t cBufferSize_;
};

#endif

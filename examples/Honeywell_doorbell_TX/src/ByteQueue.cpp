#include "ByteQueue.h"
using namespace Codingfield::Communication;

ByteQueue::ByteQueue() {

}

void ByteQueue::Push(uint8_t data, uint8_t nbBits) {
  for(uint8_t i = 0; i < nbBits; i++) {
    currentByte = static_cast<uint8_t>(currentByte & 0xFE);
    currentByte = static_cast<uint8_t>(currentByte | ((data & 0x80) >> 7));
    currentNbBit ++;
    if(currentNbBit == 8) {
      nextByte = currentByte;

      nextByteReady = true;
      currentByte = 0;
      currentNbBit = 0;
    }else {
      currentByte = currentByte << 1;
    }
    data = data << 1;
  }
}

bool ByteQueue::Pop(uint8_t &data) {
  if(nextByteReady) {
    data = nextByte;
    nextByteReady = false;
    return true;
  }
  return false;
}

void ByteQueue::Flush(uint8_t &data) {
  if(currentNbBit != 0) {
    for(uint8_t i = 0; (i < 8-currentNbBit); i ++) {
      currentByte = currentByte << 1;
      currentByte = static_cast<uint8_t>(currentByte & (0xFE));
      data = currentByte;
    }
  }
}

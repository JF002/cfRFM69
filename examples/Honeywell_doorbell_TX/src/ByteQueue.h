#pragma once

#include <cstdint>

namespace Codingfield {
  namespace Communication {
    /* Use this class to push an arbitrary number of bits (max 8 at a time)
     * and pop them as whole Bytes */
    class ByteQueue {
      public:
        ByteQueue();

        // Push max 8 bits of data
        void Push(uint8_t data, uint8_t nbBits);

        // Pop a whole byte if available, else, return FALSE
        bool Pop(uint8_t& data);

      void Flush(uint8_t &data);
      private:
        uint8_t currentByte = 0;
        uint8_t currentNbBit = 0;
        uint8_t nextByte = 0;
        bool nextByteReady = false;
    };
  }
}
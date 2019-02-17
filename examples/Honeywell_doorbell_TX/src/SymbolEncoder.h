#pragma once

#include <cstdint>
#include "ActiveLinkDataFrame.h"

namespace Codingfield {
  namespace Communication {
    /* This class encodes the bytes from an ActiveLinkDataFrame into FSK Symbols.
     * Bit 0 => 3 bits 1 0 0
     * Bit 1 => 3 Bits 1 1 0
     * Call GetNextSymbol to retrieve all the symbols one by one
     * */
    class SymbolEncoder {
      public:
        explicit SymbolEncoder(ActiveLinkDataFrame& dataFrame);

        // Set the next symbol of the data frame in data if available (returns TRUE).
        // Returns FALSE if all the symbols have already been returned
        bool GetNextSymbol(uint8_t& data);

        // Reset the state of the object so that you can call GetNextByte() to start a new frame.
        void Reset();


      private:
        ActiveLinkDataFrame& dataFrame;
        uint8_t dataByte = 0;
        uint8_t dataByteIndex = 7;
    };
  }
}
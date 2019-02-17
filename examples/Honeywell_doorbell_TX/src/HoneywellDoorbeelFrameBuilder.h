#pragma once
#include <stdint.h>
#include "ActiveLinkDataFrame.h"
#include "ByteQueue.h"
#include "SymbolEncoder.h"

namespace Codingfield {
  namespace Communication {
    /* This class generates a full data frame compatible with Honeywell doorbell, ready to be sent in the air
     * (FSK encoded, preamble, postamble, interframe,..)
     * Call GetNextByte to retrieve all the bytes one by one */
    class HoneywellDoorbeelFrameBuilder {
      public:
        HoneywellDoorbeelFrameBuilder(uint32_t deviceId, uint8_t nbRepeat);

        // Set the next byte of the data frame if available (returns TRUE).
        // Returns FALSE if all the bytes have already been returned
        bool GetNextByte(uint8_t&);

        // Reset the state of the object so that you can call GetNextByte() to start a new frame.
        void Reset();

      private:
        enum class States {DataPreamble, Data, DataPostamble, FramePostamble1, FramePostamble2, FramePostamble3, None};
        uint32_t deviceId;
        uint8_t nbRepeat;
        ActiveLinkDataFrame activeLinkDataFrame;
        SymbolEncoder symbolEncoder;
        uint8_t nextByte = 0;
        uint8_t currentNbBit = 0;
        States state = States::DataPreamble;
        uint8_t currentNbFramePostamble = 0;
        uint8_t currentNbRepeat = 0;
        ByteQueue byteQueue;
    };
  }
}

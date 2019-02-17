#pragma once

#include <cstdint>

namespace Codingfield {
  namespace Communication {
    /* This class represent the data frame sent by an active link device (doorbell button, for example)
     * Use GetNextByte() to retrieve the bytes of the frame one by one.
     * This method does not encode the data in FSK symbols */
    class ActiveLinkDataFrame {
      public:
        enum class Alerts {Default, RightLiftLight, FullVolume};

        // Init a new instance with a specific device ID */
        explicit ActiveLinkDataFrame(uint32_t deviceId);

        // Set the next byte of the data frame if available (returns TRUE).
        // Returns FALSE if all the bytes have already been returned
        bool GetNextByte(uint8_t& buffer);

        // Reset the state of the object so that you can call GetNextByte() to start a new frame.
        void Reset();

      private:
        void ProcessParity(uint8_t buffer);
        uint32_t deviceId;
        Alerts alert = Alerts::Default;
        bool secretKnock = false;
        bool relay = false;
        bool lowBattery = false;
        uint8_t parity = 0;
        uint8_t currentByteIndex = 0;
    };
  }
}
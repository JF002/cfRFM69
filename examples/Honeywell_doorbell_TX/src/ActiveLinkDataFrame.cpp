#include "ActiveLinkDataFrame.h"

using namespace Codingfield::Communication;

ActiveLinkDataFrame::ActiveLinkDataFrame(uint32_t deviceId) : deviceId{deviceId} {

}

void ActiveLinkDataFrame::Reset() {
  parity = 0;
  currentByteIndex = 0;
}

bool ActiveLinkDataFrame::GetNextByte(uint8_t& buffer) {
  switch(currentByteIndex) {
    case 0:
      currentByteIndex++;
      buffer = static_cast<uint8_t>((deviceId & 0xFF000000) >> 24);
      ProcessParity(buffer);
      return true;
    case 1:
      currentByteIndex++;
      buffer = static_cast<uint8_t>((deviceId & 0x00FF0000) >> 16);
      ProcessParity(buffer);
      return true;
    case 2:
      currentByteIndex++;
      buffer = static_cast<uint8_t>((deviceId & 0x0000FF00) >> 8);
      ProcessParity(buffer);
      return true;
    case 3:
      currentByteIndex++;
      buffer = static_cast<uint8_t>(deviceId & 0x000000FF);
      ProcessParity(buffer);
      return true;
    case 4:
      currentByteIndex++;
      switch(alert) {
        default:
        case Alerts::Default: buffer = 0; break;
        case Alerts::RightLiftLight: buffer = 1; break;
        case Alerts::FullVolume: buffer = 3; break;
      }
      return true;
    case 5:
      currentByteIndex++;
      buffer = static_cast<uint8_t>(
              (secretKnock ? (1 << 4) : 0) |
              (relay ? (1 << 3) : 0) |
              (lowBattery ? (1 << 1) : 0)
              );
      ProcessParity(buffer);
      buffer |= (parity & 0x01);
      return true;
    default: break;
  }
  return false;
}

void ActiveLinkDataFrame::ProcessParity(uint8_t buffer) {
  for(uint8_t i = 0; i < 8; i++) {
    if((buffer & 0x01) == 0x01)
      parity++;
    buffer = buffer >> 1;
  }
}

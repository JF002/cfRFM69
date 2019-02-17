#include "HoneywellDoorbeelFrameBuilder.h"

using namespace Codingfield::Communication;

HoneywellDoorbeelFrameBuilder::HoneywellDoorbeelFrameBuilder(uint32_t deviceId, uint8_t nbRepeat) : deviceId{deviceId}, nbRepeat{nbRepeat}, activeLinkDataFrame{deviceId}, symbolEncoder{activeLinkDataFrame} {
  Reset();
}

void HoneywellDoorbeelFrameBuilder::Reset() {
  activeLinkDataFrame.Reset();
  symbolEncoder.Reset();
  state = States::DataPreamble;
  nextByte = 0;
  currentNbBit = 0;
  currentNbFramePostamble = 0;
  currentNbRepeat = 0;
}

bool HoneywellDoorbeelFrameBuilder::GetNextByte(uint8_t& data) {
  uint8_t currentDataByte = 0;

  while(byteQueue.Pop(data) == false) {
    if(state == States::None) return false;
    switch(state) {
      case States::DataPreamble:
        byteQueue.Push(0, 3);
        state = States::Data;
        break;
      case States::Data:
        if(symbolEncoder.GetNextSymbol(currentDataByte)) {
          byteQueue.Push((currentDataByte<<5), 3);
        }
        else
          state = States::DataPostamble;
        break;
      case States::DataPostamble:
        byteQueue.Push((0x07 << 5), 3);
        currentNbRepeat++;
        if(currentNbRepeat < nbRepeat) {
          activeLinkDataFrame.Reset();
          state = States::DataPreamble;
        }
        else
          state = States::FramePostamble1;
        break;
      case States::FramePostamble1:
        byteQueue.Push((0x07 << 2), 6);
        state = States::FramePostamble2;
        break;
      case States::FramePostamble2:
        byteQueue.Push(0, 8);
        currentNbFramePostamble++;
        if(currentNbFramePostamble >= 13)
          state = States::FramePostamble3;
        break;
      case States::FramePostamble3:
        byteQueue.Flush(data);
        state = States::None;
        break;
      default:
      case States::None:
        break;
    }
  }
  return state != States::None;
}



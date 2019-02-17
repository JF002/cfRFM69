#include "SymbolEncoder.h"
using namespace Codingfield::Communication;

SymbolEncoder::SymbolEncoder(ActiveLinkDataFrame &dataFrame) : dataFrame{dataFrame} {

}

void SymbolEncoder::Reset() {
  dataFrame.Reset();
  dataByteIndex = 7;
  dataByte = 0;
}

bool SymbolEncoder::GetNextSymbol(uint8_t &data) {
  if(dataByteIndex == 7) {
    if (!dataFrame.GetNextByte(dataByte)) { return false; }
    dataByteIndex = 0;
  }
  else {
    dataByteIndex = static_cast<uint8_t>(dataByteIndex + 1);
    dataByte = dataByte << 1;
  }

  if((dataByte & 0x80) == 0x80)
    data = (0x06); // 110
  else
    data = (0x04); //100

  return true;
}

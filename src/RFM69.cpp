#include "RFM69.h"
#include <Arduino.h>
#include <SPI.h>

using namespace Codingfield::Communication;

RFM69::RFM69(SPIClass *spi, uint8_t selectPin) : spi{spi}, selectPin{selectPin} {

}

void RFM69::Select() {
  digitalWrite(selectPin, LOW);
}

void RFM69::Unselect() {
  digitalWrite(selectPin, HIGH);
}

void RFM69::WriteRegister(Registers reg, uint8_t value) {
  WriteRegister(static_cast<uint8_t>(reg), value);
}

uint8_t RFM69::ReadRegister(Registers reg) {
  return ReadRegister(static_cast<uint8_t>(reg));
}

void RFM69::WriteRegister(uint8_t addr, uint8_t value) {
  Select();
  spi->transfer(0x80 | addr);
  spi->transfer(value);
  Unselect();
}

uint8_t RFM69::ReadRegister(uint8_t addr) {
  Select();
  spi->transfer(0x7f & addr);
  uint8_t value =  spi->transfer(0);
  Unselect();
  return value;
}

bool RFM69::IsRfm69() {
  return ReadRegister(Registers::Version) == 0x24;
}

uint8_t RFM69::ReadTemperature() {
  WriteRegister(Registers::Temp1, static_cast<uint8_t>(TemperatureMeasurementValues::TemperatureMeasurementStart));

  bool ready = false;
  do {
    ready = !IsBitSet(ReadRegister(Registers::Temp1), static_cast<uint8_t>(TemperatureMeasurementValues::TemperatureMeasurementRunning));
    delay(1);
  }while(!ready);

  return ReadRegister(Registers::Temp2);
}

bool RFM69::IsBitSet(uint8_t value, uint8_t bit) {
  return (value & bit) == bit;
}

void RFM69::SetOperatingMode(RFM69::Modes mode) {
  uint8_t currentRegisterValue = ReadRegister(Registers::OperatingMode);
  currentRegisterValue &= ~0x1C;
  WriteRegister(Registers::OperatingMode, currentRegisterValue | static_cast<uint8_t>( mode));

  bool ready = false;
  while(!ready) {
    ready = IsBitSet(ReadRegister(Registers::IrqFlags1), static_cast<uint8_t>(IrqFlags1Values::ModeReady));
  }
}

void RFM69::SetDataModulation(const ModulationShapings modulationShaping, const ModulationTypes modulationType, const DataModes dataMode) {
  auto value = static_cast<uint8_t>(modulationShaping) | static_cast<uint8_t>(modulationType) | static_cast<uint8_t>(dataMode);
  WriteRegister(Registers::DataModulation, value);
}

void RFM69::SetBitRate(uint16_t bitrate) {
  auto msb = static_cast<uint8_t>((bitrate >> 8) & 0x00FF);
  auto lsb = static_cast<uint8_t>(bitrate & 0x00FF);
  WriteRegister(Registers::BitRate_MSB, msb);
  WriteRegister(Registers::BitRate_LSB, lsb);
}

void RFM69::SetFrequencyDeviation(uint16_t deviation) {
  auto msb = static_cast<uint8_t>((deviation >> 8) & 0x00FF);
  auto lsb = static_cast<uint8_t>(deviation & 0x00FF);
  WriteRegister(Registers::FrequencyDeviation_MSB, msb);
  WriteRegister(Registers::FrequencyDeviation_LSB, lsb);
}

void RFM69::SetFrequency(uint32_t freq) {
  auto msb = static_cast<uint8_t>((freq >> 16) & 0x000000FF);
  auto mid = static_cast<uint8_t>((freq >> 8) & 0x000000FF);
  auto lsb = static_cast<uint8_t>(freq & 0x000000FF);
  WriteRegister(Registers::Frequency_MSB, msb);
  WriteRegister(Registers::Frequency_MID, mid);
  WriteRegister(Registers::Frequency_LSB, lsb);
}

void RFM69::ReadMaskWriteRegister(Registers reg, uint8_t mask, uint8_t value) {
  auto regValue = ReadRegister(static_cast<uint8_t>(reg));
  regValue &= ~static_cast<uint8_t>(mask);
  regValue |= static_cast<uint8_t>(value);
  WriteRegister(static_cast<uint8_t>(reg), regValue);
}

void RFM69::SetDio0Mapping(RFM69::DIO0MappingPacketRX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping1, static_cast<uint8_t>(DIO0MappingPacketRX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio0Mapping(RFM69::DIO0MappingPacketTX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping1, static_cast<uint8_t>(DIO0MappingPacketTX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio0Mapping(RFM69::DIO0MappingContinuousRX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping1, static_cast<uint8_t>(DIO0MappingContinuousRX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio0Mapping(RFM69::DIO0MappingContinuousTX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping1, static_cast<uint8_t>(DIO0MappingContinuousTX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio1Mapping(RFM69::DIO1MappingPacketRX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping1, static_cast<uint8_t>(DIO1MappingPacketRX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio1Mapping(RFM69::DIO1MappingPacketTX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping1, static_cast<uint8_t>(DIO1MappingPacketTX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio1Mapping(RFM69::DIO1MappingContinuousRX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping1, static_cast<uint8_t>(DIO1MappingContinuousRX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio1Mapping(RFM69::DIO1MappingContinuousTX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping1, static_cast<uint8_t>(DIO1MappingContinuousTX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio2Mapping(RFM69::DIO2MappingPacketRX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping1, static_cast<uint8_t>(DIO2MappingPacketRX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio2Mapping(RFM69::DIO2MappingPacketTX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping1, static_cast<uint8_t>(DIO2MappingPacketTX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio2Mapping(RFM69::DIO2MappingContinuousRX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping1, static_cast<uint8_t>(DIO2MappingContinuousRX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio2Mapping(RFM69::DIO2MappingContinuousTX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping1, static_cast<uint8_t>(DIO2MappingContinuousTX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio3Mapping(RFM69::DIO3MappingPacketRX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping1, static_cast<uint8_t>(DIO3MappingPacketRX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio3Mapping(RFM69::DIO3MappingPacketTX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping1, static_cast<uint8_t>(DIO3MappingPacketTX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio3Mapping(RFM69::DIO3MappingContinuousRX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping1, static_cast<uint8_t>(DIO3MappingContinuousRX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio3Mapping(RFM69::DIO3MappingContinuousTX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping1, static_cast<uint8_t>(DIO3MappingContinuousTX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio4Mapping(RFM69::DIO4MappingPacketRX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping2, static_cast<uint8_t>(DIO4MappingPacketRX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio4Mapping(RFM69::DIO4MappingPacketTX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping2, static_cast<uint8_t>(DIO4MappingPacketTX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio4Mapping(RFM69::DIO4MappingContinuousRX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping2, static_cast<uint8_t>(DIO4MappingContinuousRX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio4Mapping(RFM69::DIO4MappingContinuousTX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping2, static_cast<uint8_t>(DIO4MappingContinuousTX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio5Mapping(RFM69::DIO5MappingPacketRX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping2, static_cast<uint8_t>(DIO5MappingPacketRX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio5Mapping(RFM69::DIO5MappingPacketTX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping2, static_cast<uint8_t>(DIO5MappingPacketTX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio5Mapping(RFM69::DIO5MappingContinuousRX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping2, static_cast<uint8_t>(DIO5MappingContinuousRX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::SetDio5Mapping(RFM69::DIO5MappingContinuousTX mapping) {
  ReadMaskWriteRegister(Registers::DIOMapping2, static_cast<uint8_t>(DIO5MappingContinuousTX::Mask), static_cast<uint8_t>(mapping));
}

void RFM69::ClearFifoOverrunFlag() {
  WriteRegister(Registers::IrqFlags2, ClearFifoOverrunFlagValue);
}

void RFM69::SetRssiThreshold(uint8_t threshold) {
  WriteRegister(Registers::RssiThreshold, threshold);
}

void RFM69::SetRxTimeoutStart(uint8_t value) {
  WriteRegister(Registers::RxTimeout1, value);
}

void RFM69::SetTimeoutRssiThreshold(uint8_t value) {
  WriteRegister(Registers::RxTimeout2, value);
}

void RFM69::SetSyncWordConfig(uint8_t bitTolerance, uint8_t syncWordSize, RFM69::FifoFillConditions condition, bool enableSyncWord) {
  auto tolerance = static_cast<uint8_t>(bitTolerance & 0x03);
  auto size = static_cast<uint8_t>(((syncWordSize-1) & 0x03) << 3);
  auto enable = static_cast<uint8_t>(enableSyncWord ? (1 << 7) : 0);
  auto c = static_cast<uint8_t>(condition);

  WriteRegister(Registers::SyncConfig, tolerance | size | c | enable);
}

void RFM69::SetSyncWordValue1(uint8_t value) {
  WriteRegister(Registers::SyncValue1, value);
}

void RFM69::SetSyncWordValue2(uint8_t value) {
  WriteRegister(Registers::SyncValue2, value);
}

void RFM69::SetSyncWordValue3(uint8_t value) {
  WriteRegister(Registers::SyncValue3, value);
}

void RFM69::SetSyncWordValue4(uint8_t value) {
  WriteRegister(Registers::SyncValue4, value);
}

void RFM69::SetSyncWordValue5(uint8_t value) {
  WriteRegister(Registers::SyncValue5, value);
}

void RFM69::SetSyncWordValue6(uint8_t value) {
  WriteRegister(Registers::SyncValue6, value);
}

void RFM69::SetSyncWordValue7(uint8_t value) {
  WriteRegister(Registers::SyncValue7, value);
}

void RFM69::SetSyncWordValue8(uint8_t value) {
  WriteRegister(Registers::SyncValue8, value);
}

void RFM69::SetPacketConfig(RFM69::AddressFilterings addressFiltering, bool crcAutoClearOff, bool enableCrc,
                            RFM69::DcFreeTypes dcFreeType, RFM69::PacketFormats format) {
  auto filtering = static_cast<uint8_t>(addressFiltering);
  auto crc = static_cast<uint8_t>(crcAutoClearOff ? (1 << 3) : 0);
  auto enable = static_cast<uint8_t>(enableCrc ? (1 << 4) : 0);
  auto dcFree = static_cast<uint8_t>(dcFreeType);
  auto f = static_cast<uint8_t>(format);
  WriteRegister(Registers::PacketConfig, filtering | crc | enable | dcFree | f);
}

void RFM69::SetPayloadLength(uint8_t length) {
  WriteRegister(Registers::PayloadLength, length);
}

bool RFM69::IsIrqFlagSet(RFM69::IrqFlags1 flag) {
  return IsBitSet(ReadRegister(Registers::IrqFlags1), static_cast<uint8_t>(flag));
}

bool RFM69::IsIrqFlagSet(RFM69::IrqFlags2 flag) {
  return IsBitSet(ReadRegister(Registers::IrqFlags2), static_cast<uint8_t>(flag));
}


void RFM69::RestartRx() {
  auto packetconfig2 = ReadRegister(Registers::PacketConfig2);
  WriteRegister(Registers::PacketConfig2, packetconfig2 | ForceRestartRx);
}

void RFM69::TransmitPacket(std::vector<uint8_t> message) {
  Select();
  spi->transfer(0x00 | 0x80);
  for(auto b : message) {
    spi->transfer(b);
  }
  Unselect();
}

void RFM69::SetFifoThreshold(uint8_t threshold, RFM69::TxStartCondition startCondition) {
  auto t = static_cast<uint8_t>(threshold & 0x3F);
  auto condition = static_cast<uint8_t>(startCondition);
  WriteRegister(Registers::FifoThreshold, t | condition);
}

void RFM69::SetPreambleSize(uint16_t size) {
  WriteRegister(Registers::Preamble_MSB, static_cast<uint8_t>((size >> 8) & 0x00FF)),
  WriteRegister(Registers::Preamble_LSB, static_cast<uint8_t>(size& 0x00FF));
}

void RFM69::SetOcp(bool enabled) {
  WriteRegister(Registers::Ocp, static_cast<uint8_t>(enabled ? 0x1A : 0x0F));
  WriteRegister(static_cast<uint8_t>(0x11), static_cast<uint8_t>(0xff));
}

void RFM69::AbortListen() {
  uint8_t currentRegisterValue = ReadRegister(Registers::OperatingMode);
  currentRegisterValue &= ~0x60;
  WriteRegister(Registers::OperatingMode, static_cast<uint8_t>(currentRegisterValue | 0x20));
}

void RFM69::EnableListen() {
  uint8_t currentRegisterValue = ReadRegister(Registers::OperatingMode);
  currentRegisterValue &= ~0x60;
  WriteRegister(Registers::OperatingMode, static_cast<uint8_t>(currentRegisterValue | 0x40));
}

void RFM69::EnableSequencer(bool enable) {
  uint8_t registerValue = ReadRegister(Registers::OperatingMode);
  registerValue &= ~0x80;
  if(!enable)
    registerValue |= 0x80;
  WriteRegister(Registers::OperatingMode, static_cast<uint8_t>(registerValue));
}

void RFM69::StartRcCalibration() {
  WriteRegister(Registers::OscillatorsSettings, static_cast<uint8_t>(0x80));
}

bool RFM69::IsRcCalibrationDone() {
  uint8_t registerValue = ReadRegister(Registers::OscillatorsSettings);
  return ((registerValue & 0x40) == 0x40);
}

void RFM69::SetAfcRoutine(RFM69::AfcRoutines routine) {
  WriteRegister(Registers::AfcControl, static_cast<uint8_t>(routine));
}

void RFM69::SetListenMode(RFM69::ListenEnds end, RFM69::ListenCriterias criteria, RFM69::ListenResolutionsRx resolutionRx, RFM69::ListenResolutionsIdle resolutionIdle) {
  auto e = static_cast<uint8_t>(end);
  auto c = static_cast<uint8_t>(criteria);
  auto resRx = static_cast<uint8_t>(resolutionRx);
  auto resIdle = static_cast<uint8_t>(resolutionIdle);
  WriteRegister(Registers::PacketConfig, e | c | resRx | resIdle);
}

void RFM69::SetListenCoefIdle(uint8_t coef) {
  WriteRegister(Registers::ListenIdleDuration, coef);
}

void RFM69::SetListenCoefRx(uint8_t coef) {
  WriteRegister(Registers::ListenRxDuration, coef);
}

void RFM69::SetPowerAmplifierLevel(uint8_t outputPower, bool pa0, bool pa1, bool pa2) {
  WriteRegister(Registers::ListenRxDuration,
                static_cast<uint8_t>((outputPower & 0x1F) | (pa2 ? (1 << 5) : 0) | (pa1 ? (1 << 6) : 0) || (pa0 ? (1 << 7) : 0)));
}

void RFM69::SetPowerAmplifierRamp(RFM69::PowerAmplifierRamps ramp) {
  WriteRegister(Registers::PowerAmplifierRamp, static_cast<uint8_t>(ramp));
}

void RFM69::SetLna(RFM69::LnaGains gain, RFM69::LnaZValues z) {
  WriteRegister(Registers::Lna, static_cast<uint8_t>(gain) | static_cast<uint8_t>(z));
}

RFM69::LnaZValues RFM69::GetCurrentLnaGain() {
  auto registerValue = ReadRegister(Registers::Lna);
  registerValue = registerValue >> 3;
  registerValue &= 0x07;
  return static_cast<LnaZValues>(registerValue);
}

void RFM69::SetNodeAddress(uint8_t addr) {
  WriteRegister(Registers::NodeAddress, addr);
}

void RFM69::SetBroadcastAddress(uint8_t addr) {
  WriteRegister(Registers::BroadcastAddress, addr);
}




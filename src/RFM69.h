#pragma once


#include <stdint.h>
#include <vector>
#include <Arduino.h>

// TODO : use std::bitfield instead of manual bit manipulations ?
// TODO configure the SELECT Pin via constructor

class SPIClass;

namespace Codingfield {
  namespace Communication {

    class RFM69 {
      public:

      enum class Registers : uint8_t {
        OperatingMode = 0x01,
        DataModulation = 0x02,
        BitRate_MSB = 0x03,
        BitRate_LSB = 0x04,
        FrequencyDeviation_MSB = 0x05,
        FrequencyDeviation_LSB = 0x06,
        Frequency_MSB = 0x07,
        Frequency_MID = 0x08,
        Frequency_LSB = 0x09,
        OscillatorsSettings = 0x0A,
        AfcControl = 0x0B,
        ListenSettings = 0x0D,
        ListenIdleDuration = 0x0E,
        ListenRxDuration = 0x0F,
        Version = 0x10,
        PowerAmplifierLevel = 0x11,
        PowerAmplifierRamp = 0x12,
        Ocp = 0x13,
        Lna = 0x18,
        RxBandwidth = 0x19,
        AfcBandwidth = 0x1A,
        OokPeak = 0x1B,
        OokAvg = 0x1C,
        OokFix = 0x1D,
        AfcFei = 0x1E,
        AfcFrequencyCorrection_MSB = 0x1F,
        AfcFrequencyCorrection_LSB = 0x20,
        FeiFrequencyError_MSB = 0x21,
        FeiFrequencyError_LSB = 0x22,
        RssiConfiguration = 0x23,
        RssiValue = 0x24,
        DIOMapping1 = 0x25,
        DIOMapping2 = 0x26,
        IrqFlags1 = 0x27,
        IrqFlags2 = 0x28,
        RssiThreshold = 0x29,
        RxTimeout1 = 0x2A,
        RxTimeout2 = 0x2B,
        Preamble_MSB = 0x2C,
        Preamble_LSB = 0x2D,
        SyncConfig = 0x2E,
        SyncValue1 = 0x2F,
        SyncValue2 = 0x30,
        SyncValue3 = 0x31,
        SyncValue4 = 0x32,
        SyncValue5 = 0x33,
        SyncValue6 = 0x34,
        SyncValue7 = 0x35,
        SyncValue8 = 0x36,
        PacketConfig = 0x37,
        PayloadLength = 0x38,
        NodeAddress = 0x39,
        BroadcastAddress = 0x3A,
        AutoModes = 0x3B,
        FifoThreshold = 0x3C,
        PacketConfig2 = 0x3D,
        AesKey1 = 0x3E,
        AesKey2 = 0x3F,
        AesKey3 = 0x40,
        AesKey4 = 0x41,
        AesKey5 = 0x42,
        AesKey6 = 0x43,
        AesKey7 = 0x44,
        AesKey8 = 0x45,
        AesKey9 = 0x46,
        AesKey10 = 0x47,
        AesKey11 = 0x48,
        AesKey12 = 0x49,
        AesKey13 = 0x4A,
        AesKey14 = 0x4B,
        AesKey15 = 0x4C,
        AesKey16 = 0x4D,
        Temp1 = 0x4E,
        Temp2 = 0x4F,
        TestLna = 0x58,
        TestPa1 = 0x5A,
        TestPa2 = 0x5C,
        TestDagc = 0x6F,
        testAfc = 0x71
      };

      enum class Modes : uint8_t {
        Sleep = 0,
        Standby = 1 << 2,
        FrequencySynth = 2 << 2,
        Tx = 3 << 2,
        Rx = 4 << 2
      };

      enum class ModulationShapings {
        NoShaping = 0,
        FSK_GaussianFilter_1_0 = 0x01,
        FSK_GaussianFilter_0_5 = 0x02,
        FSK_GaussianFilter_0_3 = 0x03,

        OOK_FilteringCutOff_BR = 0x01,
        OOK_FilteringCutOff_2BR = 0x02,
      };

      enum class ModulationTypes {
        FSK = 0,
        OOK = 1 << 3
      };

      enum class DataModes {
        Packet = 0,
        ContinuousBitSync = 2 << 5,
        Continuous = 3 << 5
      };

      enum class DIO0MappingPacketRX {
        CrcOk = 0,
        PayloadReady = 1 << 6,
        SyncAddress = 2 << 6,
        Rssi = 3 << 6,
        Mask = 0xC0
      };

      enum class DIO1MappingPacketRX {
        FifoLevel = 0,
        FifoFull = 1 << 4,
        FifoNotEmpty = 2 << 4,
        Timeout = 3 << 4,
        Mask = 0x30
      };

      enum class DIO2MappingPacketRX {
        FifoNotEmpty = 0,
        Data = 1 << 2,
        AutoMode = 3 << 2,
        Mask = 0x0C
      };

      enum class DIO3MappingPacketRX {
        FifoFull = 0,
        Rssi = 1,
        SyncAddr = 2,
        PllLock = 3,
        Mask = 0x03
      };

      enum class DIO4MappingPacketRX {
        Timeout = 0,
        Rssi = 1 << 6,
        RxReady = 2 << 6,
        PllLock = 3 << 6,
        Mask = 0xC0
      };

      enum class DIO5MappingPacketRX {
        ClkOut = 0,
        Data = 1 << 4,
        ModeReady = 3 << 4,
        Mask = 0x30
      };

      enum class DIO0MappingContinuousRX {
        SyncAddress = 0,
        Timeout = 1 << 6,
        Rssi = 2 << 6,
        ModeReady = 3 << 6,
        Mask = 0xC0
      };

      enum class DIO1MappingContinuousRX {
        Dclk = 0,
        RxReady = 0x01 << 4,
        SyncAddress = 0x03 << 4,
        Mask = 0x30
      };

      enum class DIO2MappingContinuousRX {
        Data = 0,
        Mask = 0x0C
      };

      enum class DIO3MappingContinuousRX {
        Rssi = 0,
        RxReady = 1,
        AutoMode = 2,
        Timeout = 3,
        Mask = 0x03
      };

      enum class DIO4MappingContinuousRX {
        Timeout = 0,
        RxReady = 1 << 6,
        SyncAddress = 2 << 6,
        PllLock = 3 << 6,
        Mask = 0xC0
      };

      enum class DIO5MappingContinuousRX {
        ClkOut = 0,
        Rssi = 1 << 4,
        ModeReady = 3 << 4,
        Mask = 0x30
      };

      enum class DIO0MappingPacketTX {
        PacketSent = 0,
        TxReady = 1 << 6,
        PllLock = 3 << 6,
        Mask = 0xC0
      };

      enum class DIO1MappingPacketTX {
        FifoLevel = 0,
        FifoFull = 1 << 4,
        FifoNotEmpty = 2 << 4,
        PllLock = 3 << 4,
        Mask = 0x30
      };

      enum class DIO2MappingPacketTX {
        FifoNotEmpty = 0,
        Data = 1 << 2,
        AutoMode = 3 << 2,
        Mask = 0x0C
      };

      enum class DIO3MappingPacketTX {
        FifoFull = 0,
        TxReady = 1,
        PllLock = 3,
        Mask = 0x03
      };

      enum class DIO4MappingPacketTX {
        ModeReady = 0,
        TxReady = 1 << 6,
        PllLock = 3 << 6,
        Mask = 0xC0
      };

      enum class DIO5MappingPacketTX {
        ClkOut = 0,
        Data = 1 << 4,
        ModeReady = 3 << 4,
        Mask = 0x30
      };

      enum class DIO0MappingContinuousTX {
        PllLock = 0,
        TxReady = 1 << 6,
        ModeReady = 3 << 6,
        Mask = 0xC0,
      };

      enum class DIO1MappingContinuousTX {
        Dclk = 0,
        TxReady = 1 << 4,
        PllLock = 3 << 4,
        Mask = 0x30
      };

      enum class DIO2MappingContinuousTX {
        Data = 0,
        Mask = 0x0C,
      };

      enum class DIO3MappingContinuousTX {
        TxReady = 0,
        AutoMode = 2,
        Mode = 0x03,
        Mask = 0x03
      };

      enum class DIO4MappingContinuousTX {
        TxReady = 0,
        PllLock = 3 << 6,
        Mask = 0xC0
      };

      enum class DIO5MappingContinuousTX {
        ClkOut = 0,
        ModeReady = 3 << 4,
        Mask = 0x30
      };

      enum class IrqFlags1Values {
        ModeReady = 1<<7
      };

      enum class TemperatureMeasurementValues {
        TemperatureMeasurementStart = 1 << 3,
        TemperatureMeasurementRunning = 1 << 2
      };

      enum class FifoFillConditions {
        IfSyncAddressInterruptOccurs = 0,
        AsLongAsFifoFillConditionIsSet = 1 << 6
      };

      enum class AddressFilterings {
        None = 0,
        AddressMustMatchNodeAddress = 1 << 1,
        AddressMustMatchNodeAddressAndBroadcast = 2 << 1
      };

      enum class DcFreeTypes {
        None = 0,
        Manchester = 1 << 5,
        Whitening = 2 << 5
      };

      enum class PacketFormats {
        FixedLength = 0,
        VariableLength = 1 << 7
      };

      enum class IrqFlags1 {
        SyncAddressMatch = 1,
        AutoMode = 1 << 1,
        Timeout = 1 << 2,
        Rssi = 1 << 3,
        PllLock = 1 << 4,
        TxReady = 1 << 5,
        RxReady = 1 << 6,
        ModeReady = 1 << 7
      };

      enum class IrqFlags2 {
        CrcOk = 1 << 1,
        PayloadReady = 1 << 2,
        PacketSent = 1 << 3,
        FifoOverrun = 1 << 4,
        FifoLevel = 1 << 5,
        FifoNotEmpty = 1 << 6,
        FifoFull = 1 << 7
      };


      enum class TxStartCondition {
        FifoLevel = 0x00,
        FifoNotEmpty = 1<<7
      };

      enum class AfcRoutines {
        Standard = 0,
        Improved = 1 << 5
      };

      enum class ListenEnds {
        StayInRxMode = 0, // Chip stays in Rx mode. Listen mode stops and must be disabled
        StayInRxModeThenMode = 1 << 1, // Chip stays in Rx mode until PayloadReady or Timeout interrupt occurs. It then goes to the mode defined by Mode Listen mode stops and must be disabled
        stayInRcModeThenIdle = 2 << 1  // Chip stays in Rx mode until PayloadReady or Timeout interrupt occurs. Listen mode then resumes in Idle state. FIFO content is lost at next Rx wakeup.
      };

      enum class ListenCriterias {
        RssiThreshold = 0,
        RssiThresholdAndSyncAddress = 1 << 3
      };

      enum class ListenResolutionsRx {
        us64 = 1 << 4, // 64µs
        ms4_1 = 2 << 4, // 4.1ms
        ms262 = 3 << 4, // 262ms
      };

      enum class ListenResolutionsIdle {
        us64 = 1 << 4, // 64µs
        ms4_1 = 2 << 4, // 4.1ms
        ms262 = 3 << 4, // 262ms
      };

      enum class PowerAmplifierRamps {
        ms3_4 = 0,      // 3.4 ms
        ms2 = 1,        // 2 ms
        ms1 = 2,        // 1 ms
        us500 = 3,      // 500µs
        us250 = 4,      // 250µs
        us125 = 5,      // 125µs
        us100 = 6,      // 100µs
        us62 = 7,       // 62µs
        us50 = 8,       // 50µs
        us40 = 9,       // 40µs
        us31 = 10,      // 31µs
        us25 = 11,      // 25µs
        us20 = 12,      // 20µs
        us15 = 13,      // 15µs
        us12 = 14,      // 12µs
        us10 = 15       // 10µs
      };

      enum class LnaZValues {
        Ohms50 = 0, // 50 Ohms
        Ohms200 = 1 << 7 // 200 Ohms
      };

      enum class LnaGains {
        autoGain = 0,
        Highest = 1,
        HigestMinus6db = 2, // Highest gain - 6Db
        HigestMinus12db = 3, // Highest gain - 12Db
        HigestMinus24db = 4, // Highest gain - 24Db
        HigestMinus36db = 5, // Highest gain - 36Db
        HigestMinus48db = 6, // Highest gain - 48Db
      };

      static constexpr uint8_t ClearFifoOverrunFlagValue = 1<<4;
      static constexpr uint8_t ForceRestartRx = 1<<2;

      RFM69(SPIClass *spi, uint8_t selectPin);

      void WriteRegister(Registers reg, uint8_t value);
      uint8_t ReadRegister(Registers reg);

      void WriteRegister(uint8_t addr, uint8_t value);
      uint8_t ReadRegister(uint8_t addr);

      bool IsRfm69();
      uint8_t ReadTemperature();

      /* Transceiver’s operating modes */
      void SetOperatingMode(Modes mode);

      /* Configure data modulation */
      void SetDataModulation(ModulationShapings modulationShaping, ModulationTypes modulationType, DataModes dataMode);

      /* Bitrate in RAW value : actual bitrate = (FXO SC / bitrate raw value). FXO SX = 32Mhz by default */
      void SetBitRate(uint16_t bitrate);

      /* Frequency deviation in RAW value : actual frequency deviation = Fstep * freqdev RAW value. Fstep = 61Khz by default */
      void SetFrequencyDeviation(uint16_t deviation);

      /* RF carrier frequency in RAW value : actual freq = Fstep * raw freq. Fstep = 61Khz by default */
      void SetFrequency(uint32_t freq);

      /* Mapping of DIO0 in Packet RX mode */
      void SetDio0Mapping(DIO0MappingPacketRX mapping);

      /* Mapping of DIO0 in Packet TX mode */
      void SetDio0Mapping(DIO0MappingPacketTX mapping);

      /* Mapping of DIO0 in Continuous RX mode */
      void SetDio0Mapping(DIO0MappingContinuousRX mapping);

      /* Mapping of DIO0 in Continuous TX mode */
      void SetDio0Mapping(DIO0MappingContinuousTX mapping);

      /* Mapping of DIO1 in Packet RX mode */
      void SetDio1Mapping(DIO1MappingPacketRX mapping);

      /* Mapping of DIO1 in Packet TX mode */
      void SetDio1Mapping(DIO1MappingPacketTX mapping);

      /* Mapping of DIO1 in Continuous RX mode */
      void SetDio1Mapping(DIO1MappingContinuousRX mapping);

      /* Mapping of DIO1 in Continuous TX mode */
      void SetDio1Mapping(DIO1MappingContinuousTX mapping);

      /* Mapping of DIO2 in Packet RX mode */
      void SetDio2Mapping(DIO2MappingPacketRX mapping);

      /* Mapping of DIO2 in Packet TX mode */
      void SetDio2Mapping(DIO2MappingPacketTX mapping);

      /* Mapping of DIO2 in Continuous RX mode */
      void SetDio2Mapping(DIO2MappingContinuousRX mapping);

      /* Mapping of DIO2 in Continuous TX mode */
      void SetDio2Mapping(DIO2MappingContinuousTX mapping);

      /* Mapping of DIO3 in Packet RX mode */
      void SetDio3Mapping(DIO3MappingPacketRX mapping);

      /* Mapping of DIO3 in Packet TX mode */
      void SetDio3Mapping(DIO3MappingPacketTX mapping);

      /* Mapping of DIO3 in Continuous RX mode */
      void SetDio3Mapping(DIO3MappingContinuousRX mapping);

      /* Mapping of DIO3 in Continuous TX mode */
      void SetDio3Mapping(DIO3MappingContinuousTX mapping);

      /* Mapping of DIO4 in Packet RX mode */
      void SetDio4Mapping(DIO4MappingPacketRX mapping);

      /* Mapping of DIO4 in Packet TX mode */
      void SetDio4Mapping(DIO4MappingPacketTX mapping);

      /* Mapping of DIO4 in Continuous RX mode */
      void SetDio4Mapping(DIO4MappingContinuousRX mapping);

      /* Mapping of DIO4 in Continuous TX mode */
      void SetDio4Mapping(DIO4MappingContinuousTX mapping);

      /* Mapping of DIO5 in Packet RX mode */
      void SetDio5Mapping(DIO5MappingPacketRX mapping);

      /* Mapping of DIO5 in Packet TX mode */
      void SetDio5Mapping(DIO5MappingPacketTX mapping);

      /* Mapping of DIO5 in Continuous RX mode */
      void SetDio5Mapping(DIO5MappingContinuousRX mapping);

      /* Mapping of DIO5 in Continuous TX mode */
      void SetDio5Mapping(DIO5MappingContinuousTX mapping);

      /* Clear FIFO overrung flag */
      void ClearFifoOverrunFlag();

      /* Set RSSI threshold level for Rssi interrupt */
      void SetRssiThreshold(uint8_t threshold);

      /* Timeout interrupt is generated TimeoutRxStart *16*T bit after switching to Rx mode if
       * Rssi interrupt doesn’t occur (i.e. RssiValue > RssiThreshold)
       * 0x00: TimeoutRxStart is disabled */
      void SetRxTimeoutStart(uint8_t value);

      /* Timeout interrupt is generated TimeoutRssiThresh  16*T bit after Rssi interrupt if
       * PayloadReady interrupt doesn’t occur.
       * 0x00: TimeoutRssiThresh is disabled */
      void SetTimeoutRssiThreshold(uint8_t value);

      /* Synchronize Word configuration */
      void SetSyncWordConfig(uint8_t bitTolerance, uint8_t syncWordSize, FifoFillConditions condition,
                             bool enableSyncWord);

      /* Set the value of the byte 1 of the syncword */
      void SetSyncWordValue1(uint8_t value);

      /* Set the value of the byte 2 of the syncword */
      void SetSyncWordValue2(uint8_t value);

      /* Set the value of the byte 3 of the syncword */
      void SetSyncWordValue3(uint8_t value);

      /* Set the value of the byte 4 of the syncword */
      void SetSyncWordValue4(uint8_t value);

      /* Set the value of the byte 5 of the syncword */
      void SetSyncWordValue5(uint8_t value);

      /* Set the value of the byte 6 of the syncword */
      void SetSyncWordValue6(uint8_t value);

      /* Set the value of the byte 7 of the syncword */
      void SetSyncWordValue7(uint8_t value);

      /* Set the value of the byte 8 of the syncword */
      void SetSyncWordValue8(uint8_t value);

      /* Configure Packet mode */
      void SetPacketConfig(AddressFilterings addressFiltering, bool crcAutoClearOff, bool enableCrc, DcFreeTypes dcFreeType, PacketFormats format);

      /* Configure payload length (0 = fixed) */
      void SetPayloadLength(uint8_t length);

      /* Check if a flag is set in IRQ Flags 1 register */
      bool IsIrqFlagSet(IrqFlags1 flag);

      /* Check if a flag is set in IRQ Flags 2 register */
      bool IsIrqFlagSet(IrqFlags2 flag);

      /* Forces the Receiver in WAIT mode, in Continuous Rx mode. */
      void RestartRx();

      /* Configure FIFO */
      void SetFifoThreshold(uint8_t threshold, TxStartCondition startCondition);

      /* Set the size of the preamble */
      void SetPreambleSize(uint16_t size);

      /* Enable/disable Over current protection */
      void SetOcp(bool enabled);

      /* Write a packet into the FIFO */
      void TransmitPacket(std::vector<uint8_t> message);

      /***** No tested *****/
      /* Aborts Listen mode */
      void AbortListen();

      /* Enables/Disables Listen mode, should be enabled whilst in Standby mode */
      void EnableListen();

      /* Controls the automatic Sequencer */
      void EnableSequencer(bool enable);

      /* Triggers the calibration of the RC oscillator. Must be done in Standby mode */
      void StartRcCalibration();

      /* Check if RC Calibration is done */
      bool IsRcCalibrationDone();

      /* Set AFC routine. 'Improved' is an improved AFC routine for signal with modulation index < 2 */
      void SetAfcRoutine(AfcRoutines routine);

      /* Configure Listen mode */
      void SetListenMode(ListenEnds end, ListenCriterias criteria, ListenResolutionsRx resolutionRx, ListenResolutionsIdle resolutionIdle);

      /* Set the duration of the IDLE phase in Listen mode : time listen idle = ListenCoefIdle * ListenResolutionIdle */
      void SetListenCoefIdle(uint8_t coef);

      /* Set the duration of the IDLE phase in RX mode : time listen RX = ListenCoefRX * ListenResolutionRX */
      void SetListenCoefRx(uint8_t coef);

      /* Set power amplifier level */
      void SetPowerAmplifierLevel(uint8_t outputPower, bool pa0, bool pa1, bool pa2);

      /* Set Rise/Fall time of ramp up/down in FSK */
      void SetPowerAmplifierRamp(PowerAmplifierRamps ramp);

      /* Set LNA input impedance */
      void SetLna(LnaGains gain, LnaZValues z);

      /* Get Current LNA gain (either set manually of by AGC loop) */
      LnaZValues GetCurrentLnaGain();

      /* Set node address used in address filtering */
      void SetNodeAddress(uint8_t addr);

      /* Set broadcast address used in address filtering */
      void SetBroadcastAddress(uint8_t addr);

      private:
      SPIClass *spi = nullptr;
      uint8_t selectPin;

      void Select();
      void Unselect();
      bool IsBitSet(uint8_t value, uint8_t bit);
      void ReadMaskWriteRegister(Registers reg, uint8_t mask, uint8_t value);
    };
  }
}
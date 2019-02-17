#include <memory>
#include <SPI.h>
#include "RFM69.h"
#include "HoneywellDoorbeelFrameBuilder.h"
#include <Arduino.h>

using namespace Codingfield::Communication;

SPIClass spi(HSPI); // Use either HSPI or VSPI
constexpr uint32_t deviceId = 0x44BDF020; // Use any 32 bits ID you like
constexpr uint8_t nbRepeat = 50; // 50 repeats in the original frame
HoneywellDoorbeelFrameBuilder frameBuilder{deviceId, nbRepeat};

std::unique_ptr<Codingfield::Communication::RFM69> radio;
std::vector<uint8_t> currentFrame;
int state = 0;
bool packetReceived = false;
uint8_t currentByte;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
int32_t interruptCounter = 0;

void IRAM_ATTR handleInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
  interruptCounter++;
  packetReceived = true;
  portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  Serial.begin(115200);
  Serial.flush();
  Serial.println("Hello");
  currentFrame.reserve(64);

  Serial.println("Resetting the RFM69..");
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  delay(100);
  digitalWrite(4, LOW);
  delay(100);

  pinMode(15, OUTPUT); //HSPI SS
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), handleInterrupt, CHANGE);

  Serial.println("Looking for RFM69...");
  spi.begin();
  radio.reset(new RFM69(&spi, 15));

  bool isRFM69 = radio->IsRfm69();
  if(isRFM69) {
    Serial.println("RFM found!");
    auto temperature = radio->ReadTemperature();
    Serial.println("Temperature : " + String(temperature));

    radio->SetOperatingMode(RFM69::Modes::Standby);
    Serial.println("Mode -> Standby");

    Serial.println("Init RFM69...");
    radio->SetDataModulation(RFM69::ModulationShapings::NoShaping,
                             RFM69::ModulationTypes::FSK,
                             RFM69::DataModes::Packet);
    radio->SetBitRate(0x1430); // 6.250 kpbs
    radio->SetFrequencyDeviation(0x0333); // 50Khz
    radio->SetFrequency(0x00D913E8); // 868Mhz
    radio->SetDio0Mapping(RFM69::DIO0MappingPacketTX::PacketSent);
    radio->SetDio2Mapping(RFM69::DIO2MappingPacketRX::FifoNotEmpty);
    radio->ClearFifoOverrunFlag();
    radio->SetRssiThreshold(0xA0);
    radio->SetRxTimeoutStart(0);
    radio->SetTimeoutRssiThreshold(40);
    radio->SetSyncWordConfig(0, 1, RFM69::FifoFillConditions::IfSyncAddressInterruptOccurs, false);
    radio->SetSyncWordValue1(0x33);
    radio->SetSyncWordValue2(0xAC);
    radio->SetPacketConfig(RFM69::AddressFilterings::None,
                           false,
                           false,
                           RFM69::DcFreeTypes::None,
                           RFM69::PacketFormats::FixedLength);
    radio->SetPayloadLength(64);
    radio->SetFifoThreshold(32, RFM69::TxStartCondition::FifoLevel);
    radio->SetPreambleSize(0);
    radio->SetOcp(false);

    Serial.println("Init RFM69 Done!");
  }
  else {
    Serial.println("RFM ERROR!");
    radio.reset(nullptr);
  }
}

bool endOfFrame= false;
void loop() {
  if(radio) {
    switch (state) {
      case 0:
        Serial.print("Sending data frame...");
        // Write 64 first bytes to the fifo
        for(uint8_t i = 0; i < 64; i++) {
          if(frameBuilder.GetNextByte(currentByte))
            currentFrame.push_back(currentByte);
          else
            break;
        }
        radio->TransmitPacket(currentFrame);
        currentFrame.clear();
        radio->SetOcp(false);
        radio->SetOperatingMode(RFM69::Modes::Tx);
        state = 1;
        break;

      case 1:
        // Poll FIFO level
        if(!endOfFrame) {
          if(radio->IsIrqFlagSet(RFM69::IrqFlags2::FifoLevel) == false) {
            for(uint8_t i = 0; i < 32; i++) {
              if(frameBuilder.GetNextByte(currentByte)) {
                currentFrame.push_back(currentByte);
              }
              else {
                endOfFrame = true;
                break;
              }
            }
            radio->TransmitPacket(currentFrame);
            currentFrame.clear();
          }
        }
        else {
          state = 2;
        }
        break;
      case 2:
        // Wait end of TX
        if(packetReceived) {
          Serial.println("End of TX");
          radio->SetOperatingMode(RFM69::Modes::Standby);
          state = 3;
        }
        break;
      case 3:
        delay(100);
        break;
    }
  }
}

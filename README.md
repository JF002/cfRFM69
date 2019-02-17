# cfRFM69

RFM69 driver library for ESP32/Arduino designed to run on ESP32 but should work on any board supported by the Arduino framework with an SPI bus.

RFM69 is an RF transceiver (433/868Mhz) that can be driver by the SPI bus.

I wrote this library for [one of my project](https://github.com/JF002/RFM69_Doorbell), when I was trying to make my doorbell ring, and detect the push button.
There are many existing library, but none specifically written for the ESP32, so I wrote mine (because I can :) ).

## How to use it
```
#include <Arduino.h>
#include <RFM69.h>
#include <SPI.h>
#include <memory>

using namespace Codingfield::Communication;

SPIClass spi(HSPI);
std::unique_ptr<Codingfield::Communication::RFM69> radio;

void setup() {
  spi.begin();
  radio.reset(new RFM69(&spi, 15));
  if(radio->IsRfm69()) {
      auto temperature = radio->ReadTemperature();
      Serial.println("Temperature : " + String(temperature));

      radio->SetOperatingMode(RFM69::Modes::Standby);
      radio->SetDataModulation(RFM69::ModulationShapings::NoShaping,
                             RFM69::ModulationTypes::FSK,
                             RFM69::DataModes::Packet);
      radio->SetBitRate(0x1430); // 6.250 kpbs
      radio->SetFrequencyDeviation(0x0333); // 50Khz
      radio->SetFrequency(0x00D913E8); // 868Mhz
      
      // ...
  }
}  

```
See examples for more code!

## Links
* [RFM69_Doorbell](https://github.com/JF002/RFM69_Doorbell), my project at the origin of this library
* [RFM69 datasheet](https://cdn-shop.adafruit.com/product-files/3076/RFM69HCW-V1.1.pdf)


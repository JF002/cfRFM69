# RFM69_Doorbell
The goal of this project is to use an ESP32 with an RFM69 to receive/send the data frame used by my HoneyWell doorbell

## Setup
This code runs on a WROVER-KIT V3. It uses the hardware SPI driver 'HSPI'

![Setup](https://git.codingfield.com/JF/RFM69_Doorbell/src/commit/d7003958cc42eed6c6d79c02cf48114f11504fab/ressources/setup.jpg)


| ESP32 pin | RFM69 pin |
| --------- | --------- |
| GND       | GND       |
| 3V3       | Vin       |
| ---       | EN        |
| IO2       | G0        |
| IO14      | SCK       |
| IO12      | MISO      |
| IO13      | MOSI      |
| IO15      | CS        |
| IO4       | RST       |


## TX
TX works! The packet generated in this code makes my doorbell ring! It uses the packet mode and the FIFO of the RFM69.

## RX
RX does not work! I cannot manage to configure the RFM69 correctly so that it detects and decodes the data frame sent by the button. Any help and pull-request are welcomed!

## Ressources
* [RFM69 datasheet](https://cdn-shop.adafruit.com/product-files/3076/RFM69HCW-V1.1.pdf)
* [Honeywell "ActivLink" Wireless Devices on GitHub](https://github.com/klohner/honeywell-wireless-doorbell)
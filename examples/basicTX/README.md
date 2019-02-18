# Basic TX example
This example configures the RFM69 as Transmitter (TX) and sends 128Bytes using the FIFO. The FIFO is first filled
with 64B. Then, the transmission begins. While transmitting, the level of the FIFO is monitored, and filled with 32 new bytes
when at least 32Bytes have been sent.

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

## Ressources
* [RFM69 datasheet](https://cdn-shop.adafruit.com/product-files/3076/RFM69HCW-V1.1.pdf)
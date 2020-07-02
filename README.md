# Btlejack firmware

This repository contains the firmware source code used by Btlejack to communicate and attack BLE connections with Micro:Bit devices.

Three versions of this firmware can be compiled:

 * a version for the BBC Micro:Bit
 * a version for BLE400 and Adafruit's Bluefruit LE boards
 * a version for the nRF51822 chip on the [Bornhack Badge 2018](https://github.com/bornhack/badge2018)

To compile these versions:

```
$ make all
```

Firmware hex files are then available in the `dist` directory (not versioned).


## Links

[micro:bit runtime docs](http://lancaster-university.github.io/microbit-docs/) | [microbit-dal](https://github.com/lancaster-university/microbit-dal) |  [uBit](https://github.com/lancaster-university/microbit)

## Build Environments

| Build Environment | Documentation |
| ------------- |-------------|
| ARM mbed online | http://lancaster-university.github.io/microbit-docs/online-toolchains/#mbed |
| yotta  | http://lancaster-university.github.io/microbit-docs/offline-toolchains/#yotta |

## BBC Community Guidelines

[BBC Community Guidelines](https://www.microbit.co.uk/help#sect_cg)

## Using the Bornhack Badge

The Bornhack Badge 2018 has two microcontroller, the main Happy Gecko and an auxiliary nRF51822.
To program the nRF51822, one has to flash the [nrf51prog](https://github.com/bornhack/badge2018/tree/nrf51prog) to the main microcontroller.
Afterwards, an USB mass storage is emulated, where the Btlejack firmware can be copied to.

To connect the nRF51822 microcontroller to a computer, some headers needs to be soldered to the board.
The pins needs to be added to the twelve holes in the lower right, as shown in the [schematic](https://github.com/bornhack/badge2018/raw/hardware/schematic.pdf).
Finally, the ports 10/P0.09 and 12/P0.11 are TX and RX.

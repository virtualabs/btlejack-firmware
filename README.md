# Btlejack firmware

This repository contains the firmware source code used by Btlejack to communicate and attack BLE connections with Micro:Bit devices.

Two versions of this firmware can be compiled:

 * a version for the BBC Micro:Bit
 * a version for BLE400 and Adafruit's Bluefruit LE boards

To compile these two versions:

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

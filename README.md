
Synopsis
=================
This little firmware is written for the ESP8266 [1].
The goal is to transmit MIDI data [2] e.g. from an instrument (Device) to e.g. to a synthesizer (Host).
For this, two variants are supported:
 - Host: Opens a WIFI network 
 - Device: Connects to the WIFI network

MIDI data is transfered from Host to Device and vice versa via UDP.

This firmware is only tested on a ESP-01, but might be compatible to other boards.

Preparation
=================
See [3]

Ensure, that Python 2.7 is used.

Then, do 
 - `export SDK_PATH= $PATH/$TO/ESP8266_NONOS_SDK/`
 - `export BIN_PATH=./output`

In `$SDK_PATH/Makefile`: remove `-g` from `CCFLAGS`.


In `$SDK_PATH/driver_lib/Makefile`
 - set `FLAVOR` to `release` 
 - set `CCFLAGS += -O2`.
 - add absolute path of `./include` at the bottom to `INCLUDES`

It is recommended to adapt the `M2W_KEY`  `./include/user_config.h`.

Build
=================

Host:
```
make BOOT=none APP=0 SPI_SPEED=26.7 SPI_MODE=QIO SPI_SIZE_MAP=2 COMPILE=gcc  USER_DEFINES='-DM2W_VARIANT=M2W_VARIANT_HOST' 
```

Device:
```
make BOOT=none APP=0 SPI_SPEED=26.7 SPI_MODE=QIO SPI_SIZE_MAP=2 COMPILE=gcc  USER_DEFINES='-DM2W_VARIANT=M2W_VARIANT_DEVICE' 
```


Flashing
=================
esptool is the way to go:
```
esptool.py --port <PORT> write_flash --flash_freq 26m --flash_mode qio --flash_size 8m 0x00000 output/eagle.flash.bin 0x10000 output/eagle.irom0text.bin
```


Version Vistory
=================

 - v0.0.1: First experimental version, based on FreeRTOS (https://github.com/espressif/ESP8266_RTOS_SDK)
 - vx.x.x: Uses the non-OS SDK to improve the performance

References
=================

[1] https://en.wikipedia.org/wiki/ESP8266
[2] https://www.midi.org/
[3] https://github.com/espressif/ESP8266_NONOS_SDK

License
================

Copyright (c) 2018 Christian Haettich [feddischson@gmail.com]

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


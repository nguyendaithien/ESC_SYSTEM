# pico-drone 

- proccessor pi pico
- os freertos
- sensor mpu6050 , vl53xl1x


## Contents

- [Installation](#installation)

## Installation

#### Prerequisites

- The [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) has
been successfully installed and the `PICO_SDK_PATH` environment variable has
been appropriately set
- The [FreeRTOS Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel) has been
successfully installed and the `FREERTOS_KERNEL_PATH` environment variable has
been appropriately set

If the Prerequisites are satisfied, this repository can be cloned and built
with the following commands:

```
git clone this_one*
cd this*
mkdir build
cd build
cmake ..
make
```


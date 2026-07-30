# BMS Project

STM32U385CG BMS firmware (CMake + FreeRTOS + J-Link debug).

## Build

```bash
cmake -S . -B build/Debug -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build/Debug --target BMS_Project --parallel
```

Output: `build/Debug/BMS_Project.elf`

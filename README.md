# STM32_CMSIS
This repo is introduction to CMSIS (Cortex Microcontroller Software Interface Standard) based abstraction, which gives better control than HAL although both are drivers used for developing application, but CMSIS helps in using just what you need, it bypasses unecessary Initialization of peripherals unlike HAL.

## Nucleo Board (F401RE)
We have used F401RE board if not you can work with cheaper boards such as Blackpill as well just go through ST documentation (datasheets and reference manual)
![We have used this board](F401_diagram.png)

## Important Diagram!! (Under the hood block diagram)
Once going through the code relate the buses and peripherals using this diagram for more clarity 
[Click here diagram](STM32F401CC_INTERNAL.pdf)
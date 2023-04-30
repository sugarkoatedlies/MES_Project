# Making Embedded Systems Final Project

## Exercise 1

An embedded system ive dreamed about designing for a while, is an underwater light.
This light is water activated (using hardware, not software), when submerged. It records pressure and temperature(at the minimum), has a rechargeable battery (with charging chip), and has some sort of light transceiver (ir/visible) that is used to communicate with the device and modify internal settings(light color and intensity) and communicate data (temperature and pressure data) via serial.  It stores information on a flash chip that it reads and writes data to.

## Exercise 2
Final project board = STM32L476RG
https://os.mbed.com/platforms/ST-Nucleo-L476RG/

Other board I have = STM32F429ZiT6U discovery 0 board
Processor:STM32L476RGT6
Speed = 80 MHz
Timers = 16 timers
Flash size:1 MB
Ram size:128KB
other memory size: external memory interface for static memories
List 3-5 peripherals:CAN, I2C, QSPI, UART/USART, USB OTG
ADC features:3x12b
Board Cost: development board = $14.90
Processor Cost:~$10-$15
Processor stock and vendors:Digikey - 1153 ship immediately
Application Note:

Other board I have laying around = STM32F429ZiT6U discovery 0 board
https://os.mbed.com/platforms/ST-Discovery-F429ZI/

Processor:STM32F4
Speed = 180 MHz
Timers = up to 17 timers
Flash size:2MB
Ram size:256 x 8
other memory size:
List 3-5 peripherals:CAN, Ethernet, I2C, USB OTG, LINBUS, EBI/EMI, LCD TFT controller
ADC features:3 x 12bit ADC
Board Cost:Digikey - $29.90 - obsolete(i purchased~4 years ago)
Processor Cost:$21.81 individual
Processor stock and vendors:0 in stock, Digikey
Application Note:

#Exercise 3
Blinky on the STM32L476RG (re-use the HAL's to make things easier at first.)

1. On a board, make blinky for yourself.
2. Then add a button to turn the LED on and off.
3. Bonus points for making the button cause an interrupt. Triple bonus points for debouncing the button signal.

A. What are the hardware registers that cause the LED to turn on and off? (From the processor manual, don’t worry about initialization.)
B. What are the registers that you read in order to find out the state of the button?
C. Can you read the register directly and see the button change in a debugger or by printing out thes value of the memory at the register’s address?



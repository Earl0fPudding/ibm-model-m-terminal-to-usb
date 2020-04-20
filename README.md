# IBM Model M Terminal version to USB adapter
An easy mappable adapter for the IBM Model M terminal version so it can be used as a USB keyboard.

# Features
- an easy plug and play solution for an IBM Mode M keyboard which has a RJ45 or 8P8C connector
- all keys are mappable (todo)
- performing a certain key combination on the keyboard sets the microcontroller into map mode (todo)
- during map mode the user can log onto the microcontroller via virtual serial terminal and customize the mapping to their desire which will be saved to flash memory (todo)

# Requirements
## Hardware requirements
 - STM32L452 microcontroller
 - RJ45 also known as 8P8C connector
 - A USB connector
 - One 10 Ohm and one 22 Ohm Resistor
 - Two BJT transistors
 - A ST LINK for flashing the microcontroller
 
 ## Software requirements
 - the ST LINK flasher
 - the arm gcc compiler (arm-none-eabi-binutils, arm-none-eabi-gcc, arm-none-eabi-newlib)
 - Make
 
 # Instructions
 1. Get all the required components and connect them as shown in the [schematic diagram](https://github.com/Earl0fPudding/ibm-model-m-terminal-to-usb/raw/master/ibm-model-m-terminal-to-usb-schematic.pdf)
 2. Install all required software
 3. Clone this repository
 4. Open a terminal, navigate into the project directory and build it with the `make` command
 5. Plug the ST LINK connected to the STM32 into your PC
 6. Navigate to the newly created build directory and flash the compiled code onto the microcontroller with `st-flash write ibm-model-m-terminal-to-usb.bin 0x8000000`

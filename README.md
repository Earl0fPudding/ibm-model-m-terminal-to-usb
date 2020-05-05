# IBM Model M Terminal version to USB adapter
An easy mappable adapter for the IBM Model M terminal version so it can be used as a USB keyboard.

# Features
- an easy plug and play solution for an IBM Mode M keyboard which has a RJ45 or 8P8C connector
- all keys are mappable
- supports up to 50 different mapping configurations (todo)
- performing a certain key combination switches between configurations (todo)
- performing a certain key combination sets the microcontroller into map mode
- during map mode the user can log onto the microcontroller via virtual serial terminal and customize the mapping to their desire which will be saved to an eeprom memory chip (todo)

# Requirements
## Hardware requirements
 - STM32L452 microcontroller
 - 24LC256-I/SN eeprom
 - RJ45 also known as 8P8C connector
 - A USB connector
 - One 10 Ohm and one 22 Ohm Resistor
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

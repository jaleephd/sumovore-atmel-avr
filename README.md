# Sumovore Atmel AVR

The code here is for illustrating how to program the Atmel AVR microcontroller on the Sumovore ATmega8 brainboard.

Standard features of the Atmel AVR Microcontroller are utilised, such as pulse-width modulation (PWM),
analog to digital conversion (ADC), timer interrupts, and IO ports.
These are used to provide access to the sensors and actuators on the Solarbotics Sumovore robot,
and demonstrate how to implement simple behaviour-based robotics.

**_Important Note_**: I fully expect that some of the details below are out of date as they are based on 2005 versions.
Nevertheless, they are provided for illustrative and historical purposes.

## Files

* Sumovore_lineWanderer.c : a complete working line follower with obstacle avoidance, lost timeout, and sensor calibration.
* Sumovore_lineSensorMeter.c: tests the line sensor reposnses.
* Sumovore_linelevel_test_v1.c: same but simpler version.
* Sumovore_timerint.c: an example of using the timer interrupt.

Additionally, a generated Makefile is provided, only the `TARGET = wanderer` line needs to be modified to match the filename.

## Required Hardware
* 1 constructed Sumovore with Atmega8 Brainboard (and batteries)
* 1 SP12 serial programmer cable plugged into parallel port (LPT1). Hint: to ensure that cable is plugged into Sumovore
with the right orientation, use some green electrical tape on the side with the pin that will plug into the Sumovore socket
closest to the green LED.

## Software Development Environment

This code was developed on Windows XP in 2005 and used the following software development tools:
* WinAVR from http://winavr.sourceforge.net/
* AVR Studio 4.12 or later (from http://www.atmel.com/products/avr/ ) for AVR simulator (from version 4.12 C source-level simulation is supported)

The code was compiled using GNU GCC with avr-libc 1.2.3 (from WinAVR-20050214).
It may, or may not, require modifications to run on newer versions.

## Other Requirements

`giveio` is required to access lpt1, and can be installed by either:
* run WinAVR’s install giveio.bat from the command line;
* or using WinAVR’s avrdude-gui, click the Install button in the GiveIO Driver section.

## Generating a Makefile

To create the Makefile:
* Open Mfile, an automatic AVR GCC Makefile generator that comes with WinAVR and is placed as a shortcut on
the desktop.
* define `AVRDUDE_PROGRAMMER = sp12`
* define `AVRDUDE_PORT = lpt1`
* uncomment `AVRDUDE_WRITE_FLASH = ...` line
* comment `AVRDUDE_WRITE_EEPROM = ...` line
* (optional) define `MCU = atmega8`
* (optional) define `F_CPU = 1000000`
* Save file in directory where Sumovore code will be written.

## Compiling Instructions

At the command line enter “make all”, this will compile the code and create a `.hex` file for downloading to the
Sumovore, a `.elf` file which can be loaded by AVR Studio for simulation, and several other files that aren’t used here.

## Downloading Executable to the Sumovore Robot

To download the program to the Sumovore:
* Plug the programming cable into the Sumovore;
* Switch the Sumovore on (but leave the motors turned off);
* At the command line enter “make program”, which should result in the green LED flickering to show that the Sumovore
is being reprogrammed;
* Switch the Sumovore off;
* Un-plug the programming cable from the Sumovore.

## Running the program on the Sumovore Robot

Once the program has been downloaded to the Sumovore, to run the program, turn the Sumovore on.

## Credits

Line following behaviour is based on Solarbotics Sumoline.c by Bob Cook
(based on original by Grant McKee), http://www.solarbotics.com, 2005.

The makefile is generated from the WinAVR Sample makefile written by Eric B. Weddington, Jörg Wunsch, et al.
http://winavr.sourceforge.net/, 2005.

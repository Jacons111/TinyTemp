# makefile for AVR ATmega328P projects using Arduino Uno
# ECE-231 Spring 2023
# revision history
#	Date		Author			Revision
#	2/14/22		D. McLaughlin	initial release 
# 	2/15/22		D. McLaughlin	updated with corrections (thanks S. Kaza)
#	3/30/22		D. McLaughlin	updated for use with Sparkfun Pocket Programmer
# 	4/3/22		D. McLaughlin	tested on Windows 10 (parallels 17 on MBP Apple Silicon)
#	2/13/23		D. McLaughlin	simplified for Arduino Uno dev board only, ECE231 Spring 2023
#	12/11/23	D. McLaughlin	reconfigured for ATTiny25 & tinyTemp digital thermometer

#______________ MODIFY SERIALPORT AND SOURCEFILE_______________________
# Specify the com port (windows) or USB port (macOS)
# Use Device Manager to identify COM port number for Arduino Uno board in Windows
# In Terminal, type ls /dev/tty.usb* to determine USB port number in macOS
SERIALPORT = usb # previously /dev/tty.usbmodem101
# Specify the name of your source code here:
SOURCEFILE = main.c tinyOLED.c
#_____________________________________________________________________
# Don't change anything below unless you know what you're doing....
CLOCKSPEED = 1000000	#previously 16000000
PROGRAMMER = USBtiny # previously Ardiomp

begin:	main.hex

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avr-size --format=avr --mcu=atmega328p main.elf

main.elf: $(SOURCEFILE)
	avr-gcc -Wall -Os -DF_CPU=$(CLOCKSPEED) -mmcu=attiny85 -o main.elf $(SOURCEFILE)

flash:	begin
	avrdude -c $(PROGRAMMER) -b 115200 -P $(SERIALPORT) -p attiny85 -U flash:w:main.hex:i
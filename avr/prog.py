import os

os.system('D:\\tools\\AVRDUDE\\avrdude avrdude -p atmega8 -c usbasp -P usb -V  -U flash:w:"gateDriver.hex":i')
# os.system('avr-objcopy -j .text -j .data -O ihex main.elf central.hex')
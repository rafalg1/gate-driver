import os


arg = "avr-gcc -mmcu=atmega328 -Wall -Os -DF_CPU=8000000UL -o main.elf"

src = " main.c "
src += "Gate/gate.c "
src += "Uart/uart.c "

print(src)
os.system(arg+src)
# os.system('avr-gcc -mmcu=atmega328 -Wall -Os -DF_CPU=8000000UL -o main.elf main.c Pd/pd.c Uart/uart.c Transceiver/transceiver.c Stepper/stepper.c')


os.system('avr-objcopy -j .text -j .data -O ihex main.elf gateDriver.hex')
os.system('avr-size -C --mcu=atmega328 main.elf')
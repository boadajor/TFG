.PHONY : loadmain
loadmain: maquina_estats.hex
	avrdude -c arduino -p atmega328p -P /dev/ttyACM0 -U maquina_estats.hex
	rm -f *~ *.hex osc
	picocom -b115200 /dev/ttyACM0


maquina_estats:  serial_device.c  serial_device.h tmr0.c tmr0.h adc.c adc.h maquina_estats.c
# opcio 1 standard: 2368 bytes
	avr-gcc -std=c99 -Wall -Os -mmcu=atmega328p -fshort-enums -DF_CPU=16000000UL maquina_estats.c serial_device.c tmr0.c adc.c -o maquina_estats

maquina_estats.hex: maquina_estats
	avr-objcopy -Oihex maquina_estats maquina_estats.hex



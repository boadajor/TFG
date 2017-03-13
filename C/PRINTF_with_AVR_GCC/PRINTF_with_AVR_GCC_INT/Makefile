.PHONY : loadmain
loadmain: osc.hex
	avrdude -c arduino -p atmega328p -P /dev/ttyACM0 -U osc.hex
	rm -f *~ *.hex osc
	picocom /dev/ttyACM0


osc:  serial_device.c  control_TMR0.c control_TMR0.h osc.c
# opcio 1 standard: 2368 bytes
	avr-gcc -std=c99 -Wall -Os -mmcu=atmega328p -fshort-enums -DF_CPU=16000000UL osc.c serial_device.c  control_TMR0.c -o osc

# opcio 2 minimized 2014 bytes
#	avr-gcc -std=c99 -Wall -Os -mmcu=atmega328p -fshort-enums -Wl,-u,vfprintf -lprintf_min -DF_CPU=16000000UL osc.c serial_device.c  control_TMR0.c -o osc

# opcio 3 with float 3888 bytes
#	avr-gcc -std=c99 -Wall -Os -mmcu=atmega328p -fshort-enums -Wl,-u,vfprintf -lprintf_flt -lm -DF_CPU=16000000UL osc.c serial_device.c  control_TMR0.c -o osc

osc.hex: osc
	avr-objcopy -Oihex osc osc.hex



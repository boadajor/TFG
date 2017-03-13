#include <stdbool.h>
#include <avr/interrupt.h>
#include <math.h>
#include "control_TMR0.h"
#include "utils.h"
#include "serial_device.h" //pel serial
#include <stdio.h> //pel serial 


/********************************************
        PRINTF (CHAR i ENTER) AMB AVR
---------------------------------------------
**********************
Envia chars i enters pel port sèrie cada segon començant a ini
Amb la configuració feta al makefile NO POT pot enviar floats
**********************
MODIFICACIONS AL MAKEFILE: si no es fa res surt l'standard


The heart of stdio.h library is a vfprintf() (and vscanf() for scan) function ...
Depending on linker options it can act a bit differently. There can be three different options selected: 

1) STANDARD with no float conversions 
If you create projects with ... AVR GCC tools by default linker options are set to standard with no floating point conversions.

2) MINIMIZED
If you only need to send strings with basic integer numbers, use minimized version. To do so use linker option:

-Wl,-u,vfprintf -lprintf_min

3) WITH FLOAT (alerta, pesa molt)
In other hand if you need to a floating point functionality use standard option with floating point conversions:

-Wl,-u,vfprintf -lprintf_flt -lm

Modificat 20140327
Modificat 20141019

*********************************************/
#define ini 48

static int uart_putchar(char c, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL,_FDEV_SETUP_WRITE);
static int uart_putchar(char c, FILE *stream){
  if (c == '\n')
    uart_putchar('\r', stream);
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;
  return 0;
}

void setup(){
  setup_TMR0();
  serial_init();
  sei();
}

int main(void){
  setup();
  stdout = &mystdout;
  printf("\n\nHola nois! \n\n");
  while(true){
  }
  return 0;
}

ISR(TIMER0_COMPA_vect){
  sbi(PORTD,PD4);

  static uint8_t n=0;
  static uint8_t int8=ini;
  static uint16_t int16=ini;
  static uint32_t int32=ini;
  
  n++;
  if (n==0) {
    int8++;
    int16++;
    int32++;
    
    printf("\nchar: %c\n", int8);
    printf("hex: 0x%X\n", int8);
    printf("octal: %o\n", int8);
    printf("uint8: %d\n",int8);
    printf("uint16 %d\n",int16);
    printf("uint32 %ld\n",int32);
  }
  
  cbi(PORTD,PD4);
}

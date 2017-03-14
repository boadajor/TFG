/*
Maquina d'estats per a la deteccio de veu.
Es configura T mostreig (T_m) amb TMR0.
T_m=(OCR0A+1)/2 us
*/

#include <avr/interrupt.h>
#include "adc.h"
#include "tmr0.h"
#include <stdbool.h>
#include "serial_device.h"
#include <stdio.h>

static const uint8_t ocr0a=249; //per fer F_m=8kHz, T_m=125us
static const int llindar_silenci = 40; //valor maxim de senyal d'entrada considerat com a silenci
uint8_t input=0;

static void app_init(){ //inicialitzacio de variables
  setup_ADC(0,5,16);//(adc_input,v_ref,adc_pre)
  //adc_input (0-5 (default=5),8 Tª, 14 1.1V, 15 GND 
  //v_ref 0 (AREF), 1(1.1V), default=5 (5V)
  //adc_pre 2,4,8,16(default),32,64,128
  setup_tmr0(ocr0a,8);
  serial_init();
  sei();
  DDRB |= _BV(DDB5);
}

int main(void){
  app_init();
  while(true){
    printf("%d\n", input);
    if ((input>llindar_silenci) | (input<(-llindar_silenci))){
      // counter();
      /* set pin 5 high to turn led on */
      PORTB |= _BV(PORTB5);
    }
    else{
      /* set pin 5 low to turn led off */
      PORTB &= ~_BV(PORTB5);
    }
  }
  return 0;
}

ISR(TIMER0_COMPA_vect){
  start_ADC();
  input=read8_ADC();
}

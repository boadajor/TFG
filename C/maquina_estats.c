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
#include <stdlib.h>

static const uint8_t ocr0a=249; //per fer F_m=8kHz, T_m=125us
static const int llindar_silenci = 28000; //valor maxim de senyal d'entrada considerat com a silenci
uint8_t input=0; //valor llegit despres de l'ADC
int segment[50]; //ultimes 50 mostres
uint32_t power=0; //potencia
uint16_t sum=0; //sumatori de les ultimes 50 mostres

static int uart_putchar(char c, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL,_FDEV_SETUP_WRITE);
static int uart_putchar(char c, FILE *stream){
  if (c == '\n')
    uart_putchar('\r', stream);
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;
  return 0;
}

static void app_init(){ //inicialitzacio de variables
  setup_ADC(0,5,16);//(adc_input,v_ref,adc_pre)
  //adc_input (0-5 (default=5),8 Tª, 14 1.1V, 15 GND 
  //v_ref 0 (AREF), 1(1.1V), default=5 (5V)
  //adc_pre 2,4,8,16(default),32,64,128
  setup_tmr0(ocr0a,8);
  serial_init();
  sei();
  DDRB |= _BV(DDB5);//pin 13 com a sortida per debuggar amb LED
}

int main(void){
  stdout = &mystdout;
  app_init();
  while(true){
    power=(sum*sum)/2; //calcul de la potencia
    printf("%ld\n", power);
    if (power>llindar_silenci) {
    //if ((input>llindar_silenci) || (input<(-llindar_silenci))){
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
  int i=49;
  sum=0;
  start_ADC();
  input=read8_ADC();
  while(i>0){
    segment[i+1]=segment[i]; //desplaçament de mostres
    sum+=abs(segment[i]); //sumatori de mostres
    i-=1;
  }
  segment[1]=segment[0];
  segment[0]=input;
  sum+=abs(input);
}

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

typedef enum { //Estat de la maquina
  Data,
  Silence,
  Isitdata,
  Isitsilence,
} machine_state_t;

static const uint8_t ocr0a=249; //per fer F_m=8kHz, T_m=125us
static const int llindar_silenci = 28000; //valor maxim de senyal d'entrada considerat com a silenci
uint8_t input=0; //valor llegit despres de l'ADC
int segment[50]; // mida de la finestra. Ultimes 50 mostres ->6.25ms
uint32_t power=0; //potencia
uint8_t sum=0; //compta quants cops entrem a l'estat DATA
uint8_t counter=0;
uint8_t index=0;//index de la cua circular

machine_state_t estat=Silence;

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

static int calculs(){
  uint32_t result=0;
  int i=0;
  while (i<51){
    result+=((segment[i]*segment[i])/2);
    i+=1;
  }
  return result; 
}

static void canvi_estat(){
  power=calculs();
  printf("el power val: %ld\n", power);
  index=0;
  switch(estat){
  case Silence:
    counter=0;
    PORTB &= ~_BV(PORTB5);
    if (power>llindar_silenci){
      counter+=1;
      estat=Isitdata;
    }
    break;
  case Isitdata:
    if (power>llindar_silenci){
      if (counter<5)
	counter+=1;
      else
	estat=Data;
    }
    else{
      counter=1;
      estat=Isitsilence;
    }
    break;
  case Data:
    sum+=1;
    counter=0;
    PORTB |= _BV(PORTB5);
    if (power<llindar_silenci){
      counter+=1;
      estat=Isitsilence;
    }
    else{
      counter+=1;
      printf("el counter val: %d\n", counter);
    }
    break;
  case Isitsilence:
    if (power<llindar_silenci){
      if (counter<5)
	counter+=1;
      else
	estat=Silence;
    }
    else{
      counter=1;
      estat=Isitdata;
    }
    break;
  }
}
  

int main(void){
  stdout = &mystdout;
  app_init();
  while(true){      
  }
  return 0;
}
  
ISR(TIMER0_COMPA_vect){
  input=read8_ADC();  
  start_ADC();
  segment[index]=input;
  index+=1;
  if (index=50){
    canvi_estat();
    index=0;
  }
}

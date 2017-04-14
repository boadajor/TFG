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
#define MIDA 50 
#define CONTINUA 128
#define LLINDAR 28000

typedef enum { //Estat de la maquina
  Data,
  Silence,
  Isitdata,
  Isitsilence,
} machine_state_t;

static const uint8_t ocr0a=249; //per fer F_m=8kHz, T_m=125us
uint8_t input=0; //valor llegit despres de l'ADC
uint8_t segment[MIDA]; // mida de la finestra. Ultimes 50 mostres ->6.25ms
volatile int32_t power=0; //potencia
uint8_t data_blocks=0; //compta quants cops entrem a l'estat DATA
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
  DDRD |=(1<<DDD4);//pin 4 com a sortida per flag de debuggeig
}
/*
static int32_t calculs(){
  int32_t result=0;
  int i=0;
  while (i<=MIDA){
    result+=(segment[i]*segment[i]);
    //result+=abs(segment[i]);
    i+=1;
  }
  //printf("el power val: %ld\n", result);
  return result; 
}
*/
static void canvi_estat(){
  switch(estat){
  case Silence:
    serial_put('0');
    counter=0;
    PORTB &= ~_BV(PORTB5);
    if (power>LLINDAR){
      counter+=1;
      estat=Isitdata;
    }
    break;
  case Isitdata:
    serial_put('+');
    if (power>LLINDAR){
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
    serial_put('1');
    data_blocks+=1;
    counter=0;
    PORTB |= _BV(PORTB5);
    if (power<LLINDAR){
      counter+=1;
      estat=Isitsilence;
    }
    else{
      counter+=1;
      //printf("el counter val: %d\n", counter);
    }
    break;
  case Isitsilence:
    serial_put('-');
    if (power<LLINDAR){
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
  PORTD |= (1<<PD4);
  int8_t input=read8_ADC()-CONTINUA;  //pot donar problemes amb continua diferent de 127 -> solucio int16_t
  //int16_t input=(int16_t)read8_ADC()-CONTINUA; //alternativa
  start_ADC();
  index+=1;
  power+=(input*input);
  if (index==MIDA){
    //printf("%ld\n", power);
    canvi_estat();
    index=0;
    power=0;
  }
  PORTD &= ~(1<<PD4);
}

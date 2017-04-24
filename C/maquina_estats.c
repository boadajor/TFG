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
#define VALIDBLOCK 80 //80 mostres de sorol = 500ms de soroll
#define SHORTPAUSE 80
#define LONGPAUSE 160

typedef enum { //Estat de la maquina detectora de senyal
  Data,
  Silence,
  Isitdata,
  Isitsilence,
} machine_state_t;

typedef enum { //Estat de la maquina detectora de senyal
  Pause,
  Block,
  Action,
} state_t;

static const uint8_t ocr0a=249; //per fer F_m=8kHz, T_m=125us
uint8_t input=0; //valor llegit despres de l'ADC
uint8_t segment[MIDA]; // mida de la finestra. Ultimes 50 mostres ->6.25ms
volatile int32_t power=0; //potencia
uint8_t data_blocks=0; //compta quants cops entrem a l'estat DATA
uint8_t ncounter=0; //quantitat de mostres en senyal
uint8_t scounter=0; //quantitat de mostres en silenci
uint8_t index=0;//index de la cua circular
bool estataux=false;

machine_state_t estat=Silence;
state_t state=Pause;

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
  DDRD |=(1<<DDD5);
  DDRD |=(1<<DDD6);
  DDRD |=(1<<DDD7);
}

static void canvi_estat(){
  switch(estat){
  case Silence:
    estataux=false;
    serial_put('0');
    ncounter=0;
    PORTB &= ~_BV(PORTB5);
    if (power>LLINDAR){
      ncounter=1;
      estat=Isitdata;
    }
    else
      scounter+=1;
    break;
  case Isitdata:
    serial_put('+');
    if (power>LLINDAR){
      if (ncounter<5)
	ncounter+=1;
      else{
	ncounter+=1;
	estat=Data;
      }
    }
    else{
      scounter+=1;
      estat=Isitsilence;
    }
    break;
  case Data:
    estataux=true;
    serial_put('1');
    data_blocks+=1;
    scounter=0;
    PORTB |= _BV(PORTB5);
    if (power<LLINDAR){
      scounter=1;
      estat=Isitsilence;
    }
    else{
      ncounter+=1;
      //printf("el counter val: %d\n", counter);
    }
    break;
  case Isitsilence:
    serial_put('-');
    if (power<LLINDAR){
      if (scounter<5)
	scounter+=1;
      else{
	scounter+=1;
	estat=Silence;
      }
    }
    else{
      ncounter=1;
      estat=Isitdata;
    }
    break;
    }
}
  

int main(void){
  stdout = &mystdout;
  app_init();
  uint8_t blockcount=0;
  while(true){
    switch(state) {
    case Pause:
      if (estataux==true){
	if (ncounter>=VALIDBLOCK)
	  state=Block;
      }
      else{
	if (scounter>=LONGPAUSE)
	  state=Action;
      }
      break;
    case Block:
      blockcount+=1;
	if (scounter>=SHORTPAUSE)
	  state=Pause;
      break;
    case Action:
      switch(blockcount) {
      case 1:
	//OK
	PORTD |= _BV(PORTD5);
	break;
      case 2:
	//Llanterna
	PORTD |= _BV(PORTD6);
	break;
      case 3:
	//Radio
	PORTD |= _BV(PORTD7);
	break;
      default:
	//SOS
	break;
	blockcount=0;
	state=Pause;
      }
      break;
    }
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

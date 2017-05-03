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
#define LLINDAR 28000//28000
#define TS1 80 //80 mostres de silenci = 500ms de silenci
#define TS2 160
#define TD1 80
#define TD2 160

typedef enum { //Estat de la maquina detectora de senyal
  Data,
  Silence,
  Isitdata,
  Isitsilence,
} machine_state_t;

typedef enum { //Estat de la maquina detectora de senyal
  Inicial,
  Primersilenci,
  Block,
  Segonsilenci
} state_t;

typedef enum { //Estat de la maquina detectora de senyal
  ON,
  OFF
} state3_t;

static const uint8_t ocr0a=249; //per fer F_m=8kHz, T_m=125us
uint8_t input=0; //valor llegit despres de l'ADC
uint8_t segment[MIDA]; // mida de la finestra. Ultimes 50 mostres ->6.25ms
volatile int32_t power=0; //potencia
volatile int32_t power2=0; //copia de la potencia
uint8_t index=0;//index de la cua circular
volatile bool flag_int=false;
bool flag_silenci=false;
bool flag_data=false;

machine_state_t estat=Silence;
state_t estat2=Inicial;
state3_t estat3=OFF;

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
  

int main(void){
  stdout = &mystdout;
  app_init();
  uint8_t aux_counter=0;
  uint8_t ts=0; //controla temps a la maquina2
  uint8_t td=0; //controla temps a la maquina2
  bool flag_pols=false;
  while(true){
    /*
      /MAQUINA 1:
    */
    if (flag_int){
      switch(estat){
      case Silence:
	//serial_put('0');
	//PORTB &= ~_BV(PORTB5);
	flag_silenci=false;
	if (power2>LLINDAR){
	  aux_counter=1;
	  estat=Isitdata;
	}
	break;
      case Isitdata:
	//serial_put('+');
	if (power2>LLINDAR){
	  if (aux_counter<5)
	    aux_counter+=1;
	  else{
	    //serial_put('1');
	    flag_data=true;
	    estat=Data;
	    aux_counter=0;
	  }
	}
	else{
	  estat=Silence;
	  aux_counter=0;
	}
	break;
      case Data:
	//serial_put('1');
	//PORTB |= _BV(PORTB5);
	flag_data=false;
	if (power2<LLINDAR){
	  aux_counter=1;
	  estat=Isitsilence;
	}
	break;
      case Isitsilence:
	//serial_put('-');
	if (power2<LLINDAR){
	  if (aux_counter<5)
	    aux_counter+=1;
	  else{
	    //serial_put('0');
	    flag_silenci=true;
	    estat=Silence;
	    aux_counter=0;
	  }
	}
	else{
	  estat=Data;
	}
	break;
      }
      /*
	/MAQUINA 2:
      */
      switch(estat2){
      case Inicial:
	if(flag_silenci){
	  estat2=Primersilenci;
	  serial_put('a');
	  ts=0;
	}
	break;
      case Primersilenci:
	flag_pols=false;
	ts+=1;
	if(flag_data){
	  if(ts>TS1){
	    estat2=Block;
	    serial_put('B');
	    td=0;
	  } 
	  else{
	    estat2=Inicial;
	    serial_put('I');
	  }
	}
	break;
      case Block:
	td+=1;
	if(td>TD2){
	  estat2=Inicial;
	  serial_put('I');
	}
	else{
	  if(flag_silenci){
	    ts=0;
	    if(td>TD1){
	      estat2=Segonsilenci;
	      serial_put('2');
	    }
	    else{
	      estat2=Primersilenci;
	      serial_put('b');
	    }
	  }
	}
	break;
      case Segonsilenci:
	ts+=1;
	if(ts>TS2){
	  estat2=Primersilenci;
	  serial_put('c');
	  ts=0;
	  //ACCIO!!
	  flag_pols=true;
	}
	else{
	  if(flag_data){
	    estat2=Inicial;
	    serial_put('I');
	  }
	}
	break;
      }
      /*
	/MAQUINA 3:
      */
      switch(estat3){
      case ON:
	if(flag_pols){
	  estat3=OFF;
	  PORTB |= _BV(PORTB5);
	}
	break;
	case OFF:
	if(flag_pols){
	  estat3=ON;
	  PORTB &= ~_BV(PORTB5);
	}
	break;
      }
      flag_int=false;
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
    power2=power;
    flag_int=true;
    index=0;
    power=0;
  }
  PORTD &= ~(1<<PD4);
}

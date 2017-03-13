#include <avr/interrupt.h>
#include "adc.h"
#include "tmr0.h"
#include <stdbool.h>
#include "serial_device.h"
#include <stdio.h>

/********************************************
                Clock measurement
---------------------------------------------
This program takes samples from the analog
input A5 of the Ardiuno UNO Board.
The conversion time is 13/fadc_clk, fadc_clk=16e6/adc_pre.

An interrupt is activated each 50us (ocr0a=99, tmr0_pre=8)

The digital output PD4 is set to '1'
at the beginning of the ISR
and to '0' at the end.

26 September 2014
*********************************************/

static uint8_t ocr0a=100;

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
  setup_ADC(5,5,16);//(adc_input,v_ref,adc_pre)
  //adc_input (0-5 (default=5),8 Tª, 14 1.1V, 15 GND 
  //v_ref 0 (AREF), 1(1.1V), default=5 (5V)
  //adc_pre 2,4,8,16(default),32,64,128
  setup_tmr0(ocr0a,8);//(ocr0a, tmr0_pre)
  //tmr0_pre 1,default=8,64,256,1024
  //TMR0=prescaler*(ocr0a+1)*T_clk
  serial_init();
  sei();
}

float calcul(int V){
  float Fm, Fo, Fa;
  Fm=20000/(1+(1/2*V));
  Fo=Fm/(2*V);
  Fa=16000000*(20000-Fo)/20000;
  return Fa;
}

int main(void){
  stdout = &mystdout;
  setup();
  while(1){}
}

ISR(TIMER0_COMPA_vect){
  PORTD |= (1<<PD4);
  start_ADC();
  uint8_t value=read8_ADC();
  set_pwm_tmr2(value);
  switch(state){
  case E0:
    if (value > 128 + Delta){
      state=E1;}
    else{
      state=E0;}
    break;
  case E1:
    if (value < 128 - Delta){
      state=E2;}
    else{
      state=E1;}
    break;
  case E2:
    if (value > 128 + Delta){
      state=E3;}
    else{
      state=E2;}
    break;
  case E3:
    if (value < 128 - Delta){
      state=E4;}
    else{
      Nmostres++;
       state=E3;}
    break;
  case E4:
    printf("La frequencia de l'arduino es: %.5f \n", calcul(Nmostres));
    state=E5;
    break;
  case E5:
    while (1){}
  }
  PORTD &= ~(1<<PD4);
}

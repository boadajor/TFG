#include <ADXL345.h>
#include <Wire.h>

//--------------------------------------------------------------------------------------//
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "adc.h"
#include "tmr0.h"
//---------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------//

#define ADXL345_DATA_READY				0x07
#define ADXL345_SINGLE_TAP				0x06
#define ADXL345_DOUBLE_TAP				0x05
#define ADXL345_ACTIVITY				0x04
#define ADXL345_INACTIVITY				0x03
#define ADXL345_FREE_FALL				0x02
#define ADXL345_WATERMARK				0x01
#define ADXL345_OVERRUNY				0x00
#define ADXL345_DEVICE (0x53)    // Device Address for ADXL345

ADXL345 adxl = ADXL345();

boolean caiguda=false; 
boolean cop=false;
boolean inactivitat=false;
long emergency_counter; //comptador de temps pel protocol d'emergencia

//-----------------------------------------------------------------------------------//

#define MIDA 50 
#define CONTINUA 128//128+7
#define LLINDAR 5000//28000
#define TS1 20 //80 mostres de silenci = 500ms de silenci
#define TS2 100
#define TD1 20
#define TD2 80
#define CINC 800 //5s per confirmar emergencia
#define HISTERESI 10
#define TRENTA 4800
#define DEU 1600
#define VINT 3200
#define QUINZE 2400

typedef enum { //Estat de la maquina detectora de senyal
  Data,
  Silence,
  Isitdata,
  Isitsilence,
} machine_state_t;

typedef enum { //Estat de la maquina de blocks
  Inicial,
  Primersilenci,
  Block,
  Segonsilenci
} state_t;

typedef enum { //Estat de les maquines d'accio
  ON,
  OFF
} state3_t;

typedef enum { //Estat de la maquina PTT
  Ok,
  Avis,
  Confirmacio,
  Emergencia
} state31_t;

typedef enum { //Estat de la maquina detectora d'accidents
  Espera,
  Alerta,
  Activitat,
  Accident
} accident_state_t;

static const uint8_t ocr0a=249; //per fer F_m=8kHz, T_m=125us
//uint16_t input=0; //valor llegit despres de l'ADC
uint8_t segment[MIDA]; // mida de la finestra. Ultimes 50 mostres ->6.25ms
volatile int32_t power=0; //potencia
volatile int32_t power2=0; //copia de la potencia
uint8_t index=0;//index de la cua circular
volatile bool flag_int=false;
bool flag_silenci=false;
bool flag_data=false;

machine_state_t estat=Silence;
state_t estat2=Primersilenci;
state3_t estat32=OFF;
state31_t estat31=Ok;
accident_state_t acstate=Espera;

//--------------------------------------------------------------------------------------//

void setup(){
  Serial.begin(9600);
  Serial.println("SETUP");
  emergency_counter=0;
  Wire.begin();
  adxl.init(ADXL345_DEVICE);
  adxl.setRangeSetting(4);           // Give the range settings
  // Accepted values are 2g, 4g, 8g or 16g
  // Higher Values = Wider Measurement Range
  // Lower Values = Greater Sensitivity
  adxl.setActivityX(1);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityY(1); 
  adxl.setActivityZ(1); 
  adxl.setActivityThreshold(17);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)
 
  adxl.setInactivityX(0);  // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setInactivityY(0); 
  adxl.setInactivityZ(0);
  adxl.setInactivityThreshold(17);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  adxl.setTimeInactivity(CINC);         // How many seconds of no activity is inactive?

  adxl.setTapDetectionOnX(1); // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setTapDetectionOnY(1);
  adxl.setTapDetectionOnZ(1);
 
  // Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
  adxl.setTapThreshold(300);           // 62.5 mg per increment
  adxl.setTapDuration(15);            // 625 μs per increment
  adxl.setDoubleTapLatency(80);       // 1.25 ms per increment
  adxl.setDoubleTapWindow(200);       // 1.25 ms per increment
 
  // Set values for what is considered FREE FALL (0-255)
  adxl.setFreeFallThreshold(7);       // (5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(30);       // (20 - 70) recommended - 5ms per increment
 
  // Setting all interupts to take place on INT1 pin
  //adxl.setImportantInterruptMapping(1, 1, 1, 1, 1);     // Sets "adxl.setEveryInterruptMapping(single tap, double tap, free fall, activity, inactivity);" 
  // Accepts only 1 or 2 values for pins INT1 and INT2. This chooses the pin on the ADXL345 to use for Interrupts.
  
  // Turn on Interrupts for each mode (1 == ON, 0 == OFF)
  adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 0);
  
  //attachInterrupt(digitalPinToInterrupt(interruptPin), ADXL_ISR, RISING);   // Attach Interrupt

  //--------------------------------------------------------------------------------------//
  setup_ADC(1,5,16);//(adc_input,v_ref,adc_pre)
  //adc_input (0-5 (default=5),8 Tª, 14 1.1V, 15 GND 
  //v_ref 0 (AREF), 1(1.1V), default=5 (5V)
  //adc_pre 2,4,8,16(default),32,64,128
  setup_tmr0(ocr0a,8);
  sei();
  DDRB |= _BV(DDB5);//pin 13 com a sortida per debuggar amb LED
  DDRD |=(1<<DDD4);//pin 4 com a sortida
  DDRD |=(1<<DDD5);
  DDRD |=(1<<DDD6);
  DDRD |=(1<<DDD7);
  DDRC |=(1<<DDC2);
  DDRC |=(1<<DDC3);
  //--------------------------------------------------------------------------------------//
  PORTD |= _BV(PORTD5); //Encendre verd
}

void loop(){
  //-------------------------------------------------------------------------------------//
  uint8_t aux_counter=0;
  uint8_t ts=0; //controla temps a la maquina2
  uint8_t td=0; //controla temps a la maquina2
  uint8_t pok=0; //nombre de polsos valids
  bool flag_accio=false;
  uint16_t conf_temp=0; //temps per confirmar PTT
  while(true){
    /*
      /MAQUINA 1:
    */
    if (flag_int){
      switch(estat){
      case Silence:
	flag_silenci=false;
	if (power2>LLINDAR){
	  aux_counter=1;
	  estat=Isitdata;
	}
	break;
      case Isitdata:
	if (power2>LLINDAR){
	  if (aux_counter<HISTERESI)
	    aux_counter+=1;
	  else{
	    flag_data=true;
	    estat=Data;
	    PORTC |= _BV(PORTC3);
	    aux_counter=0;
	  }
	}
	else{
	  estat=Silence;
	  aux_counter=0;
	}
	break;
      case Data:
	flag_data=false;
	if (power2<LLINDAR){
	  aux_counter=1;
	  estat=Isitsilence;
	}
	break;
      case Isitsilence:
	if (power2<LLINDAR){
	  if (aux_counter<HISTERESI)
	    aux_counter+=1;
	  else{
	    flag_silenci=true;
	    estat=Silence;
	    PORTC &= ~_BV(PORTC3);
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
	  Serial.print("1");
	  ts=0;
	}
	break;
      case Primersilenci:
	if(ts<20){
	  ts+=1;
	  if(flag_data){
	    estat2=Inicial;
	    Serial.print("i");
	  }
	}
	else{
	  ts+=1;
	  flag_accio=false;
	  pok=0;
	  if(flag_data){
	    estat2=Block;
	    Serial.print("B");
	    td=0;
	  }
	}
	break;
      case Block:
	td+=1;
	if(td>TD2){
	  estat2=Inicial;
	  Serial.print("i");
	}
	else{
	  if(flag_silenci){
	    ts=0;
	    if(td>TD1){
	      estat2=Segonsilenci;
	      Serial.print("2");
	    }
	    else{
	      estat2=Primersilenci;
	      Serial.print("1");
	    }
	  }
	}
	break;
      case Segonsilenci:
	ts+=1;
	if(ts>TS2){
	  pok+=1;
	  estat2=Primersilenci;
	  Serial.print("1");
	  //ACCIO!!
	  flag_accio=true;
	}
	else{
	  if(flag_data){
	    if(ts>TS1){
	      pok+=1;
	      td=0;
	      estat2=Block;
	      Serial.print("B");
	      //POLS VALID!!
	    }
	    else{
	      //POLS INVALID
	      estat2=Inicial;
	      Serial.print("i");
	    }
	  }
	}
	break;
      }
      /*
	/MAQUINA 3.1: Emergencies
      */
      switch(estat31){
      case Ok:
	if(flag_accio && pok==3){
	  estat31=Confirmacio;
          PORTD |= _BV(PORTD6); //Encenem taronja
          PORTD &= ~_BV(PORTD5); //Apaguem verd
	  conf_temp=0;
	}
	break;
      case Avis:
        conf_temp+=1;
	if(flag_accio && pok==4){
	  estat31=Ok;
	  PORTD |= _BV(PORTD5); //Encendre verd
          PORTD &= ~_BV(PORTD6); //Apagar taronja
	  Serial.println("R");
          acstate=Espera;
	  caiguda=false;
	  cop=false;
	  inactivitat=false;
	  emergency_counter=0;
	  conf_temp=0;
	  adxl.setInactivityX(0);  // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
	  adxl.setInactivityY(0); 
	  adxl.setInactivityZ(0);
	}
        else{
          if((flag_accio && pok==3) | conf_temp>=VINT){
            estat31=Emergencia;
            PORTD |= _BV(PORTD7); //Encendre vermell
            PORTD &= ~_BV(PORTD6); //Apagar taronja
            conf_temp=0;
          }
        }
	break;
      case Confirmacio:
	conf_temp+=1;
	if(conf_temp>DEU | (flag_accio && pok==4)){
	  estat31=Ok;
          PORTD |= _BV(PORTD5); //Encendre verd
          PORTD &= ~_BV(PORTD6); //Apagar taronja
	  Serial.println("R");
	  conf_temp=0;
	}
	else{
	  if(flag_accio && pok==3){
	    estat31=Emergencia;
	    PORTD |= _BV(PORTD7); //Encendre vermell
            PORTD &= ~_BV(PORTD6); //Apagar taronja
	  }
	}
	break;
      case Emergencia:
	if(flag_accio && pok==4){
	  estat31=Ok;
	  PORTD |= _BV(PORTD5); //Encendre verd
          PORTD &= ~_BV(PORTD7); //Apagar vermell
	  Serial.println("R");
	  caiguda=false;
	  cop=false;
	  inactivitat=false;
	  emergency_counter=0;
	  conf_temp=0;
	  adxl.setInactivityX(0);  // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
	  adxl.setInactivityY(0); 
	  adxl.setInactivityZ(0);
	}
	break;
      }
      /*
	/MAQUINA 3.2: LED
      */
      switch(estat32){
      case ON:
	if(flag_accio && pok==2){
	  estat32=OFF;
	  PORTC &= ~_BV(PORTC2);
	}
	break;
      case OFF:
	if(flag_accio && pok==2){
	  estat32=ON;
	  PORTC |= _BV(PORTC2);
	}
	break;
      }
//----------------------------------------------------------------------------------//
      switch(acstate){
      case Espera:
	if(caiguda | cop){
	  acstate=Alerta;
	}
	break;
      case Alerta:
        emergency_counter+=1;
	if(caiguda & cop){
	  acstate=Accident;
	  Serial.println("ac1");
        }
        else{
          if (emergency_counter==CINC){ //5 segons d'espera
            acstate=Activitat;
          }
        }
	break;
      case Activitat:
	emergency_counter+=1;
	if(inactivitat){
	  acstate=Accident;
	  Serial.println("ac2");
	}
	else{
	  if(emergency_counter==QUINZE){
	    Serial.println("R");
	    caiguda=false;
	    cop=false;
	    inactivitat=false;
	    emergency_counter=0;
            acstate=Espera;
	    adxl.setInactivityX(0);  // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
	    adxl.setInactivityY(0); 
	    adxl.setInactivityZ(0);
	  }
	}
	break;
      case Accident:
	//PROTOCOL ACCIDENT
        Serial.println("AC");
        emergency_counter=0;
        estat31=Avis;
        caiguda=false;
	cop=false;
	inactivitat=false;
	emergency_counter=0;
        acstate=Espera;
	adxl.setInactivityX(0);  // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
	adxl.setInactivityY(0); 
	adxl.setInactivityZ(0);
        PORTD |= _BV(PORTD6); //Encenem taronja
        PORTD &= ~_BV(PORTD5); //Apaguem verd
	break;
      }
      //-------------------------------------------------------------------------------------//
      flag_int=false;
    }
  
  
    ADXL_ISR();
    // You may also choose to avoid using interrupts and simply run the functions within ADXL_ISR(); 
    //  and place it within the loop instead.  
    // This may come in handy when it doesn't matter when the action occurs. 
  }
}
/********************* ISR *********************/
/* Look for Interrupts and Triggered Action    */
void ADXL_ISR() {
  
  // getInterruptSource clears all triggered actions after returning value
  // Do not call again until you need to recheck for triggered actions
  byte interrupts = adxl.getInterruptSource();

  // Free Fall Detection
  if(adxl.triggered(interrupts, ADXL345_FREE_FALL)){
    //Serial.println("*** FREE FALL ***");
    caiguda=true;
    enable_inactivity();
  } 
  
  // Inactivity
  if(adxl.triggered(interrupts, ADXL345_INACTIVITY)){
    Serial.println("*** INACTIVITY ***");
    inactivitat=true;
  }
  
  /*  
  // Activity
  if(adxl.triggered(interrupts, ADXL345_ACTIVITY)){
  Serial.println("*** ACTIVITY ***"); 
  }
  */
  
  // Tap Detection
  if(adxl.triggered(interrupts, ADXL345_SINGLE_TAP)){
    Serial.println("*** TAP ***");
    cop=true;
    enable_inactivity();
  } 
}

void enable_inactivity(){
  // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setInactivityY(1); 
  adxl.setInactivityZ(1);
}

//--------------------------------------------------------------------------------------//
ISR(TIMER0_COMPA_vect){
  PORTD |= (1<<PD4);
  int8_t input=read8_ADC()-CONTINUA;  //pot donar problemes amb continua diferent de 127 -> solucio int16_t
  //  int16_t input=(int16_t)read8_ADC()-CONTINUA; //alternativa
  start_ADC();
  //printf("%d ", input);
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

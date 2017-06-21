#include <ADXL345.h>

#include <Wire.h>


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
  adxl.setActivityThreshold(75);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)
 
  adxl.setInactivityX(0);  // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setInactivityY(0); 
  adxl.setInactivityZ(0);
  adxl.setInactivityThreshold(75);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  adxl.setTimeInactivity(10);         // How many seconds of no activity is inactive?

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

}

void loop(){
  
  // Accelerometer Readings
  //int x,y,z;
  //adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store them in variables declared above x,y,z

  // Output Results to Serial
  /* UNCOMMENT TO VIEW X Y Z ACCELEROMETER VALUES * 
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(z); */

  if(caiguda | cop){
     if(caiguda & cop){
        //PROTOCOL ACCIDENT
        Serial.println("ACCIDENT!");
        emergency_reset();
     }
     else{
       emergency_counter+=1;
       if(inactivitat){
         //PROTOCOL ACCIDENT
         Serial.println("PROTOCOL");
         emergency_reset();
       }
       else{
         if(emergency_counter==3200){
           Serial.println("R");
           emergency_reset();
         }
       }
     }
  }
  
  ADXL_ISR();
  // You may also choose to avoid using interrupts and simply run the functions within ADXL_ISR(); 
  //  and place it within the loop instead.  
  // This may come in handy when it doesn't matter when the action occurs. 

}

/********************* ISR *********************/
/* Look for Interrupts and Triggered Action    */
void ADXL_ISR() {
  
  // getInterruptSource clears all triggered actions after returning value
  // Do not call again until you need to recheck for triggered actions
  byte interrupts = adxl.getInterruptSource();

  // Free Fall Detection
  if(adxl.triggered(interrupts, ADXL345_FREE_FALL)){
    Serial.println("*** FREE FALL ***");
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

void emergency_reset(){
 caiguda=false;
 cop=false;
 inactivitat=false;
 emergency_counter=0;
 adxl.setInactivityX(0);  // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
 adxl.setInactivityY(0); 
 adxl.setInactivityZ(0);
}

void enable_inactivity(){
 adxl.setInactivityX(1);  // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
 adxl.setInactivityY(1); 
 adxl.setInactivityZ(1);
}


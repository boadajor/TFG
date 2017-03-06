/*to operate with In-Circuit radino nRF8001
 * original file: radino_accel_i2c.ino by Albert Babí
 * that is a modification of another one
 * radino_nRF8001_IO_HF_USB_Test.ino
 * intended to IO-test, USB-test and HF-Test
 * for more information: www.in-circuit.de or www.radino.cc
 *  
 * starting January 20th 2016
*/
 
#include <SPI.h>//included by hal_aci_tl.cpp used by lib_aci.h
/*#include <lib_aci.h>//similar to SPI but 5 lines (MISO, MOSI, SCK, REQN(SS) and RDYN). to communicate with nrf8001
                    //USES DELAY(100) in lib_aci_board_init used by lib_aci_init
                    //a delay doesn't block an ISR

#include <aci_setup.h>*/
#include "uart_over_ble.h"

#include "Wire.h"//i2c, included by ADXL345.h
#include "ADXL345.h"
#include <TimerOne.h>//blink

#define adxl345_range_g 2//10bits (range=2g),11bits (range=4g),12bits (range=8g),13bits (range=16g),
#define adxl345_left_just 1
#define adxl345_full_res 1
#define pin_int2 0//RX-INT2; ADXL345 pin INT1 connected
#define pin_int3 1//TX-INT3; ADXL345 pin INT1 connected


/**
Put the nRF8001 setup in the RAM of the nRF8001.
*/
#include "services.h"//it is the same that services_lock.h but without protection
/**
Include the services_lock.h to put the setup in the OTP memory of the nRF8001.
This would mean that the setup cannot be changed once put in.
However this removes the need to do the setup of the nRF8001 on every reset.
*/

#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
    static services_pipe_type_mapping_t
        services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
    #define NUMBER_OF_PIPES 0
    static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
#endif

// Store the setup for the nRF8001 in the flash of the AVR to save on RAM 
static const hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;

static struct aci_state_t aci_state;

//Temporary buffers for sending ACI commands
static hal_aci_evt_t  aci_data;
//static hal_aci_data_t aci_cmd;

//Timing change state variable
static bool timing_change_done          = false;

//Used to test the UART TX characteristic notification
static uart_over_ble_t uart_over_ble;
static uint8_t         uart_buffer[20];
static uint8_t         uart_buffer_len = 0;
static uint8_t         dummychar = 0;

static bool is_connected = false;
static bool pipe_available = false;
static bool interval_changed = false;

int16_t XValue, YValue, ZValue;
static int counter = 0;
static bool deviceReady = false;
ADXL345 Accel_1; //class that defines all the functions to R/W to the ADXL345

//Initialize the radio_ack. This is the ack received for every transmitted packet.
//static bool radio_ack_pending = false;
// Define how assert should function in the BLE library 
void __ble_assert(const char *file, uint16_t line)
{
  Serial.print("ERROR ");
  Serial.print(file);
  Serial.print(": ");
  Serial.print(line);
  Serial.print("\n");
  while(1);
}

/*
Description:
In this template we are using the BTLE as a UART and can send and receive packets.
The maximum size of a packet is 20 bytes.
When a command it received a response(s) are transmitted back.
Since the response is done using a Notification the peer must have opened it(subscribed to it) before any packet is transmitted.
The pipe for the UART_TX becomes available once the peer opens it.
See section 20.4.1 -> Opening a Transmit pipe
In the master control panel, clicking Enable Services will open all the pipes on the nRF8001.
The ACI Evt Data Credit provides the radio level ack of a transmitted packet.
*/
void setup(void)
{
  Serial.begin(115200);//it works at 9600, 115200 no matter the value I put here!
  delay(1000);  //5 seconds delay for enabling to see the start up comments on the serial board
  Serial.println(F("Arduino setup"));
  Serial.println(F("Set line ending to newline to send data from the serial monitor"));

  /*
  //Point ACI data structures to the the setup data that the nRFgo studio generated for the nRF8001
  if (NULL != services_pipe_type_mapping)
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
  }
  else
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
  }
  aci_state.aci_setup_info.number_of_pipes    = NUMBER_OF_PIPES;
  aci_state.aci_setup_info.setup_msgs         = setup_msgs;
  aci_state.aci_setup_info.num_setup_msgs     = NB_SETUP_MESSAGES;

  
  //Tell the ACI library, the MCU to nRF8001 pin connections.
  //The Active pin is optional and can be marked UNUSED
  aci_state.aci_pins.board_name = BOARD_DEFAULT; //See board.h for details REDBEARLAB_SHIELD_V1_1 or BOARD_DEFAULT
  aci_state.aci_pins.reqn_pin   = 9; //SS for Nordic board, 9 for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.rdyn_pin   = 8; //3 for Nordic board, 8 for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.mosi_pin   = MOSI;
  aci_state.aci_pins.miso_pin   = MISO;
  aci_state.aci_pins.sck_pin    = SCK;

  aci_state.aci_pins.spi_clock_divider      = SPI_CLOCK_DIV8;//SPI_CLOCK_DIV8  = 2MHz SPI speed
                                                             //SPI_CLOCK_DIV16 = 1MHz SPI speed
  
  aci_state.aci_pins.reset_pin              = 4; //4 for Nordic board, UNUSED for REDBEARLAB_SHIELD_V1_1
  aci_state.aci_pins.active_pin             = UNUSED;
  aci_state.aci_pins.optional_chip_sel_pin  = UNUSED;

  aci_state.aci_pins.interface_is_interrupt = false; //Interrupts still not available in Chipkit
  aci_state.aci_pins.interrupt_number       = 1;

  //We reset the nRF8001 here by toggling the RESET line connected to the nRF8001
  //If the RESET line is not available we call the ACI Radio Reset to soft reset the nRF8001
  //then we initialize the data structures required to setup the nRF8001
  //The second parameter is for turning debug printing on for the ACI Commands and Events so they be printed on the Serial
  lib_aci_init(&aci_state, false);//IT USES delay(100)
  */
  
  //I2C configuration
  delay(1);
  Wire.begin();
  delay(1);
  Wire.setClock(400000);//i2c at 400kHz
  delay(1);
  
  //ADXL345 configuration
  Accel_1.init(ADXL345_ADDR_ALT_LOW);//it write a private variable (the address)
                                     //it calls powerON() that sets bit D3 (actually it clears all bits except D3) 
                                     //of register 0x2d (measure mode) 
                                     //it is recommended first configure and then enable measurement mode 
                                     //but the library does write the address and powerOn in the same function!
  Accel_1.writeTo(ADXL345_POWER_CTL,0);//it undoes powerON()
  
  Accel_1.setRangeSetting(adxl345_range_g);//+- 2/4/8/16 g
  Accel_1.setJustifyBit(adxl345_left_just);//left-rigth justified 1/0)
  Accel_1.setFullResBit(adxl345_full_res);//10bits (range=2g),11bits (range=4g),12bits (range=8g),13bits (range=16g),
  Accel_1.set_bw(ADXL345_BW_400);//always in normal mode (more power, less noise) sampling_freq=BW*2=400Hz
  //TAP ONE
  Accel_1.writeTo(0x2e,0);//disables all INT
//  Accel_1.writeTo(0x2f,0xBF);//TAP ONE to INT1
  Accel_1.writeTo(0x2f,0xFB);//TAP ONE to INT1
 // Accel_1.writeTo(0x21,0x30);//625usxLSB max duration of TAP ONE
//  Accel_1.writeTo(0x1D,0x20);//62.5 mg/LSB (that is, 0xFF = 16 g).
  Accel_1.writeTo(0x22,0);//disables double tap
  Accel_1.writeTo(0x23,0);//disables double tap  

  Accel_1.writeTo(0x28,0x06);//THRESH_FF 62.5 mg/LSB 
  Accel_1.writeTo(0x29,0x30);//TIME_FF 5 ms/LSB


//  Accel_1.writeTo(0x2a,0x07);//enables all axes in single tap; disables double tap
//  Accel_1.writeTo(0x2e,0x40);//enables SINGLE_TAP INT
  Accel_1.writeTo(0x2e,0x04);//enables FREE_FALL
  
  Accel_1.writeTo(ADXL345_POWER_CTL, 8);//equivalent to powerON()
  delay(1000);

  //IO configuration
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);

  pinMode(pin_int2,INPUT);
  pinMode(pin_int3,INPUT);

  //ISR configuration
  attachInterrupt(3,int3Isr, RISING);    
  
  Timer1.initialize(2500); // set a timer of length microseconds (put a number, NOT an operation)

  Timer1.attachInterrupt(timerIsr); // attach the service routine here
  }
  
void uart_over_ble_init(void)
{
    uart_over_ble.uart_rts_local = true;
}

bool uart_process_control_point_rx(uint8_t *byte, uint8_t length)
{
  bool status = false;
  aci_ll_conn_params_t *conn_params;

  if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_TX) )
  {
    Serial.println(*byte, HEX);
    switch(*byte)
    {
      /*
      Queues a ACI Disconnect to the nRF8001 when this packet is received.
      May cause some of the UART packets being sent to be dropped
      */
      case UART_OVER_BLE_DISCONNECT:
        /*
        Parameters:
        None
        */
        lib_aci_disconnect(&aci_state, ACI_REASON_TERMINATE);
        is_connected = false;
        status = true;
        break;

      /*
      Queues an ACI Change Timing to the nRF8001
      */
      case UART_OVER_BLE_LINK_TIMING_REQ:
        /*
        Parameters:
        Connection interval min: 2 bytes
        Connection interval max: 2 bytes
        Slave latency:           2 bytes
        Timeout:                 2 bytes
        Same format as Peripheral Preferred Connection Parameters (See nRFgo studio -> nRF8001 Configuration -> GAP Settings
        Refer to the ACI Change Timing Request in the nRF8001 Product Specifications
        */
        conn_params = (aci_ll_conn_params_t *)(byte+1);
        lib_aci_change_timing( conn_params->min_conn_interval,
                                conn_params->max_conn_interval,
                                conn_params->slave_latency,
                                conn_params->timeout_mult);
        status = true;
        break;

      /*
      Clears the RTS of the UART over BLE
      */
      case UART_OVER_BLE_TRANSMIT_STOP:
        /*
        Parameters:
        None
        */
        uart_over_ble.uart_rts_local = false;
        status = true;
        break;

      /*
      Set the RTS of the UART over BLE
      */
      case UART_OVER_BLE_TRANSMIT_OK:
        /*
        Parameters:
        None
        */
        uart_over_ble.uart_rts_local = true;
        status = true;
        break;
    }
  }
  return status;
}

void aci_loop()
{
  static bool setup_required = false;

  // We enter the if statement only when there is a ACI event available to be processed
  if (lib_aci_event_get(&aci_state, &aci_data))
  {
    aci_evt_t * aci_evt;
    aci_evt = &aci_data.evt;

    switch(aci_evt->evt_opcode)
    {
      /**
      As soon as you reset the nRF8001 you will get an ACI Device Started Event
      */
      case ACI_EVT_DEVICE_STARTED:
      {
        aci_state.data_credit_total = aci_evt->params.device_started.credit_available;
        switch(aci_evt->params.device_started.device_mode)
        {
          case ACI_DEVICE_SETUP:
            /**
            When the device is in the setup mode
            */
            Serial.println(F("Evt Device Started: Setup"));
            setup_required = true;
            break;

          case ACI_DEVICE_STANDBY:
            Serial.println(F("Evt Device Started: Standby"));
            //Looking for an iPhone by sending radio advertisements
            //When an iPhone connects to us we will get an ACI_EVT_CONNECTED event from the nRF8001
            if (aci_evt->params.device_started.hw_error)
            {
              delay(20); //Magic number used to make sure the HW error event is handled correctly.
            }
            else
            {
            lib_aci_connect(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
            Serial.println(F("Advertising started"));
            }
            break;
        }
      }
        break; //ACI Device Started Event

      case ACI_EVT_CMD_RSP:
        //If an ACI command response event comes with an error -> stop
        if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status)
        {
          //ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
          //TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
          //all other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command
          Serial.print(F("ACI Command "));
          Serial.println(aci_evt->params.cmd_rsp.cmd_opcode, HEX);
          Serial.print(F("Evt Cmd respone: Status "));
          Serial.println(aci_evt->params.cmd_rsp.cmd_status, HEX);
        }
        if (ACI_CMD_GET_DEVICE_VERSION == aci_evt->params.cmd_rsp.cmd_opcode)
        {
          //Store the version and configuration information of the nRF8001 in the Hardware Revision String Characteristic
          lib_aci_set_local_data(&aci_state, PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET,
            (uint8_t *)&(aci_evt->params.cmd_rsp.params.get_device_version), sizeof(aci_evt_cmd_rsp_params_get_device_version_t));
        }
        break;

      case ACI_EVT_CONNECTED:
        Serial.println(F("Evt Connected"));
        uart_over_ble_init();
        timing_change_done              = false;
        aci_state.data_credit_available = aci_state.data_credit_total;
        is_connected = true;
        /*
        Get the device version of the nRF8001 and store it in the Hardware Revision String
        */
        lib_aci_device_version();
        break;

      case ACI_EVT_PIPE_STATUS:
        Serial.println(F("Evt Pipe Status"));
        if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) && (false == timing_change_done))
        {
          pipe_available = true;
          lib_aci_change_timing_GAP_PPCP(); // change the timing on the link as specified in the nRFgo studio -> nRF8001 conf. -> GAP.
                                            // Used to increase or decrease bandwidth
          timing_change_done = true;
        }
        break;

      case ACI_EVT_TIMING:
        interval_changed = true;
        Serial.println(F("Evt link connection interval changed"));
        lib_aci_set_local_data(&aci_state,
                                PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET,
                                (uint8_t *)&(aci_evt->params.timing.conn_rf_interval), /* Byte aligned */
                                PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET_MAX_SIZE);
        break;

      case ACI_EVT_DISCONNECTED:
        Serial.println(F("Evt Disconnected/Advertising timed out"));
        lib_aci_connect(180/* in seconds */, 0x0100 /* advertising interval 100ms*/);
        Serial.println(F("Advertising started"));
        is_connected = false;
        interval_changed = false;
        pipe_available = false;
        break;

      case ACI_EVT_DATA_RECEIVED:
        Serial.print(F("Pipe Number: "));
        Serial.println(aci_evt->params.data_received.rx_data.pipe_number, DEC);
        if (PIPE_UART_OVER_BTLE_UART_RX_RX == aci_evt->params.data_received.rx_data.pipe_number)
        {
          Serial.print(F(" Data(Hex) : "));
          for(int i=0; i<aci_evt->len - 2; i++)
          {
            Serial.print((char)aci_evt->params.data_received.rx_data.aci_data[i]);
            uart_buffer[i] = aci_evt->params.data_received.rx_data.aci_data[i];
            Serial.print(F(" "));
          }

          uart_buffer_len = aci_evt->len - 2;
          Serial.println(F(""));
          if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX))
          {
          }
        }
        if (PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_RX == aci_evt->params.data_received.rx_data.pipe_number)
        {
            uart_process_control_point_rx(&aci_evt->params.data_received.rx_data.aci_data[0], aci_evt->len - 2); //Subtract for Opcode and Pipe number
        }
        break;

      case ACI_EVT_DATA_CREDIT:
        aci_state.data_credit_available = aci_state.data_credit_available + aci_evt->params.data_credit.credit;
        break;

      case ACI_EVT_PIPE_ERROR:
        //See the appendix in the nRF8001 Product Specication for details on the error codes
        Serial.print(F("ACI Evt Pipe Error: Pipe #:"));
        Serial.print(aci_evt->params.pipe_error.pipe_number, DEC);
        Serial.print(F("  Pipe Error Code: 0x"));
        Serial.println(aci_evt->params.pipe_error.error_code, HEX);

        //Increment the credit available as the data packet was not sent.
        //The pipe error also represents the Attribute protocol Error Response sent from the peer and that should not be counted
        //for the credit.
        if (ACI_STATUS_ERROR_PEER_ATT_ERROR != aci_evt->params.pipe_error.error_code)
        {
          aci_state.data_credit_available++;
        }
        break;

      case ACI_EVT_HW_ERROR:
        Serial.print(F("HW error: "));
        Serial.println(aci_evt->params.hw_error.line_num, DEC);

        for(uint8_t counter = 0; counter <= (aci_evt->len - 3); counter++)
        {
          Serial.write(aci_evt->params.hw_error.file_name[counter]); //uint8_t file_name[20];
        }
        Serial.println();
        lib_aci_connect(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
        Serial.println(F("Advertising started"));
        break;

    }
  }
  else
  {
    //Serial.println(F("No ACI Events available"));
    // No event in the ACI Event queue and if there is no event in the ACI command queue the arduino can go to sleep
    // Arduino can go to sleep now
    // Wakeup from sleep from the RDYN line
  }

  /* setup_required is set to true when the device starts up and enters setup mode.
   * It indicates that do_aci_setup() should be called. The flag should be cleared if
   * do_aci_setup() returns ACI_STATUS_TRANSACTION_COMPLETE.
   */
  if(setup_required)
  {
    if (SETUP_SUCCESS == do_aci_setup(&aci_state))
    {
      setup_required = false;
    }
  }
}

volatile bool ISR_TMR1_flag=false;
volatile unsigned long time2,time1=0;
//my own ISR
void timerIsr(){ISR_TMR1_flag=true;}
void int3Isr(){digitalWrite(10,!digitalRead(10));}

//variables
int acc_data_1[3];
//int8_t buff[6];//changed from uint8_t
int8_t buff2[3];//I will use one byte per axis
uint8_t sr=8;//in rigth justified mode divide the lecture by 2^sr
             //in left justified mode must be sr>=8, divide by 2^(sr-8) 
//threshold to turn on a LED at half the range of axis X
uint8_t g=round(256/pow(2,sr-7+log(adxl345_range_g)/log(2)));
uint8_t llindar1=g/2;


void loop() {

  if (digitalRead(pin_int2)==HIGH){
    digitalWrite(12, HIGH);
    //delay(100);
    Accel_1.getInterruptSource();
  }
  else{
    digitalWrite(12, LOW);
  }
  
  if (ISR_TMR1_flag){//(is_connected && pipe_available && interval_changed && deviceReady){

    time2=micros();
    ISR_TMR1_flag=false;
    
    Accel_1.readAccel(acc_data_1);
    
    if(Accel_1.status){
    
      buff2[0] = acc_data_1[0] >> sr;
      buff2[1] = acc_data_1[1] >> sr;
      buff2[2] = acc_data_1[2] >> sr;
  
      Serial.print(time2-time1);time1=time2;
      Serial.print(",");
      Serial.print(g,DEC);//256 i g before sifting sr bts to the rigth
      Serial.print(",");
      Serial.print(buff2[0],DEC);
      Serial.print(",");
      Serial.print(buff2[1],DEC);
      Serial.print(",");   
      Serial.println(buff2[2],DEC);
    
      if(abs(buff2[2])>llindar1){//blink
      digitalWrite(11, HIGH);
      digitalWrite(13, HIGH);      
      }
      else{   
      digitalWrite(11, LOW);    
      digitalWrite(13, LOW);            
      }
  
      //lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t *)buff2, 3);//this one
    }
    //else{Serial.println(",");}
  }
        
}

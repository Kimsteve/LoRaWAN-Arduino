
#include <lmic.h>
#include <hal/hal.h>
#include "DFRobot_EC.h"
#include "DFRobot_PH.h"
#include <EEPROM.h>

#define PH_PIN A1
#define EC_PIN A2
float voltageEC,voltagePH, phValue, ecValue,temperature = 25;
DFRobot_PH ph;
DFRobot_EC ec;
struct sensorValues {
 float ec;
 float ph;
};

// EUI little-endian format
static const u1_t PROGMEM APPEUI[8] = { };
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// little endian format
static const u1_t PROGMEM DEVEUI[8] = {  };
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

static const u1_t PROGMEM APPKEY[16] = {};

void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}


static osjob_t sendjob;

// Schedule TX every this many 
//seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN, 
  .rst = 9,  
  .dio = {2, 6, 7},
};

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      

      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Scheduling next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j) {

  struct sensorValues ss = ecread();
﻿ 
  float structec = ss.ec;
  float structph = ss.ph;  
  byte payload1[4];
  uint32_t ecValue = structec*100;
  uint32_t phValue = structph*100;
  payload1[0] =highByte(ecValue);
  payload1[1] =lowByte(ecValue);
  payload1[2] =highByte(phValue);
  payload1[3] =lowByte(phValue);
  

  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
     LMIC_setTxData2(1, payload1, sizeof(payload1), 0); //paload1
    
    Serial.println(F("Packet queued"));
    Serial.println(LMIC.freq);
  }
  //Next transmission after TX_COMPLETE
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting"));
  ph.begin();
  ec.begin();
  
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  LMIC_disableChannel(1);
  LMIC_disableChannel(2);
  printotaainformation();

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}
//print OTAA info
void printotaainformation(void)
{
  unsigned char i;
  unsigned char chartemp;
  unsigned char messagelength;

  Serial.println(F("OTAA mode to join network"));
  Serial.print("DevEui: ");
  for (i = 0; i <= 7; i++)
  {
    chartemp = pgm_read_word_near(DEVEUI+7-i);
    covertandprint((chartemp >> 4) & 0xf);
    covertandprint(chartemp & 0xf);    
  }
  Serial.println("");
  Serial.print("AppEui: ");
  for (i = 0; i <=7; i++)
  {
    chartemp = pgm_read_word_near(APPEUI+7-i);
    covertandprint((chartemp >> 4) & 0xf);
    covertandprint(chartemp & 0xf);    
  }

  Serial.println("");
  Serial.print("AppKey: ");
  //memcpy_P(buftemp, APPKEY, 16);
  for (i = 0; i <= 15; i++)
  {
    chartemp = pgm_read_word_near(APPKEY+i);
    //Serial.print(buftemp[i],HEX); 
    covertandprint((chartemp >> 4) & 0xf);
    covertandprint(chartemp & 0xf);
  }
  Serial.println("");
 
}

void covertandprint(unsigned char value)
{
  switch (value)
  {
    case 0  : Serial.print("0"); break;
    case 1  : Serial.print("1"); break;
    case 2  : Serial.print("2"); break;
    case 3  : Serial.print("3"); break;
    case 4  : Serial.print("4"); break;
    case 5  : Serial.print("5"); break;
    case 6  : Serial.print("6"); break;
    case 7  : Serial.print("7"); break;
    case 8  : Serial.print("8"); break;
    case 9  : Serial.print("9"); break;
    case 10  : Serial.print("A"); break;
    case 11  : Serial.print("B"); break;
    case 12  : Serial.print("C"); break;
    case 13  : Serial.print("D"); break;
    case 14  : Serial.print("E"); break;
    case 15 :  Serial.print("F"); break;
    default :
      Serial.print("?");   break;
  }
}

//Read sensor Values
struct sensorValues ecread(){

  float ecValueRead;
  struct sensorValues val;
   char cmd[10];
   static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U)  //time interval: 1s
    {
      timepoint = millis();
      // read the ph voltage
       voltagePH = analogRead(PH_PIN)/1024.0*5000; 
       // convert voltage to pH with temperature compensation         
       phValue    = ph.readPH(voltagePH,temperature);       
      
      // read the voltage
      voltageEC = analogRead(EC_PIN)/1024.0*5000;  
      // convert voltage to EC with temperature compensation
      ecValueRead =  ec.readEC(voltageEC,temperature);  

      //val.ec = ecValueRead;
      //val.ph = phValue;
      val.ec = 1.222;
      val.ph = 4.555;
      Serial.println("transmitting now");
      delay(60*60*1000);
      
      Serial.println("transmitting after delay");
      return val;
     
      //return ecValueRead;
    }

    if(readSerial(cmd)){
       strupr(cmd);
       if(strstr(cmd,"PH")){
        //PH calibration process by Serail CMD
         ph.calibration(voltagePH,temperature,cmd);       
        }
        if(strstr(cmd,"EC")){
          //EC calibration process by Serail CMD
         ec.calibration(voltageEC,temperature,cmd);       
        }
    }
    
}

int i = 0;
bool readSerial(char result[]){
    while(Serial.available() > 0){
        char inChar = Serial.read();
        if(inChar == '\n'){
             result[i] = '\0';
             Serial.flush();
             i=0;
             return true;
        }
        if(inChar != '\r'){
             result[i] = inChar;
             i++;
        }
        delay(1);
    }
    return false;
}


void loop() {
  os_runloop_once();
   
}

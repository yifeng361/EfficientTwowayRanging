/*
 * Copyright (c) 2020 by Ashutosh Dhekne <dhekne@gatech.edu>
 * Peer-peer protocol
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file PeerProtocol_test01.ino
 * 
 *  
 */

#include <SPI.h>
#include <math.h>
//#include <DW1000.h>
#include "genericFunctions.h"
#include "RangingContainer.h"
#include "Adafruit_LSM9DS1.h"
#include <SdFat.h>
#include <time.h>
#include<TimeLib.h>
#include "RTClib.h"
#include<Wire.h>

// PIN Macro
#define VBATPIN A2
#define LED_PIN 12
#define NOISE_PIN 13
#define GOOD_PIN 6
#define SILENCE_PIN 5
#define DEV_INDICATOR_PIN 13

//Timer for implementing timeouts
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

//Other Macros
#define INIT_RTC_ALWAYS 0
#define USB_CONNECTION 0
#define DEBUG_PRINT 0
#define OUR_UWB_FEATHER 1
#define AUS_UWB_FEATHER 0


#if(OUR_UWB_FEATHER==1)
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 17; // irq pin
const uint8_t PIN_SS = 19; // spi select pin
#endif
/*
#if(AUS_UWB_FEATHER==1)
const uint8_t PIN_RST = 2; // reset pin
const uint8_t PIN_IRQ = 3; // irq pin
const uint8_t PIN_SS = 4; // spi select pin
#endif
*/


// Global control variables
volatile boolean received = false;
volatile boolean error = false;
volatile int16_t numReceived = 0; // todo check int type
volatile boolean sendComplete = false;
volatile boolean RxTimeout = false;

byte tx_poll_msg[MAX_POLL_LEN] = {POLL_MSG_TYPE,0};
byte tx_resp_msg[MAX_RESP_LEN] = {RESP_MSG_TYPE, 0x02, 0, 0, 0, 0, 0};
byte tx_final_msg[MAX_FINAL_LEN] = {FINAL_MSG_TYPE, 0};
byte tx_ack_msg[MAX_FINAL_LEN] = {FINAL_MSG_TYPE, 0};


typedef enum states{STATE_IDLE, STATE_POLL, STATE_POLL_EXPECT, STATE_EXPECT, STATE_RESP_EXPECT, STATE_FINAL_SEND, STATE_TWR_DONE, STATE_RESP_SEND, STATE_FINAL_EXPECT, STATE_ACK_SEND, STATE_ACK_EXPECT} STATES;
volatile uint8_t current_state = STATE_IDLE;
unsigned long silenced_at =0;
long randNumber;
int currentSlots = 8;






// SD-card related variables
SdFat sd;
SdFile store_distance; //The root file system useful for deleting all files on the SDCard
char filename[14];
int filenum = 0;
int entries_in_file=0;
int SDChipSelect = 10;
int SDEnabled=0;

//Variables storing information
const int recv_id_table[TRX_NUM][TRX_NUM]= {
  {1,1,1,1,1,1},
  {1,1,1,1,1,1},
  {1,1,1,1,1,1}, 
  {1,1,1,1,1,1},
  {1,1,1,1,1,1},
  {1,1,1,1,1,1}
};

byte rx_packet[128];
uint8_t myAcc[1000];
Ranging thisRange;
String message;
int sendDelay;
uint64_t respRxTs[TRX_NUM+1]; //first element is reserved 
int distMat[TRX_NUM+1][TRX_NUM+1] = {0};
uint16_t distTsMat[TRX_NUM+1][TRX_NUM+1] = {0};
int respRxIds[TRX_NUM] = {0,0,0,0,0,0};
double nextPollTime;
double eventStartTime; // in ms
double currentTime;
double elapsedTime;
double pollExpectStartTime;
double pollExpectElapsedTime;
double pollExpectRemainTime;

double respExpectStartTime;
double respExpectElapsedTime;
double finalExpectStartTime;
double finalExpectElapsedTime;
double ackExpectStartTime;
double ackExpectElapsedTime;

int pollExpectTuneSlot = 0;
int finalExpectTimeout = 0;

boolean skip_receive = 1;

int currentDeviceIndex = 0;
int respCount = 0;
int ackCount = 0;
int currentInitiator = 0;
uint16_t initiatorSeq = 0; // When I am an initiator, which seq # to put in the packet
uint16_t currentSeq = 0; // When I am a responder, which seq do I receive from the initiator

uint16_t rxTimeoutUs = TYPICAL_TIMEOUT;
//Time
RTC_PCF8523 rtc;


void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);
void TC3_Handler();

typedef struct DeviceRespTs {
  int deviceID;
  uint64_t respRxTime;
};


void receiver(uint16_t rxtoval=0 ) {
  RxTimeout = false;
  received = false;
  DW1000.newReceive();
  DW1000.setDefaults();
  // we cannot don't need to restart the receiver manually
  DW1000.receivePermanently(false);
  if (rxtoval>0) {
    DW1000.setRxTimeout(rxtoval);
  } else {
    //Serial.print("Resetting Timeout to  ");
    //Serial.println(rxtoval);
    DW1000.setRxTimeout(rxtoval);
  }
  DW1000.startReceive();
  //Serial.println("Started Receiver");
}


void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(NOISE_PIN, OUTPUT);
  pinMode(GOOD_PIN, OUTPUT);
  pinMode(DEV_INDICATOR_PIN, OUTPUT);
  pinMode(SILENCE_PIN, INPUT_PULLUP);
  digitalWrite(GOOD_PIN, HIGH);
  analogReadResolution(10);
  // DEBUG monitoring
  Serial.begin(115200);
  while(!Serial)
  {
    delay(10);
    #if(USB_CONNECTION==0)
      break;
    #endif
  }
Serial.print("Waiting...");
delay(5000);
Serial.print("Should see this...");
  //Setting up the RTC Clock
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    //while (1);
  }
  
  if (! rtc.initialized()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println("Setting new time");
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
     rtc.adjust(DateTime(2020, 10, 17, 19, 40, 0));
  }

//In production, INIT_RTC_ALWAYS should be 0.
//Only turn this to 1 when testing
#if (INIT_RTC_ALWAYS == 1)
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
#endif
  //rtc.adjust(DateTime(2019, 11, 13, 10, 06, 00));
  //SoftRTC.begin(rtc.now());  // Initialize SoftRTC to the current time

//while(1) {
  Serial.println("Current Time");
  DateTime now = rtc.now();
  Serial.print(now.year());
  Serial.print("/");
  Serial.print(now.month());
  Serial.print("/");
  Serial.print(now.day());
  Serial.print(" ");
  Serial.print(now.hour());
  Serial.print(":");
  Serial.print(now.minute());
  Serial.print(":");
  Serial.print(now.second());
  delay(1000);
//}
  Serial.print("Initializing SD card...");
  //delay(1000);
  if (!sd.begin(SDChipSelect, SPI_FULL_SPEED)) {
    Serial.println("SDCard Initialization failed!");
    SDEnabled = 0;
  } else {
    Serial.println("SDCard Initialization done.");
    SDEnabled = 1;
  }

  if (SDEnabled==1) {
    sprintf(filename, "dist%03d.txt", filenum);
    if (!store_distance.open(filename, O_WRITE|O_CREAT)) {
      Serial.println("Could not create file");
      delay(10000);
    }
  }
  randomSeed(analogRead(0));
  Serial.println(F("Peer-peer ranging protocol"));
  Serial.println("Free memory: ");
  Serial.println(freeMemory());
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  Serial.println(F("DW1000 initialized ..."));
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(6);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.commitConfiguration();
  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
  // attach callback for (successfully) received messages
  DW1000.attachReceivedHandler(handleReceived);
  DW1000.attachReceiveTimeoutHandler(handleRxTO);
  DW1000.attachReceiveFailedHandler(handleError);
  DW1000.attachErrorHandler(handleError);
  DW1000.attachSentHandler(handleSent);
  // start reception
  
  current_state = STATE_IDLE;

#if (INITIATOR == 1)
digitalWrite(DEV_INDICATOR_PIN, 1);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 0);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 1);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 0);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 1);
delay(5000);
digitalWrite(DEV_INDICATOR_PIN, 0);
//delay(500);
#else
digitalWrite(DEV_INDICATOR_PIN, 1);
delay(1000);
digitalWrite(DEV_INDICATOR_PIN, 0);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 1);
delay(200);
digitalWrite(DEV_INDICATOR_PIN, 0);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 1);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 0);
#endif

#if (INITIATOR==1)
  while (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    delay(1000);
    #if IGNORE_IMU==1
    disable_imu = 1;
    break;
    #endif
  }
  if (disable_imu==0) {
  Serial.println("Found LSM9DS1 9DOF");
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
  }
#endif
}


void handleSent() {
  // status change on sent success
  sendComplete = true;
  //Serial.println("Send complete");
}


void handleReceived() {
  // status change on reception success
  
  DW1000.getData(rx_packet, DW1000.getDataLength());
  //Serial.println("Received something...");
  received = true;
  //show_packet(rx_packet, DW1000.getDataLength());
}

void handleError() {
  error = true;
}

void handleRxTO() {
  RxTimeout = true;
  #if (DEBUG_PRINT==1)
  Serial.println("Rx Timeout");
  Serial.println("State: ");
  Serial.println(current_state);
  #endif
}


float gravity_f = 0.0f;



#define YIFENG_TEST 0
int test_flag = 0;

double debugPollStartTime = 0;
double debugPollExpectStartTime = 0;
double debugStartTime = 0;
double debugElapsedTime = 0;

void loop() {
  //digitalWrite(GOOD_PIN, HIGH);
  //Serial.println(analogRead(A2));
  //Serial.print("Pin Read: ");
  //Serial.println(digitalRead(SILENCE_PIN));

  // if(SDEnabled) {
  //   if (digitalRead(SILENCE_PIN)==0)
  //   {
  //     store_distance.println("Silenced");
  //     silenced_at = rtc.now().unixtime();
  //   }
  // }

  if(!skip_receive){
    receiver(rxTimeoutUs);
    while(!RxTimeout && !received){

    }
  }
  skip_receive = 0;
 

  switch(current_state) {
    case STATE_IDLE: {
        current_state = STATE_POLL_EXPECT;
        pollExpectRemainTime = (TRX_NUM - 1) * RANGING_TIMER;
        pollExpectStartTime = get_time_us();
        skip_receive = 1;
        break;
    }
    case STATE_POLL: {
      //Send POLL here
      initiatorSeq++;
      tx_poll_msg[SRC_IDX] = myid;
      tx_poll_msg[DST_IDX] = BROADCAST_ID;
      tx_poll_msg[SEQ_IDX] = initiatorSeq & 0xFF;
      tx_poll_msg[SEQ_IDX + 1] = initiatorSeq >> 8;

      //send
      DW1000.newTransmit();
      DW1000Time txTime;
      DW1000Time deltaTime = DW1000Time(FIXED_DELAY, DW1000Time::MICROSECONDS);
      txTime = DW1000.setDelay(deltaTime);
      uint64_t txTime64 = txTime.getTimestamp();
      any_msg_set_ts(&tx_poll_msg[POLL_TX_TS_IDX], txTime64);
      generic_send(tx_poll_msg, sizeof(tx_poll_msg));
#if(DEBUG_FLAG)
      Serial.print("Send  a poll, seq = ");
      Serial.print(initiatorSeq);
      Serial.print("poll tx ts = ");
      print_uint64(txTime64);
      Serial.println("");
#endif
      current_state = STATE_RESP_EXPECT;
      while(!sendComplete);
      respCount = 0;
      sendComplete = false;
      rxTimeoutUs = MAXTIME_RESP_EXPECT;
      respExpectStartTime = get_time_us();
      break;
    }
    case STATE_RESP_EXPECT: {
      currentTime = get_time_us();
      respExpectElapsedTime = get_elapsed_time_us(respExpectStartTime, currentTime);
      if(respExpectElapsedTime > MAXTIME_RESP_EXPECT){
        current_state = STATE_FINAL_SEND;
        skip_receive = 1;
      }else{
        if (received) {

          received = false;
          //Serial.print("received MSG type: ");
          //Serial.println(rx_packet[0]);
          show_packet(rx_packet, DW1000.getDataLength());
          uint16_t thisSeq = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX+1] << 8);
          
          if (rx_packet[0] == RESP_MSG_TYPE && (rx_packet[DST_IDX] == myid || rx_packet[DST_IDX] == BROADCAST_ID)
            && thisSeq == initiatorSeq) {
            //Serial.println("Recieved response!");
            currentDeviceIndex = rx_packet[SRC_IDX];
            DW1000Time rxTS;
            DW1000.getReceiveTimestamp(rxTS);
            respRxTs[currentDeviceIndex] = rxTS.getTimestamp();
            respRxIds[respCount] = currentDeviceIndex;
            respCount++;
#if(DEBUG_FLAG)
            Serial.print("receive a response, seq = ");
            Serial.print(thisSeq);
            Serial.print(", src = ");
            Serial.print(rx_packet[SRC_IDX]);
            Serial.print(" rx Ts: ");
            print_uint64(respRxTs[currentDeviceIndex]);
            Serial.println("");
#endif
            if(respCount == TRX_NUM - 1){
              current_state = STATE_FINAL_SEND;
              skip_receive = 1;
            }
          }else if(rx_packet[0] == POLL_MSG_TYPE){
#if(DEBUG_FLAG)

            Serial.print("Receive a poll while expecting a response, seq = ");
            Serial.println(thisSeq);
#endif
            
          }
        }
        if(current_state == STATE_RESP_EXPECT){
          //  currentTime = get_time_us();
          //  respExpectElapsedTime = get_elapsed_time_us(respExpectStartTime, currentTime);
          double tmpRxTimeout = MAXTIME_RESP_EXPECT - respExpectElapsedTime;
          rxTimeoutUs = getNonnegRxTimeout(tmpRxTimeout);
        }
      }
      break;
    }
    case STATE_FINAL_SEND: {
#if(DEBUG_FLAG)
      Serial.print("State: FINAL SEND, ");
      Serial.print("recvd packets: ");
      Serial.println(respCount);
#endif
      tx_final_msg[SRC_IDX] = myid;
      tx_final_msg[DST_IDX] = BROADCAST_ID;
      tx_final_msg[SEQ_IDX] = initiatorSeq & 0xFF;
      tx_final_msg[SEQ_IDX + 1] = initiatorSeq >> 8;
      tx_final_msg[FINAL_NUM_NODES_IDX] = respCount;
      int j;
      for(int i = 0; i < respCount; i++){
          j = FINAL_NUM_NODES_IDX + i * FINAL_ONE_RESP_LEN + 1;
          tx_final_msg[j] = respRxIds[i];
          any_msg_set_ts(&tx_final_msg[j+1], respRxTs[respRxIds[i]]);
      }

      //send
      DW1000.newTransmit();
      DW1000Time txTime;
      DW1000Time deltaTime = DW1000Time(FIXED_DELAY, DW1000Time::MICROSECONDS);
      txTime = DW1000.setDelay(deltaTime);
      uint64_t txTime64 = txTime.getTimestamp();
      any_msg_set_ts(&tx_final_msg[FINAL_TX_TS_IDX], txTime64);
      generic_send(tx_final_msg, sizeof(tx_final_msg));
#if(DEBUG_FLAG)
      Serial.print("send final: resp rx ts: ");
      print_uint64(respRxTs[respRxIds[0]]);
      Serial.print(", final tx ts: ");
      print_uint64(txTime64);
      Serial.println("");
#endif

      while(!sendComplete);
      sendComplete = false;

      current_state = STATE_POLL_EXPECT;
      rxTimeoutUs = TYPICAL_TIMEOUT;
      if(respCount > 0){
        pollExpectRemainTime = (TRX_NUM - 1) * RANGING_TIMER;
      }else{
        pollExpectRemainTime = (TRX_NUM - 1 + myid) * RANGING_TIMER;
      }
      pollExpectStartTime = get_time_us();
      break;
    }

    case STATE_POLL_EXPECT:{
      debugStartTime = get_time_us();
      currentTime = get_time_us();
      pollExpectElapsedTime = get_elapsed_time_us(pollExpectStartTime, currentTime); // Note here should be +=
      // Serial.print("Poll expect slapsed time: ");
      // Serial.println(pollExpectElapsedTime);

      if(pollExpectElapsedTime > pollExpectRemainTime){
        current_state = STATE_POLL;
        skip_receive = 1;
      }else{
        if (received) {
          received = false;
          //show_packet(rx_packet, DW1000.getDataLength());
          if (rx_packet[0] == POLL_MSG_TYPE && (rx_packet[DST_IDX] == myid || rx_packet[DST_IDX] == BROADCAST_ID)) {
            //Serial.println("Recieved poll!");
            currentInitiator = rx_packet[SRC_IDX];
            currentSeq = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX+1] << 8);


            uint64_t PollTxTime_64=0L;
            any_msg_get_ts(&rx_packet[POLL_TX_TS_IDX], &PollTxTime_64);
            thisRange.PollTxTime = DW1000Time((int64_t)PollTxTime_64);



            //Serial.println("Now the time thing");
            DW1000Time rxTS; 
            DW1000.getReceiveTimestamp(rxTS);
            thisRange.PollRxTime = rxTS;

            tx_resp_msg[DST_IDX] = rx_packet[SRC_IDX];
            tx_resp_msg[SRC_IDX] = myid;
            tx_resp_msg[SEQ_IDX] = currentSeq & 0xFF;
            tx_resp_msg[SEQ_IDX + 1] = currentSeq >> 8;
            sendDelay = TIME_SLOT_LEN * ((myid > rx_packet[SRC_IDX]) ? (myid - 1): myid);
            

            //send
            DW1000.newTransmit();
            DW1000Time txTime;
            DW1000Time deltaTime = DW1000Time(sendDelay, DW1000Time::MICROSECONDS);
            txTime = DW1000.setDelay(deltaTime);
            generic_send(tx_resp_msg, sizeof(tx_resp_msg));
#if(DEBUG_FLAG)
            Serial.print("receive a poll, seq = ");
            Serial.print(currentSeq);
            Serial.print(", src = ");
            Serial.print(rx_packet[SRC_IDX]);
            Serial.print(", send delay: ");
            Serial.print(sendDelay);
            Serial.print(", poll tx Ts: ");
            print_uint64((int64_t)PollTxTime_64);
            Serial.print(", poll rx Ts: ");
            print_uint64(rxTS.getTimestamp());
            Serial.println("");
#endif
            while(!sendComplete);
            sendComplete = false;
            DW1000Time txTS; 
            DW1000.getTransmitTimestamp(txTS);
            thisRange.RespTxTime = txTS;
            
            //Serial.println("Response sent");
            finalExpectTimeout =  MAXTIME_FINAL_EXPECT - sendDelay;
            rxTimeoutUs = finalExpectTimeout;
            finalExpectStartTime = get_time_us();
            current_state = STATE_FINAL_EXPECT;
            
            // int idle_delay = (MAXTIME_RESP_EXPECT - sendDelay) / 1e3 - 2;
            // idle_delay = (idle_delay > 0) ? idle_delay : 1;
            // delay(idle_delay);
          }
        }
        if(current_state == STATE_POLL_EXPECT){// No need to reduce remain time here
          double tmpRxTimeout = pollExpectRemainTime - pollExpectElapsedTime;
          rxTimeoutUs = getNonnegRxTimeout(tmpRxTimeout);
          // Serial.print("Remain Time (us): ");
          // Serial.print(pollExpectRemainTime);
          // Serial.print(", elapsed time (us): ");
          // Serial.print(pollExpectElapsedTime);
          // Serial.print(", STATE_POLL_EXPECT (us): ");
          // Serial.println((pollExpectRemainTime - pollExpectElapsedTime));
          // Serial.print("rxTimeout (us): ");
          // Serial.println(rxTimeoutUs);
          //rxTimeoutUs = 0;
        }
      }
      currentTime = get_time_us();
      break;
    }
    //*/

    case STATE_FINAL_EXPECT: {
      currentTime = get_time_us();
      finalExpectElapsedTime = get_elapsed_time_us(finalExpectStartTime, currentTime);
      if(finalExpectElapsedTime > finalExpectTimeout){
         current_state = STATE_POLL_EXPECT;
         pollExpectElapsedTime = get_elapsed_time_us(pollExpectStartTime, get_time_us());
         pollExpectRemainTime -= pollExpectElapsedTime;
         rxTimeoutUs = getNonnegRxTimeout(pollExpectRemainTime);
      }else{
        if(received) {
          received = false;
          uint16_t thisSeq = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX+1] << 8);
          if (rx_packet[0] == FINAL_MSG_TYPE && (rx_packet[DST_IDX]==myid || rx_packet[DST_IDX] == BROADCAST_ID)
            && thisSeq == currentSeq && rx_packet[SRC_IDX] == currentInitiator) {
            DW1000Time rxTS;
            DW1000.getReceiveTimestamp(rxTS);
            thisRange.FinalRxTime = rxTS;
            boolean rangeSuccuess = false;
            for(int i = 0; i < TRX_NUM; i++){
              if(rx_packet[FINAL_NUM_NODES_IDX + i*FINAL_ONE_RESP_LEN + 1] == myid){
                rangeSuccuess = true;
                uint64_t RespRxTime_64=0L;
                any_msg_get_ts(&rx_packet[FINAL_NUM_NODES_IDX + (i*FINAL_ONE_RESP_LEN)+2], &RespRxTime_64);
                thisRange.RespRxTime = DW1000Time((int64_t)RespRxTime_64);
                break;
              }
            }

            uint64_t FinalTxTime_64=0L;
            any_msg_get_ts(&rx_packet[FINAL_TX_TS_IDX], &FinalTxTime_64);     
            thisRange.FinalTxTime = DW1000Time((int64_t)FinalTxTime_64);
#if(DEBUG_FLAG)
            Serial.print("receive a final, seq = ");
            Serial.print(thisSeq);
            Serial.print(", src = ");
            Serial.print(rx_packet[SRC_IDX]);
            Serial.print(" poll tx Ts: ");
            print_uint64(thisRange.PollTxTime.getTimestamp());
            Serial.print(" poll rx Ts: ");
            print_uint64(thisRange.PollRxTime.getTimestamp());
            Serial.print(" resp tx Ts: ");
            print_uint64(thisRange.RespTxTime.getTimestamp());
            Serial.print(" resp rx Ts: ");
            print_uint64(thisRange.RespRxTime.getTimestamp());
            Serial.print(" final tx Ts: ");
            print_uint64(thisRange.FinalTxTime.getTimestamp());
            Serial.print(" final rx Ts: ");
            print_uint64(thisRange.FinalRxTime.getTimestamp());
            Serial.println("");
#endif
            // Update distMat
            int dist = thisRange.calculateRange();
            distMat[myid][rx_packet[SRC_IDX]] = dist;
            Serial.print(myid);
            Serial.print(", ");
            Serial.print(rx_packet[SRC_IDX]);
            Serial.print(", ");
            Serial.print(thisSeq);
            Serial.print(", ");
            Serial.print(dist);
            Serial.print("   || distMat: ");
            for(int i = 1; i <= TRX_NUM; i++){
              Serial.print(distMat[myid][i]);
              Serial.print(",");
            }
            Serial.println("");
            //current_state = STATE_POLL_EXPECT;
            
            int myorder = (myid - rx_packet[SRC_IDX] + TRX_NUM) % TRX_NUM - 1;
            if(myorder == 0){
              current_state = STATE_POLL;
              delay(5);
              skip_receive = 1;
#if(DEBUG_FLAG)
              Serial.println("I will poll next.");
#endif
            }else{
              pollExpectRemainTime = myorder * RANGING_TIMER;
              current_state = STATE_POLL_EXPECT;
              rxTimeoutUs = getNonnegRxTimeout(pollExpectRemainTime);
              pollExpectStartTime = get_time_us();
#if(DEBUG_FLAG)
            Serial.print("To poll, waiting time = ");
            Serial.println(pollExpectRemainTime);
#endif
            }
            //pollExpectRemainTime -= pollExpectElapsedTime;
            
            //Send ACK
            /*
            tx_ack_msg[0] = ACK_MSG_TYPE;
            tx_ack_msg[SRC_IDX] = myid;
            tx_ack_msg[DST_IDX] = rx_packet[SRC_IDX];
            tx_ack_msg[SEQ_IDX] = currentSeq & 0xFF;
            tx_ack_msg[SEQ_IDX + 1] = currentSeq >> 8;
            tx_ack_msg[ACK_DIST_NUM_IDX] = 6;
            int j;
            for(int i = 0; i < TRX_NUM; i++){
                j = ACK_DIST_NUM_IDX+1;
                tx_ack_msg[j] = i + 1;
                j += 1;
                short_msg_set(&tx_ack_msg[j], distMat[myid][i+1]);
                j += ACK_DIST_LEN;
                uint16_t currentTimeMs = get_time_ms_uint16();
                short_msg_set(&tx_ack_msg[j], currentTimeMs);
                break;
              }
            }
            sendDelay = TIME_SLOT_LEN * ((myid > recvid) ? (myid - 1): myid);
        
            DW1000Time txTime;
            DW1000Time deltaTime = DW1000Time(sendDelay, DW1000Time::MILLISECONDS);
            txTime = DW1000.setDelay(deltaTime);
            generic_send(tx_ack_msg, sizeof(tx_resp_msg));
            */
          }
        }
        if(current_state == STATE_FINAL_EXPECT){
          // currentTime = get_time_us();
          double tmpRxTimeout = finalExpectTimeout - finalExpectElapsedTime;
          rxTimeoutUs = getNonnegRxTimeout(tmpRxTimeout);
        }
      }

      break;
    }
    /*
    case STATE_ACK_EXPECT: {
      if (received) {
        received = false;
        //Serial.print("received MSG type: ");
        //Serial.println(rx_packet[0]);
        show_packet(rx_packet, DW1000.getDataLength());
        uint16_t thisSeq = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX+1] << 8);
        if (rx_packet[0] == ACK_MSG_TYPE && (rx_packet[DST_IDX] == myid || rx_packet[DST_IDX] == BROADCAST_ID) &&
        thisSeq == initiatorSeq) {
          //Serial.println("Recieved response!");
          currentDeviceIndex = rx_packet[SRC_IDX];
          int src = rx_packet[SRC_IDX]; 
          int distNum = rx_packet[ACK_DIST_NUM_IDX];
          uint16_t dist; uint16_t distTs; 
          for(int i = 0; i < distNum; i++){
              j = ACK_DIST_NUM_IDX + 1 + i * ACK_DIST_ENTRY_LEN;
              short_msg_get(&rx_packet[j+1], &dist);
              distMat[src][j] = dist;
              short_msg_get(&rx_packet[j+1+ACK_DIST_LEN], &distTs);
              distTsMat[src][j] = distTs;
          }
          distMat[myid][src] = distMat[src][myid];
          distTsMat[myid][src] = distTsMat[src][myid];

          ackCount++;

          currentTime = get_time_us();
          elapsedTime = get_elapsed_time_us(eventStartTime, currentTime);
          if(respCount == TRX_NUM - 1 || elapsedTime >= MAXTIME_ACK_EXPECT){
            current_state = STATE_POLL_EXPECT;
          }
        }
      }

      if(current_state == STATE_ACK_EXPECT){
          elapsedTime = get_elapsed_time_us(eventStartTime, currentTime);
          uint16_t tmpRxTimeout= MAXTIME_RESP_EXPECT - (uint16_t)(elapsedTime/1000) > 0 ? MAXTIME_RESP_EXPECT - (uint16_t)(elapsedTime/1000) : 1;
          receiver(tmpRxTimeout);
          eventStartTime = get_time_us();
      }

      break;
    }*/
    /* Dont delete, TBC*/
    
    //*/
    
    
  }
}





void show_packet(byte packet[], int num) {
  #if (DEBUG_PRINT==1)
  for (int i=0;i<num;i++) {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  #endif
  
}

//Timer Functions
//Timer functions.
void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  //Serial.println(TC->COUNT.reg);
  //Serial.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {
  
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;
  
  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  
    NVIC_SetPriority(TC3_IRQn, 3);
    NVIC_EnableIRQ(TC3_IRQn);  
    

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;

  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

/*
void collect_imu_data(byte *imu_buff, int *imu_buffer_counter) {
  if (disable_imu==0)
  {
  lsm.readBuffer(XGTYPE, 0x80 | lsm.LSM9DS1_REGISTER_OUT_X_L_XL, IMU_SINGLE_READING_BUFF_SIZE, &imu_buff[(*imu_buffer_counter)]);
  (*imu_buffer_counter) += IMU_SINGLE_READING_BUFF_SIZE; 
  
  
  
  lsm.readBuffer(MAGTYPE, 0x80 | lsm.LSM9DS1_REGISTER_OUT_X_L_M, IMU_SINGLE_READING_BUFF_SIZE, &imu_buff[(*imu_buffer_counter)]);
  (*imu_buffer_counter) += IMU_SINGLE_READING_BUFF_SIZE; 
  
  
  lsm.readBuffer(XGTYPE, 0x80 | lsm.LSM9DS1_REGISTER_OUT_X_L_G, IMU_SINGLE_READING_BUFF_SIZE, &imu_buff[(*imu_buffer_counter)]);
  (*imu_buffer_counter) += IMU_SINGLE_READING_BUFF_SIZE; 
  } else {
    (*imu_buffer_counter) += IMU_SINGLE_READING_BUFF_SIZE*3;
  }
}
*/


void TC3_Handler() 
{
  /*
  DW1000Time currectTS;
  uint64_t currentUWBTime;
  for(int i=0;i<MAX_TIMEOUTS;i++) {
    if (timeout_established[i]) {
      DW1000.getSystemTimestamp(currectTS);
      currentUWBTime = currectTS.getTimestamp();
      break; //Any timeout if established will populate the currentUWBTime
    }
  }
  for(int i=0;i<MAX_TIMEOUTS;i++) {
    if (timeout_established[i]) {
      if(currentUWBTime > timeout_time[i]) {
        timeout_established[i] = false;
        timeout_time[i] = INFINITE_TIME;
        timeout_overflow[i] = false;
        timeout_triggered[i] = true;
      } else if (timeout_overflow[i] == true && currentUWBTime > (timeout_time[i] - 2^40)) {
        timeout_established[i] = false;
        timeout_time[i] = INFINITE_TIME;
        timeout_overflow[i] = false;
        timeout_triggered[i] = true;
      }
    }
  }
  */
}
/*
void set_timeout(int whichTO, uint32_t delayTime) {
  DW1000Time currectTS;
  uint64_t currentUWBTime;
  DW1000.getSystemTimestamp(currectTS);
  currentUWBTime = currectTS.getTimestamp();
  DW1000Time deltaTime = DW1000Time(delayTime, DW1000Time::MILLISECONDS);
  timeout_time[whichTO] = (currectTS + deltaTime).getTimestamp();
  if (timeout_time[whichTO] > 2^40) {
    timeout_overflow[whichTO] = true;
  } else {
    timeout_overflow[whichTO] = false;
  }
}

*/

//Utility functions
void dateTime(uint16_t* date, uint16_t* time_) {
  DateTime now = rtc.now();
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time_ = FAT_TIME(now.hour(), now.minute(), now.second());
  printDateTime();
}

float getVoltage()
{
  
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat;
}

void printDateTime()
{
  DateTime now = rtc.now();
  Serial.print(now.year());
  Serial.print("/");
  Serial.print(now.month());
  Serial.print("/");
  Serial.print(now.day());
  Serial.print(" ");
  Serial.print(now.hour());
  Serial.print(":");
  Serial.print(now.minute());
  Serial.print(":");
  Serial.println(now.second());
}

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__



int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

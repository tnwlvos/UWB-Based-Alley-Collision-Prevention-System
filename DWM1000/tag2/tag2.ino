
/**
   DW1000Ng Tag Main Code (Auto-Reset Version)
   주요 공지: 데이터 고정 현상 발생 시 ESP32 소프트웨어 리셋 로직 추가
   스타일: Allman Style
*/

#include "setting.h"
#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgConstants.hpp>

#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define BEACON 4
#define FINAL_REPORT 5
#define WARNING 6
#define TAG_REQUEST 7
#define TAG_RESPONSE 8

#define ANCHOR_NO 4
int distance_storage[ANCHOR_NO];
int other_tag_id = 0;
int other_tag_dist = 0;
int current_target_id = 0;

volatile byte expectedMsgId = POLL_ACK;
volatile boolean sentAck = false;
volatile boolean receivedAck = false;

uint64_t timePollSent, timePollAckReceived, timeRangeSent;

#define LEN_DATA 20
byte data[LEN_DATA];
uint32_t lastActivity;
uint32_t resetPeriod = 150;
uint16_t replyDelayTimeUS = 3000;

/* 리셋 감시를 위한 변수 추가 */
uint32_t last_data_update_time = 0;
const uint32_t HANG_TIMEOUT = 2000; // 2초간 데이터 변화 없으면 리셋

device_configuration_t DEFAULT_CONFIG =
{
  false, true, true, true, false,
  SFDMode::STANDARD_SFD, Channel::CHANNEL_5,
  DataRate::RATE_850KBPS, PulseFrequency::FREQ_64MHZ,
  PreambleLength::LEN_256, PreambleCode::CODE_9
};

interrupt_configuration_t DEFAULT_INTERRUPT_CONFIG = { true, true, true, false, true };

void handleSent() {
  sentAck = true;
}
void handleReceived() {
  receivedAck = true;
}

void noteActivity() {
  lastActivity = millis();
}

void transmitTagNo()
{
  data[0] = TAG_RESPONSE;
  data[17] = MY_TAG_ID;
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}

void transmitPoll()
{
  data[0] = POLL;
  for (int i = 0; i < ANCHOR_NO; i++)
  {
    data[1 + i * 2] = distance_storage[i] >> 8;
    data[1 + i * 2 + 1] = distance_storage[i] & 0xFF;
  }
  data[16] = current_target_id;
  data[17] = MY_TAG_ID;
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}

void transmitFinalReport()
{
  Serial.println("FINAL REPORT");
  data[0] = FINAL_REPORT;
  for (int i = 0; i < ANCHOR_NO; i++)
  {
    data[1 + i * 2] = distance_storage[i] >> 8;
    data[1 + i * 2 + 1] = distance_storage[i] & 0xFF;
  }
  data[17] = MY_TAG_ID;
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}

void transmitRange()
{
  Serial.println("TRASMIT RANGE");
  data[0] = RANGE;
  byte futureTimeBytes[LENGTH_TIMESTAMP];
  timeRangeSent = DW1000Ng::getSystemTimestamp() + DW1000NgTime::microsecondsToUWBTime(replyDelayTimeUS);
  DW1000NgUtils::writeValueToBytes(futureTimeBytes, timeRangeSent, LENGTH_TIMESTAMP);
  DW1000Ng::setDelayedTRX(futureTimeBytes);
  timeRangeSent += DW1000Ng::getTxAntennaDelay();
  DW1000NgUtils::writeValueToBytes(data + 1, timePollSent, LENGTH_TIMESTAMP);
  DW1000NgUtils::writeValueToBytes(data + 6, timePollAckReceived, LENGTH_TIMESTAMP);
  DW1000NgUtils::writeValueToBytes(data + 11, timeRangeSent, LENGTH_TIMESTAMP);
  data[16] = current_target_id;
  data[17] = MY_TAG_ID;
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit(TransmitMode::DELAYED);
}

unsigned long warning_millis = 0;
void dwm1000_process()
{
  /*
    if (!sentAck && !receivedAck)
    {
      if (millis() - lastActivity > (resetPeriod + random(0, 70)))
      {
        expectedMsgId = POLL_ACK;
        DW1000Ng::forceTRxOff();
        transmitPoll();
        noteActivity();
      }
      return;
    }
  */
  if ( Serial.available() )
  {
    int cmd = Serial.read();
    if ( cmd == '1' )
    {
      current_target_id = 0;
      expectedMsgId = POLL_ACK;
      DW1000Ng::forceTRxOff();
      transmitPoll();
      noteActivity();
    }
  }

  if (sentAck)
  {
    Serial.println("SEND OK");
    sentAck = false;
    receiver();
  }
  if (receivedAck)
  {
    Serial.println("RECEIVE OK");
    receivedAck = false;
    DW1000Ng::getReceivedData(data, LEN_DATA);

    if ( data[0] == BEACON )
    {
      Serial.println("BEACON");
      if ( data[1] == MY_TAG_ID )
      {
        current_target_id = 0;
        expectedMsgId = POLL_ACK;
        DW1000Ng::forceTRxOff();
        transmitPoll();
      }
      else
      {
        Serial.print("TAG PROCESS : ");
        Serial.println(data[1]);
        receiver();
      }
      noteActivity();
      return;
    }

    if ( data[0] == WARNING )
    {
      warning_millis = millis() + 500;
      noteActivity();
      receiver();
      return;
    }

    if ( data[0] == TAG_REQUEST )
    {
      Serial.println("TAG NO REQUESTED");
      delay(random(0,30));
      DW1000Ng::forceTRxOff();
      transmitTagNo();
      noteActivity();
      return;
    }
    

    Serial.print("MY TAG ID :");
    Serial.println(data[17]);
    Serial.println(data[0]);
    Serial.println(expectedMsgId);
    if ( data[17] != MY_TAG_ID )
    {
      receiver();
      return;
    }

    if (data[0] == POLL_ACK && expectedMsgId == POLL_ACK)
    {
      Serial.println("POLL ACK");
      timePollSent = DW1000Ng::getTransmitTimestamp();
      timePollAckReceived = DW1000Ng::getReceiveTimestamp();
      expectedMsgId = RANGE_REPORT;
      transmitRange();
    }
    else if (data[0] == RANGE_REPORT && expectedMsgId == RANGE_REPORT)
    {
      Serial.println("RANGE REPORT");
      float curRange;
      memcpy(&curRange, data + 1, 4);
      if (curRange > 0 && curRange < 100.0 && data[16] < ANCHOR_NO)
      {
        distance_storage[data[16]] = (int)(curRange * 1000.0);
        last_data_update_time = millis(); // 정상적으로 거리를 받았을 때 시간 업데이트
      }

      other_tag_id = data[17];
      other_tag_dist = (data[18] << 8) | data[19];

      Serial.print(data[16]);
      Serial.print(",");
      Serial.print(other_tag_id);
      Serial.print(",");
      Serial.println(other_tag_dist);

      expectedMsgId = POLL_ACK;
      current_target_id++;// = (current_target_id + 1) % ANCHOR_NO;
      if ( current_target_id <  ANCHOR_NO )
      {
        Serial.print(current_target_id);
        Serial.println(" - NEW POLL");
        expectedMsgId = POLL_ACK;
        transmitPoll();
      }
      else
      {
        transmitFinalReport();
        Serial.println("############");
        Serial.print(distance_storage[0]);
        Serial.print(",");
        Serial.print(distance_storage[1]);
        Serial.print(",");
        Serial.print(distance_storage[2]);
        Serial.print(",");
        Serial.print(distance_storage[3]);
        Serial.println("");
        current_target_id = 0;
      }
    }
    else
    {
      receiver();
    }
    noteActivity();
  }
}

void receiver()
{
  Serial.println("RECEIVER");

  DW1000Ng::forceTRxOff();
  DW1000Ng::startReceive();
}

void setup()
{
  Serial.begin(115200);
  pinMode(26, OUTPUT);

  DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);
  DW1000Ng::setNetworkId(10);
  DW1000Ng::setAntennaDelay(16436);
  DW1000Ng::attachSentHandler(handleSent);
  DW1000Ng::attachReceivedHandler(handleReceived);

  Serial.print("TAG START - ID: ");
  Serial.println(MY_TAG_ID);

  last_data_update_time = millis(); // 초기화
  //    transmitPoll();
  receiver();
  noteActivity();
}

void loop()
{
  if ( warning_millis > millis() )
  {
    digitalWrite(26, (millis()/500)%2);
  }
  else
  {
    digitalWrite(26, LOW);
  }

  /* 먹통 방지 리셋 로직 */
  if (millis() - last_data_update_time > HANG_TIMEOUT)
  {
    Serial.println("System Hang Detected! Restarting...");
    delay(100);
    ESP.restart();
  }

  dwm1000_process();
  /*
    static unsigned long p = 0;
    if (millis() - p > 200)
    {
      Serial.print("MY("); Serial.print(MY_TAG_ID); Serial.print("):");
      for (int i = 0; i < ANCHOR_NO; i++)
      {
        Serial.print(distance_storage[i]); Serial.print(i == ANCHOR_NO - 1 ? "" : ",");
      }
      if (other_tag_id > 0 && other_tag_dist > 0)
      {
        Serial.print(" | OTHER("); Serial.print(other_tag_id);
        Serial.print("):"); Serial.print(other_tag_dist);
      }
      Serial.println();
      p = millis();
    }
  */
}

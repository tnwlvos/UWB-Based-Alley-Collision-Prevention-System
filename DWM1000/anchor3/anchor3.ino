/**
   DW1000Ng Anchor Main Code
   사용버전: DW1000Ng Library
   주요 공지: setting.h 연동, 태그별 출력 간격 제어(200ms) 적용
   스타일: Allman Style
*/

#include "setting.h"
#define MAX_TAG_ID 256
#define ANCHOR_NO 4

#define USE_PA_LNA
#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>

unsigned long prev_succeed_millis;
uint16_t distance_storage[ANCHOR_NO];
uint16_t all_tags_dist[MAX_TAG_ID];

int heard_tag_list[10];
unsigned long heard_tag_millis[10];
unsigned long heard_tag_report_millis[10];

/* 태그별 출력 안정화를 위한 변수 */
uint32_t last_tag_print[MAX_TAG_ID];
const uint32_t HEARD_TAG_REPORT_INTERVAL = 1000;
const uint32_t PRINT_INTERVAL = 100; // 출력 간격 (0.2초)

#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define BEACON 4
#define FINAL_REPORT 5
#define WARNING 6
#define TAG_REQUEST 7
#define TAG_RESPONSE 8
#define ANCHOR_TAG_REPORT 9
#define DELEGATED_FINAL_REPORT 11

volatile byte expectedMsgId = POLL;
volatile boolean sentAck = false;
volatile boolean receivedAck = false;

uint64_t tps, tpr, tpas, tpar, trs, trr;

#define LEN_DATA 20
byte data[LEN_DATA];
uint32_t lastActivity;
uint32_t resetPeriod = 1000;

device_configuration_t DEFAULT_CONFIG =
{
  false, true, true, true, false,
  SFDMode::STANDARD_SFD,
  Channel::CHANNEL_2,
  DataRate::RATE_110KBPS,
  PulseFrequency::FREQ_64MHZ,
  PreambleLength::LEN_1024,
  PreambleCode::CODE_9
};

interrupt_configuration_t DEFAULT_INTERRUPT_CONFIG = { true, true, true, false, true };

void handleSent() {
  sentAck = true;
}
void handleReceived() {
  receivedAck = true;
}

void receiver()
{
  DW1000Ng::forceTRxOff();
  DW1000Ng::startReceive();
}

void noteActivity() {
  lastActivity = millis();
}

void resetInactive()
{
  expectedMsgId = POLL;
  receiver();
  noteActivity();
}

void transmitPollAck()
{
  data[0] = POLL_ACK;
  DW1000Ng::forceTRxOff();
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}

void transmitRangeReport(float curRange, byte targetTagId)
{
  data[0] = RANGE_REPORT;
  memcpy(data + 1, &curRange, 4);

  data[16] = MY_ID;
  byte relayTagId = data[17];//(targetTagId == 1) ? 2 : 1;
  //  data[17] = relayTagId;
  uint16_t relayDist = relayTagId < MAX_TAG_ID ? all_tags_dist[relayTagId] : 0;
  data[18] = relayDist >> 8;
  data[19] = relayDist & 0xFF;

  DW1000Ng::forceTRxOff();
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}

void setup()
{
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  Serial.begin(115200);

  for (int i = 0; i < 10; i++)
  {
    heard_tag_list[i] = -1;
    heard_tag_millis[i] = 0;
    heard_tag_report_millis[i] = 0;
  }

  DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);
  DW1000Ng::setDeviceAddress(MY_ID + 1);
  DW1000Ng::setAntennaDelay(16436);
  DW1000Ng::attachSentHandler(handleSent);
  DW1000Ng::attachReceivedHandler(handleReceived);
  receiver();
  noteActivity();
  prev_succeed_millis = millis();
}

void send_beacon(int id)
{
  data[0] = BEACON;
  data[1] = id;
  data[16] = MY_ID;
  data[17] = id;
  DW1000Ng::forceTRxOff();
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}

void transmitAnchorTagReport(byte tagId)
{
  data[0] = ANCHOR_TAG_REPORT;
  data[1] = MY_ID;
  data[16] = 0;
  data[17] = tagId;

  DW1000Ng::forceTRxOff();
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}

void transmitDelegatedFinalReport(byte tagId)
{
  data[0] = DELEGATED_FINAL_REPORT;
  data[15] = MY_ID;
  data[16] = 0;
  data[17] = tagId;

  DW1000Ng::forceTRxOff();
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}

bool update_heard_tag_status(int tag_id)
{
  for (int i = 0; i < 10; i++)
  {
    if (heard_tag_list[i] == tag_id)
    {
      heard_tag_millis[i] = millis();
      if (millis() - heard_tag_report_millis[i] >= HEARD_TAG_REPORT_INTERVAL)
      {
        heard_tag_report_millis[i] = millis();
        return true;
      }
      return false;
    }
  }

  for (int i = 0; i < 10; i++)
  {
    if (heard_tag_list[i] == -1)
    {
      heard_tag_list[i] = tag_id;
      heard_tag_millis[i] = millis();
      heard_tag_report_millis[i] = millis();
      return true;
    }
  }

  return false;
}

int id = 1;
unsigned long prev_send_beacon_millis;
void loop()
{
  // anchor 0 만 마스터 모드임.
  if ( MY_ID == 0 )
  {
    if ( ( millis() - prev_send_beacon_millis ) > 100 )
    {
      prev_send_beacon_millis = millis();
      send_beacon(id);
      id++;
      if ( id > 2 ) id = 1;
    }
  }

  if ( Serial.available() )
  {
    int cmd = Serial.read();
    if ( cmd == '1' )
    {
      send_beacon(1);
    }
    if ( cmd == '2' )
    {
      send_beacon(2);
    }
  }

  if ((millis() - prev_succeed_millis) > 10000) ESP.restart();

  if (!sentAck && !receivedAck)
  {
    if (millis() - lastActivity > resetPeriod) resetInactive();
    return;
  }

  if (sentAck)
  {
    sentAck = false;
    if (data[0] == POLL_ACK) tpas = DW1000Ng::getTransmitTimestamp();
    DW1000Ng::startReceive();
  }

  if (receivedAck)
  {
    receivedAck = false;
    DW1000Ng::getReceivedData(data, LEN_DATA);
    byte msgId = data[0];
    byte current_tag_id = data[17];

    if (msgId == TAG_RESPONSE && data[16] == MY_ID)
    {
      if (update_heard_tag_status(current_tag_id))
      {
        transmitAnchorTagReport(current_tag_id);
        noteActivity();
        return;
      }

      receiver();
      noteActivity();
      return;
    }

    if (msgId == POLL)
    {
      for (int i = 0; i < ANCHOR_NO; i++)
      {
        distance_storage[i] = ((uint16_t)data[1 + i * 2] << 8) | data[1 + i * 2 + 1];
      }
    }

    if ( msgId == FINAL_REPORT )
    {
      receiver();
      noteActivity();
      return;
    }

    if (data[16] != MY_ID)
    {
      resetInactive();
      receiver();
      return;
    }

    if (msgId == POLL)
    {
      tpr = DW1000Ng::getReceiveTimestamp();
      expectedMsgId = RANGE;
      transmitPollAck();
    }
    else if (msgId == RANGE)
    {
      trr = DW1000Ng::getReceiveTimestamp();
      expectedMsgId = POLL;
      tps = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
      tpar = DW1000NgUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);
      trs = DW1000NgUtils::bytesAsValue(data + 11, LENGTH_TIMESTAMP);

      double dist = DW1000NgRanging::computeRangeAsymmetric(tps, tpr, tpas, tpar, trs, trr);
      dist = DW1000NgRanging::correctRange(dist);

      if (dist > 0 && dist < 100.0)
      {
        /*
                uint16_t curDistInt = (uint16_t)min(dist * 1000.0, 65535.0);
                if (current_tag_id < MAX_TAG_ID) all_tags_dist[current_tag_id] = curDistInt;

                // 태그별 출력 간격 제어 로직
                if (current_tag_id < MAX_TAG_ID && (millis() - last_tag_print[current_tag_id] >= PRINT_INTERVAL))
                {
                  Serial.print("T"); Serial.print(current_tag_id); Serial.print(":");
                  for (int i = 0; i < ANCHOR_NO; i++)
                  {
                    Serial.print(i == MY_ID ? curDistInt : distance_storage[i]);
                    Serial.print(i == ANCHOR_NO - 1 ? "" : ",");
                  }
                  Serial.println();
                  last_tag_print[current_tag_id] = millis();
                }
        */
        transmitRangeReport(dist, current_tag_id);
        prev_succeed_millis = millis();
      }
      else
      {
        receiver();
      }
    }
    else
    {
      receiver();
    }
    noteActivity();
  }
}

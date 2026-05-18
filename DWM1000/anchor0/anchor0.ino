/**
   DW1000Ng Anchor Main Code
   사용버전: DW1000Ng Library
   주요 공지: setting.h 연동, 태그별 출력 간격 제어(200ms) 적용
   스타일: Allman Style
*/

int active_tag_count = 0;
int active_tag_list[10];

// 태그 활성화 관리 DB
int tag_list[10]; // 태그 번호
unsigned long tag_millis[10]; // 태그와 마지막으로 통신한 시각

int id_index = 0;
unsigned long prev_send_beacon_millis;

unsigned long tag_id_request_millis; // 태그들이게 ID요청한 시각
#define TAG_RESPONSE_DELAY 100 // 태그에게 ID 요청후 응답 대기시각

#include "setting.h"
#define MAX_TAG_ID 30
#define ANCHOR_NO 4

#define USE_PA_LNA
#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>

unsigned long prev_succeed_millis;
int16_t distance_storage[ANCHOR_NO];
int16_t all_tags_dist[MAX_TAG_ID];

/* 태그별 출력 안정화를 위한 변수 */
uint32_t last_tag_print[MAX_TAG_ID];
const uint32_t PRINT_INTERVAL = 100;

#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define BEACON 4
#define FINAL_REPORT 5
#define WARNING 6
#define TAG_REQUEST 7
#define TAG_RESPONSE 8

volatile byte expectedMsgId = POLL;
volatile boolean sentAck = false;
volatile boolean receivedAck = false;

uint64_t tps, tpr, tpas, tpar, trs, trr;

#define LEN_DATA 20
byte data[LEN_DATA];
uint32_t lastActivity;
uint32_t resetPeriod = 250;

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

void transmitWarning()
{
  data[0] = WARNING;
  DW1000Ng::forceTRxOff();
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}

void transmitPollAck()
{
  // Serial.println("TRANSMIT POLL ACK");
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
  data[18] = all_tags_dist[relayTagId] >> 8;
  data[19] = all_tags_dist[relayTagId] & 0xFF;

  DW1000Ng::forceTRxOff();
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}

void setup()
{
  pinMode(2, OUTPUT);
  pinMode(27, OUTPUT);
  digitalWrite(2, HIGH);
  Serial.begin(115200);

  for ( int i = 0; i < 10; i++)
  {
    tag_list[i] = -1;
    tag_millis[i] = 0;
    active_tag_list[i] = -1;
  }

  // Serial.print("ANCHOR START - ID: ");
  // Serial.println(MY_ID);

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
  // Serial.println("BEACON");
  data[0] = BEACON;
  data[1] = id;
  DW1000Ng::forceTRxOff();
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}

void send_tag_request()
{
  // Serial.println("BEACON");
  data[0] = TAG_REQUEST;
  DW1000Ng::forceTRxOff();
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
  tag_id_request_millis = millis();
}

// 일정시간 응답없는 태그 삭제
void remove_idle_tag_list()
{
  for ( int i = 0; i < 10; i++)
  {
    if ( ( millis() - tag_millis[i] ) > 500 )
    {
      tag_list[i] = -1;
    }
  }
}

void update_active_tag_list()
{
  int tag_index = 0;
  for ( int i = 0; i < 10; i++)
  {
    if ( tag_list[i] != -1 )
    {
      active_tag_list[tag_index] = tag_list[i];
      tag_index++;
    }
  }

  active_tag_count = tag_index;

  Serial.print("TAG LIST : ");
  for ( int i = 0; i < active_tag_count; i++)
  {
    Serial.print(active_tag_list[i]);
    Serial.print(",");
  }
  Serial.println("");
}

// 해당 태그가 리스트에 없으면 추가하고
// 있으면 시간을 업데이트
bool update_tag_status(int tag_id)
{
  bool flag = false;
  for ( int i = 0; i < 10; i++)
  {
    if ( tag_list[i] == data[17] )
    {
      tag_millis[i] = millis();
      /*
            Serial.print(i);
            Serial.print("슬롯에 ");
            Serial.println("시간 업데이트");
      */
      flag = true;
    }
  }
  if ( flag == false )
  {
    // 빈슬롯(-1인 슬롯) 찾기
    bool flag2 = false;
    for ( int j = 0; j < 10; j++)
    {
      if ( tag_list[j] == -1 )
      {
        tag_list[j] = data[17];
        tag_millis[j] = millis();
        /*
                Serial.print(j);
                Serial.print("슬롯에 ");
                Serial.print(data[17]);
                Serial.println("태그 추가 완료");
        */
        flag2 = true;
        break;
      }
    }
    if ( flag2 == false )
    {
      //      Serial.println("빈 슬롯 찾기 실패");
      return false;
    }
  }
  return true;
}

void loop()
{
  if ( Serial.available() )
  {
    int cmd = Serial.read();
    if ( cmd == '1' )
    {
      transmitWarning();
      digitalWrite(27, HIGH);
    }
    if ( cmd == '0' )
    {
      digitalWrite(27, LOW);
    }

    if ( cmd == '2' )
    {
      send_tag_request();
    }
  }
  // anchor 0 만 마스터 모드임.
  if ( MY_ID == 0 )
  {
    if ( ( millis() - prev_send_beacon_millis ) > PRINT_INTERVAL )
    {
      prev_send_beacon_millis = millis();

      // 처음시작시 활성화 태그 정리부터함.
      if ( id_index == 0 )
      {
        remove_idle_tag_list();
        update_active_tag_list();
      }

      if ( id_index == active_tag_count )
      {
        send_tag_request();
      }
      else
      {
        send_beacon(active_tag_list[id_index]);
      }
      id_index++;
      if ( id_index > active_tag_count ) id_index = 0;
    }
  }
  /*
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
  */
  if ((millis() - prev_succeed_millis) > 2000) ESP.restart();

  if (!sentAck && !receivedAck)
  {
    if (millis() - lastActivity > resetPeriod) resetInactive();
    return;
  }

  if (sentAck)
  {
    sentAck = false;
    // Serial.println("SEND OK");
    if (data[0] == POLL_ACK) tpas = DW1000Ng::getTransmitTimestamp();
    DW1000Ng::startReceive();
  }

  if (receivedAck)
  {
    // Serial.println("RECEIVED");
    receivedAck = false;
    DW1000Ng::getReceivedData(data, LEN_DATA);
    byte msgId = data[0];
    byte current_tag_id = data[17];


    // 태그 응답 대기중인 경우
    if ( millis() < tag_id_request_millis + TAG_RESPONSE_DELAY )
    {
      prev_send_beacon_millis = millis(); // 거리 측정 시작하지 않기 위해서
      if (msgId == TAG_RESPONSE )
      {
/*        
        Serial.print("TAG LIST : ");
        Serial.println(data[17]);
*/        
        update_tag_status(data[17]);
        receiver();
      }
      return;
    }

    if (msgId == POLL)
    {
      update_tag_status(data[17]);
      
      // Serial.println("RECEIVED POLL");
      for (int i = 0; i < ANCHOR_NO; i++)
      {
        distance_storage[i] = (data[1 + i * 2] << 8) | data[1 + i * 2 + 1];
      }
      
    }

    if ( msgId == FINAL_REPORT )
    {
      update_tag_status(data[17]);

      for (int i = 0; i < ANCHOR_NO; i++)
      {
        distance_storage[i] = (data[1 + i * 2] << 8) | data[1 + i * 2 + 1];
      }

      Serial.print("T"); Serial.print(current_tag_id); Serial.print(":");
      for (int i = 0; i < ANCHOR_NO; i++)
      {
        Serial.print(distance_storage[i]);
        Serial.print(i == ANCHOR_NO - 1 ? "" : ",");
      }
      Serial.println();
      last_tag_print[current_tag_id] = millis();

      receiver();
    }

    // Serial.print("ID = ");
    // Serial.println(data[16]);
    // Serial.println(msgId);

    if (data[16] != MY_ID)
    {
      // Serial.println("Not My Msg");
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
      update_tag_status(data[17]);
      // Serial.println("RANGE");
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
                int16_t curDistInt = (int16_t)(dist * 1000.0);
                if (current_tag_id < MAX_TAG_ID) all_tags_dist[current_tag_id] = curDistInt;

                // 태그별 출력 간격 제어 로직
                if (current_tag_id < MAX_TAG_ID && (millis() - last_tag_print[current_tag_id] >= PRINT_INTERVAL))
                {
                  // Serial.print("T"); // Serial.print(current_tag_id); // Serial.print(":");
                  for (int i = 0; i < ANCHOR_NO; i++)
                  {
                    // Serial.print(i == MY_ID ? curDistInt : distance_storage[i]);
                    // Serial.print(i == ANCHOR_NO - 1 ? "" : ",");
                  }
                  // Serial.println();
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

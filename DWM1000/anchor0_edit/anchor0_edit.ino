/**
   DW1000Ng Anchor Main Code
   장거리 안정화 설정 반영 버전
*/

int active_tag_count = 0;
int active_tag_list[10];
int active_tag_heard_anchor[10];

int tag_list[10];
unsigned long tag_millis[10];
int tag_heard_anchor[10];
int prev_active_tag_count = -1;
int prev_active_tag_list[10];
int prev_active_tag_heard_anchor[10];
int last_reported_tag_id_by_anchor[4];
unsigned long last_anchor_report_print_millis[4];

int id_index = 0;
unsigned long prev_send_beacon_millis;

unsigned long tag_id_request_millis;
#define TAG_RESPONSE_DELAY 100

#include "setting.h"
#define MAX_TAG_ID 30
#define ANCHOR_NO 4

#define USE_PA_LNA
#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>

unsigned long prev_succeed_millis;
uint16_t distance_storage[ANCHOR_NO];
uint16_t all_tags_dist[MAX_TAG_ID];

uint32_t last_tag_print[MAX_TAG_ID];
const uint32_t PRINT_INTERVAL = 100;
const uint32_t ANCHOR_REPORT_PRINT_INTERVAL = 1000;
const uint32_t TAG_IDLE_TIMEOUT = 6000;

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
#define BEACON_DELEGATE 10
#define DELEGATED_FINAL_REPORT 11

volatile byte expectedMsgId = POLL;
volatile boolean sentAck = false;
volatile boolean receivedAck = false;

uint64_t tps, tpr, tpas, tpar, trs, trr;

#define LEN_DATA 20
byte data[LEN_DATA];

uint32_t lastActivity;
uint32_t resetPeriod = 1000;   // 기존 250 → 500

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

void handleSent()
{
  sentAck = true;
}

void handleReceived()
{
  receivedAck = true;
}

void receiver()
{
  DW1000Ng::forceTRxOff();
  DW1000Ng::startReceive();
}

void noteActivity()
{
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
  byte relayTagId = data[17];
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
  pinMode(27, OUTPUT);
  digitalWrite(2, HIGH);
  Serial.begin(115200);

  for (int i = 0; i < 10; i++)
  {
    tag_list[i] = -1;
    tag_millis[i] = 0;
    tag_heard_anchor[i] = -1;
    active_tag_list[i] = -1;
    active_tag_heard_anchor[i] = -1;
    prev_active_tag_list[i] = -1;
    prev_active_tag_heard_anchor[i] = -1;
  }

  for (int i = 0; i < ANCHOR_NO; i++)
  {
    last_reported_tag_id_by_anchor[i] = -1;
    last_anchor_report_print_millis[i] = 0;
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

void send_beacon_delegate(int delegate_anchor_id, int tag_id)
{
  Serial.print("DELEGATE TAG ");
  Serial.print(tag_id);
  Serial.print(" TO A");
  Serial.println(delegate_anchor_id);

  data[0] = BEACON_DELEGATE;
  data[1] = tag_id;
  data[16] = delegate_anchor_id;
  data[17] = tag_id;

  DW1000Ng::forceTRxOff();
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}

void send_tag_request()
{
  data[0] = TAG_REQUEST;
  data[16] = MY_ID;
  data[17] = 0;

  DW1000Ng::forceTRxOff();
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();

  tag_id_request_millis = millis();
}

void remove_idle_tag_list()
{
  for (int i = 0; i < 10; i++)
  {
    if ((millis() - tag_millis[i]) > TAG_IDLE_TIMEOUT)
    {
      tag_list[i] = -1;
      tag_heard_anchor[i] = -1;
    }
  }
}

void update_active_tag_list()
{
  int tag_index = 0;

  for (int i = 0; i < 10; i++)
  {
    if (tag_list[i] != -1)
    {
      active_tag_list[tag_index] = tag_list[i];
      active_tag_heard_anchor[tag_index] = tag_heard_anchor[i];
      tag_index++;
    }
  }

  active_tag_count = tag_index;

  bool changed = active_tag_count != prev_active_tag_count;

  for (int i = 0; i < active_tag_count && !changed; i++)
  {
    if (active_tag_list[i] != prev_active_tag_list[i])
    {
      changed = true;
    }
    if (active_tag_heard_anchor[i] != prev_active_tag_heard_anchor[i])
    {
      changed = true;
    }
  }

  if (changed)
  {
    Serial.print("TAG LIST : ");
    for (int i = 0; i < active_tag_count; i++)
    {
      Serial.print(active_tag_list[i]);
      Serial.print("(A");
      Serial.print(active_tag_heard_anchor[i]);
      Serial.print(")");
      Serial.print(",");
    }
    Serial.println("");

    prev_active_tag_count = active_tag_count;
    for (int i = 0; i < 10; i++)
    {
      prev_active_tag_list[i] = active_tag_list[i];
      prev_active_tag_heard_anchor[i] = active_tag_heard_anchor[i];
    }
  }
}

int heard_anchor_priority(int heard_anchor_id)
{
  if (heard_anchor_id == 0) return 4;
  if (heard_anchor_id == 2) return 3;
  if (heard_anchor_id == 3) return 2;
  if (heard_anchor_id == 1) return 1;
  return 0;
}

void update_tag_heard_anchor(int slot, int heard_anchor_id)
{
  if (tag_heard_anchor[slot] == -1 ||
      heard_anchor_priority(heard_anchor_id) > heard_anchor_priority(tag_heard_anchor[slot]))
  {
    tag_heard_anchor[slot] = heard_anchor_id;
  }
}

bool update_tag_status(int tag_id, int heard_anchor_id)
{
  bool flag = false;

  for (int i = 0; i < 10; i++)
  {
    if (tag_list[i] == tag_id)
    {
      tag_millis[i] = millis();
      update_tag_heard_anchor(i, heard_anchor_id);
      flag = true;
    }
  }

  if (flag == false)
  {
    bool flag2 = false;

    for (int j = 0; j < 10; j++)
    {
      if (tag_list[j] == -1)
      {
        tag_list[j] = tag_id;
        tag_millis[j] = millis();
        tag_heard_anchor[j] = heard_anchor_id;
        flag2 = true;
        break;
      }
    }

    if (flag2 == false)
    {
      return false;
    }
  }

  return true;
}

bool should_print_anchor_tag_report(byte reporter_anchor_id, int tag_id)
{
  if (reporter_anchor_id >= ANCHOR_NO)
  {
    return true;
  }

  if (last_reported_tag_id_by_anchor[reporter_anchor_id] != tag_id ||
      millis() - last_anchor_report_print_millis[reporter_anchor_id] >= ANCHOR_REPORT_PRINT_INTERVAL)
  {
    last_reported_tag_id_by_anchor[reporter_anchor_id] = tag_id;
    last_anchor_report_print_millis[reporter_anchor_id] = millis();
    return true;
  }

  return false;
}

void loop()
{
  if (Serial.available())
  {
    int cmd = Serial.read();

    if (cmd == '1')
    {
      transmitWarning();
      digitalWrite(27, HIGH);
    }

    if (cmd == '0')
    {
      digitalWrite(27, LOW);
    }

    if (cmd == '2')
    {
      send_tag_request();
    }
  }

  if (MY_ID == 0)
  {
    if ((millis() - prev_send_beacon_millis) > PRINT_INTERVAL)
    {
      prev_send_beacon_millis = millis();

      if (id_index == 0)
      {
        remove_idle_tag_list();
        update_active_tag_list();
      }

      if (id_index == active_tag_count)
      {
        send_tag_request();
      }
      else
      {
        if (active_tag_heard_anchor[id_index] != 2)
        {
          send_beacon(active_tag_list[id_index]);
        }
        else
        {
          send_beacon_delegate(active_tag_heard_anchor[id_index], active_tag_list[id_index]);
        }
      }

      id_index++;
      if (id_index > active_tag_count) id_index = 0;
    }
  }

  if ((millis() - prev_succeed_millis) > 10000) ESP.restart();   // 기존 2000 → 5000

  if (!sentAck && !receivedAck)
  {
    if (millis() - lastActivity > resetPeriod) resetInactive();
    return;
  }

  if (sentAck)
  {
    sentAck = false;

    if (data[0] == POLL_ACK)
    {
      tpas = DW1000Ng::getTransmitTimestamp();
    }

    DW1000Ng::startReceive();
  }

  if (receivedAck)
  {
    receivedAck = false;
    DW1000Ng::getReceivedData(data, LEN_DATA);

    byte msgId = data[0];
    byte current_tag_id = data[17];

    if (msgId == ANCHOR_TAG_REPORT)
    {
      byte reporter_anchor_id = data[1];
      update_tag_status(current_tag_id, reporter_anchor_id);

      if (should_print_anchor_tag_report(reporter_anchor_id, current_tag_id))
      {
        Serial.print("ANCHOR TAG REPORT FROM A");
        Serial.print(reporter_anchor_id);
        Serial.print(" : ");
        Serial.println(current_tag_id);
      }

      receiver();
      prev_succeed_millis = millis();
      noteActivity();
      return;
    }

    if (millis() < tag_id_request_millis + TAG_RESPONSE_DELAY)
    {
      prev_send_beacon_millis = millis();

      if (msgId == TAG_RESPONSE)
      {
        update_tag_status(data[17], MY_ID);
        prev_succeed_millis = millis();
        receiver();
      }

      return;
    }

    if (msgId == POLL)
    {
      update_tag_status(data[17], MY_ID);
      prev_succeed_millis = millis();

      for (int i = 0; i < ANCHOR_NO; i++)
      {
        distance_storage[i] = ((uint16_t)data[1 + i * 2] << 8) | data[1 + i * 2 + 1];
      }
    }

    if (msgId == FINAL_REPORT || msgId == DELEGATED_FINAL_REPORT)
    {
      update_tag_status(data[17], msgId == DELEGATED_FINAL_REPORT ? data[15] : MY_ID);
      prev_succeed_millis = millis();

      for (int i = 0; i < ANCHOR_NO; i++)
      {
        distance_storage[i] = ((uint16_t)data[1 + i * 2] << 8) | data[1 + i * 2 + 1];
      }

      Serial.print("T");
      Serial.print(current_tag_id);
      Serial.print(":");

      for (int i = 0; i < ANCHOR_NO; i++)
      {
        Serial.print(distance_storage[i]);
        Serial.print(i == ANCHOR_NO - 1 ? "" : ",");
      }

      Serial.println();
      if (current_tag_id < MAX_TAG_ID)
      {
        last_tag_print[current_tag_id] = millis();
      }

      receiver();
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
      update_tag_status(data[17], MY_ID);

      trr = DW1000Ng::getReceiveTimestamp();
      expectedMsgId = POLL;

      tps = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
      tpar = DW1000NgUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);
      trs = DW1000NgUtils::bytesAsValue(data + 11, LENGTH_TIMESTAMP);

      double dist = DW1000NgRanging::computeRangeAsymmetric(tps, tpr, tpas, tpar, trs, trr);
      dist = DW1000NgRanging::correctRange(dist);

      if (dist > 0 && dist < 100.0)
      {
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

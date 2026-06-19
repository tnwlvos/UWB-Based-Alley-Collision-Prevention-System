/**
   DW1000Ng Anchor Main Code
   장거리 안정화 설정 반영 버전
*/

#include "setting.h"
#define MAX_TAG_ID 256
#define ANCHOR_NO 4
#define DIST_INVALID 65535

int active_tag_count = 0;
int active_tag_list[10];
int active_tag_heard_anchor[10];
int tag_list[10];
unsigned long tag_millis[10];
unsigned long tag_distance_millis[10];
int tag_heard_anchor[10];
int prev_active_tag_count = -1;
int prev_active_tag_list[10];
int prev_active_tag_heard_anchor[10];
int last_reported_tag_id_by_anchor[4];
unsigned long last_anchor_report_print_millis[4];
byte tag_risk_level[MAX_TAG_ID];
bool pending_risk_notify[MAX_TAG_ID];
char serial_line_buffer[32];
byte serial_line_index = 0;

int id_index = 0;
unsigned long prev_send_beacon_millis;
bool beacon_outstanding = false;
int outstanding_tag_id = -1;
unsigned long beacon_sent_millis = 0;
unsigned long last_final_report_millis = 0;

unsigned long tag_id_request_millis;
#define TAG_RESPONSE_DELAY 120
bool a0_discovery_waiting = false;
bool a0_found_tag_this_discovery = false;
unsigned long a0_discovery_start_millis = 0;
bool a2_discovery_waiting = false;
unsigned long a2_discovery_start_millis = 0;
uint8_t a2_aux_discovery_counter = 0;

#define USE_PA_LNA
#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>

unsigned long prev_succeed_millis;
uint16_t distance_storage[ANCHOR_NO];
uint16_t all_tags_dist[MAX_TAG_ID];

uint32_t last_tag_print[MAX_TAG_ID];
uint32_t last_tag_report_hash[MAX_TAG_ID];
const uint32_t PRINT_INTERVAL = 100;
const uint32_t DUPLICATE_REPORT_SUPPRESS_MS = 80;
const uint32_t POST_FINAL_REPORT_GUARD_MS = 40;
const uint32_t ANCHOR_REPORT_PRINT_INTERVAL = 1000;
const uint32_t TAG_IDLE_TIMEOUT = 10000;
const uint32_t BEACON_SLOT_GUARD_MS = 500;
const uint32_t TAG_REQUEST_INTERVAL_EMPTY = 300;
const uint32_t TAG_REQUEST_INTERVAL_ACTIVE = 1500;
const uint32_t A0_DIRECT_DISCOVERY_WAIT = 100;
const uint32_t A0_A2_DISCOVERY_TIMEOUT = 150;

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
#define A2_DISCOVERY_GO 12
#define A2_DISCOVERY_REPORT 13
#define A2_DISCOVERY_EMPTY 14

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
  data[1] = 255;
  data[2] = 2;
  data[16] = MY_ID;
  data[17] = 255;
  DW1000Ng::forceTRxOff();
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}

void transmitRiskNotify(byte tag_id)
{
  data[0] = WARNING;
  data[1] = tag_id;
  data[2] = tag_risk_level[tag_id];
  data[16] = MY_ID;
  data[17] = tag_id;

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
    tag_distance_millis[i] = 0;
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

  for (int i = 0; i < MAX_TAG_ID; i++)
  {
    last_tag_print[i] = 0;
    last_tag_report_hash[i] = 0;
    all_tags_dist[i] = DIST_INVALID;
    tag_risk_level[i] = 0;
    pending_risk_notify[i] = false;
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
  data[2] = id < MAX_TAG_ID ? tag_risk_level[id] : 0;
  data[16] = MY_ID;
  data[17] = id;

  DW1000Ng::forceTRxOff();
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}

void send_beacon_delegate(int delegate_anchor_id, int tag_id)
{
  data[0] = BEACON_DELEGATE;
  data[1] = tag_id;
  data[2] = tag_id < MAX_TAG_ID ? tag_risk_level[tag_id] : 0;
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

void start_a0_discovery()
{
  send_tag_request();
  a0_discovery_waiting = true;
  a0_found_tag_this_discovery = false;
  a0_discovery_start_millis = millis();
}

void send_a2_discovery_go()
{
  data[0] = A2_DISCOVERY_GO;
  data[16] = 2;
  data[17] = 0;

  DW1000Ng::forceTRxOff();
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();

  a2_discovery_waiting = true;
  a2_discovery_start_millis = millis();
}

bool should_send_scheduled_tag_request()
{
  uint32_t interval = active_tag_count == 0 ? TAG_REQUEST_INTERVAL_EMPTY : TAG_REQUEST_INTERVAL_ACTIVE;
  return tag_id_request_millis == 0 || millis() - tag_id_request_millis >= interval;
}

bool is_discovery_busy()
{
  return a0_discovery_waiting || a2_discovery_waiting;
}

bool is_post_final_report_guard_active()
{
  return millis() - last_final_report_millis < POST_FINAL_REPORT_GUARD_MS;
}

void remove_idle_tag_list()
{
  for (int i = 0; i < 10; i++)
  {
    if (tag_list[i] == -1)
    {
      continue;
    }

    if ((millis() - tag_millis[i]) > TAG_IDLE_TIMEOUT)
    {
      tag_list[i] = -1;
      tag_millis[i] = 0;
      tag_distance_millis[i] = 0;
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
        tag_distance_millis[j] = millis();
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

void mark_tag_distance_received(int tag_id)
{
  for (int i = 0; i < 10; i++)
  {
    if (tag_list[i] == tag_id)
    {
      tag_distance_millis[i] = millis();
      return;
    }
  }
}

void mark_beacon_outstanding(int tag_id)
{
  beacon_outstanding = true;
  outstanding_tag_id = tag_id;
  beacon_sent_millis = millis();
}

void clear_beacon_outstanding()
{
  beacon_outstanding = false;
  outstanding_tag_id = -1;
  beacon_sent_millis = 0;
}

void finish_a0_discovery_if_needed()
{
  if (!a0_discovery_waiting)
  {
    return;
  }

  if (millis() - a0_discovery_start_millis < A0_DIRECT_DISCOVERY_WAIT)
  {
    return;
  }

  a0_discovery_waiting = false;
  remove_idle_tag_list();
  update_active_tag_list();

  bool should_run_a2_discovery = false;

  if (!a0_found_tag_this_discovery)
  {
    should_run_a2_discovery = true;
    a2_aux_discovery_counter = 0;
  }
  else if (active_tag_count == 1)
  {
    a2_aux_discovery_counter++;
    if (a2_aux_discovery_counter >= 3)
    {
      should_run_a2_discovery = true;
      a2_aux_discovery_counter = 0;
    }
  }
  else
  {
    a2_aux_discovery_counter = 0;
  }

  if (should_run_a2_discovery)
  {
    send_a2_discovery_go();
  }
}

void handle_a2_discovery_timeout()
{
  if (a2_discovery_waiting && millis() - a2_discovery_start_millis > A0_A2_DISCOVERY_TIMEOUT)
  {
    a2_discovery_waiting = false;
  }
}

void handle_a2_discovery_report()
{
  byte reporter_anchor_id = data[1];
  byte tag_count = data[2];
  if (tag_count > 8) tag_count = 8;

  for (byte i = 0; i < tag_count; i++)
  {
    update_tag_status(data[3 + i], reporter_anchor_id);
  }

  a2_discovery_waiting = false;
  update_active_tag_list();
}

void handle_beacon_slot_timeout()
{
  if (!beacon_outstanding)
  {
    return;
  }

  if (millis() - beacon_sent_millis <= BEACON_SLOT_GUARD_MS)
  {
    return;
  }

  // Keep the tag in the active list through short vehicle-body/RF dropouts.
  // Actual removal is handled by TAG_IDLE_TIMEOUT in remove_idle_tag_list().

  prev_succeed_millis = millis();
  clear_beacon_outstanding();
}

void handle_risk_command(char *line)
{
  if (line[0] != 'R' || line[1] != ',')
  {
    return;
  }

  char *tagPart = line + 2;
  char *levelPart = strchr(tagPart, ',');
  if (levelPart == NULL)
  {
    return;
  }

  *levelPart = '\0';
  levelPart++;

  int tagId = atoi(tagPart);
  int level = atoi(levelPart);
  if (tagId < 0 || tagId >= MAX_TAG_ID)
  {
    return;
  }

  if (level < 0) level = 0;
  if (level > 2) level = 2;

  if (tag_risk_level[tagId] != (byte)level)
  {
    tag_risk_level[tagId] = (byte)level;
    pending_risk_notify[tagId] = true;
  }
}

void handle_serial_byte(char cmd)
{
  if (cmd == '1')
  {
    transmitWarning();
    digitalWrite(27, HIGH);
    return;
  }

  if (cmd == '0')
  {
    digitalWrite(27, LOW);
    return;
  }

  if (cmd == '2')
  {
    if (!beacon_outstanding && !is_discovery_busy())
    {
      start_a0_discovery();
    }
  }
}

void handle_serial_commands()
{
  while (Serial.available())
  {
    char c = (char)Serial.read();
    if (c == '\r')
    {
      continue;
    }

    if (c == '\n')
    {
      serial_line_buffer[serial_line_index] = '\0';
      if (serial_line_index > 0)
      {
        handle_risk_command(serial_line_buffer);
      }
      serial_line_index = 0;
      return;
    }

    if (serial_line_index == 0 && (c == '1' || c == '0' || c == '2'))
    {
      handle_serial_byte(c);
      return;
    }

    if (serial_line_index < sizeof(serial_line_buffer) - 1)
    {
      serial_line_buffer[serial_line_index++] = c;
    }
    else
    {
      serial_line_index = 0;
    }
  }
}

int next_pending_risk_tag()
{
  for (int i = 0; i < MAX_TAG_ID; i++)
  {
    if (pending_risk_notify[i])
    {
      return i;
    }
  }

  return -1;
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

bool has_invalid_distance()
{
  for (int i = 0; i < ANCHOR_NO; i++)
  {
    if (distance_storage[i] == DIST_INVALID)
    {
      return true;
    }
  }

  return false;
}

uint32_t current_report_hash()
{
  uint32_t hash = 2166136261UL;

  for (int i = 0; i < ANCHOR_NO; i++)
  {
    hash ^= distance_storage[i];
    hash *= 16777619UL;
  }

  return hash;
}

bool should_print_final_report(byte tag_id)
{
  if (tag_id >= MAX_TAG_ID)
  {
    return true;
  }

  uint32_t report_hash = current_report_hash();
  if (report_hash == last_tag_report_hash[tag_id] &&
      millis() - last_tag_print[tag_id] < DUPLICATE_REPORT_SUPPRESS_MS)
  {
    return false;
  }

  if (has_invalid_distance())
  {
    last_tag_print[tag_id] = millis();
    last_tag_report_hash[tag_id] = report_hash;
    return true;
  }

  if (millis() - last_tag_print[tag_id] >= PRINT_INTERVAL)
  {
    last_tag_print[tag_id] = millis();
    last_tag_report_hash[tag_id] = report_hash;
    return true;
  }

  return false;
}

void loop()
{
  handle_serial_commands();

  if (MY_ID == 0)
  {
    if (!sentAck && !receivedAck)
    {
      handle_beacon_slot_timeout();
      finish_a0_discovery_if_needed();
      handle_a2_discovery_timeout();

      if (!beacon_outstanding && !is_discovery_busy() && !is_post_final_report_guard_active())
      {
        int riskTag = next_pending_risk_tag();
        if (riskTag >= 0)
        {
          pending_risk_notify[riskTag] = false;
          transmitRiskNotify((byte)riskTag);
          prev_send_beacon_millis = millis();
          return;
        }
      }

      if (!beacon_outstanding && !is_discovery_busy() && !is_post_final_report_guard_active() &&
          (millis() - prev_send_beacon_millis) > PRINT_INTERVAL)
      {
        prev_send_beacon_millis = millis();

        if (id_index == 0)
        {
          remove_idle_tag_list();
          update_active_tag_list();
        }

        if (id_index == active_tag_count)
        {
          if (should_send_scheduled_tag_request())
          {
            start_a0_discovery();
          }
        }
        else
        {
          int target_tag_id = active_tag_list[id_index];

          if (active_tag_heard_anchor[id_index] != 2)
          {
            send_beacon(target_tag_id);
          }
          else
          {
            send_beacon_delegate(active_tag_heard_anchor[id_index], target_tag_id);
          }

          mark_beacon_outstanding(target_tag_id);
        }

        id_index++;
        if (id_index > active_tag_count) id_index = 0;
      }
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

    if (msgId == A2_DISCOVERY_REPORT && data[16] == MY_ID)
    {
      handle_a2_discovery_report();
      receiver();
      prev_succeed_millis = millis();
      noteActivity();
      return;
    }

    if (msgId == A2_DISCOVERY_EMPTY && data[16] == MY_ID)
    {
      a2_discovery_waiting = false;
      receiver();
      prev_succeed_millis = millis();
      noteActivity();
      return;
    }

    if (msgId == ANCHOR_TAG_REPORT)
    {
      byte reporter_anchor_id = data[1];
      update_tag_status(current_tag_id, reporter_anchor_id);

      receiver();
      prev_succeed_millis = millis();
      noteActivity();
      return;
    }

    if (millis() < tag_id_request_millis + TAG_RESPONSE_DELAY && msgId == TAG_RESPONSE && data[16] == MY_ID)
    {
      prev_send_beacon_millis = millis();

      update_tag_status(data[17], MY_ID);
      a0_found_tag_this_discovery = true;
      prev_succeed_millis = millis();
      receiver();

      return;
    }

    if (msgId == POLL)
    {
      for (int i = 0; i < ANCHOR_NO; i++)
      {
        distance_storage[i] = ((uint16_t)data[1 + i * 2] << 8) | data[1 + i * 2 + 1];
      }
    }

    if (msgId == FINAL_REPORT || msgId == DELEGATED_FINAL_REPORT)
    {
      last_final_report_millis = millis();
      update_tag_status(data[17], msgId == DELEGATED_FINAL_REPORT ? data[15] : MY_ID);
      mark_tag_distance_received(data[17]);
      if (beacon_outstanding && outstanding_tag_id == data[17])
      {
        clear_beacon_outstanding();
      }
      prev_succeed_millis = millis();

      for (int i = 0; i < ANCHOR_NO; i++)
      {
        distance_storage[i] = ((uint16_t)data[1 + i * 2] << 8) | data[1 + i * 2 + 1];
      }

      uint16_t speedCentiKmh = ((uint16_t)data[9] << 8) | data[10];
      bool speedValid = data[11] == 1;

      if (should_print_final_report(current_tag_id))
      {
        Serial.print("T");
        Serial.print(current_tag_id);
        Serial.print(":");

        for (int i = 0; i < ANCHOR_NO; i++)
        {
          Serial.print(distance_storage[i]);
          Serial.print(i == ANCHOR_NO - 1 ? "" : ",");
        }

        Serial.println();

        Serial.print("S");
        Serial.print(current_tag_id);
        Serial.print(":");
        if (speedValid)
        {
          Serial.println(speedCentiKmh);
        }
        else
        {
          Serial.println("INVALID");
        }
      }

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
      update_tag_status(data[17], MY_ID);
      prev_succeed_millis = millis();

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

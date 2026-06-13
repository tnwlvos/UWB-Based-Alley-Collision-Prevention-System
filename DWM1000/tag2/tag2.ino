/**
   DW1000Ng Tag Main Code
   ?κ굅由??덉젙???ㅼ젙 諛섏쁺 踰꾩쟾
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
#define DIST_INVALID 65535
#define MAX_ANCHOR_RETRY 1

uint16_t distance_storage[ANCHOR_NO];
uint16_t last_good_distance[ANCHOR_NO];
uint32_t last_good_distance_millis[ANCHOR_NO];
int other_tag_id = 0;
uint16_t other_tag_dist = 0;
int current_target_id = 0;
int anchor_retry_count = 0;
bool ranging_in_progress = false;
bool tag_registered = false;
uint32_t last_registered_activity = 0;
uint32_t last_tag_response_millis = 0;

volatile byte expectedMsgId = POLL_ACK;
volatile boolean sentAck = false;
volatile boolean receivedAck = false;

uint64_t timePollSent, timePollAckReceived, timeRangeSent;

#define LEN_DATA 20
const uint8_t SPEED_RX_BUFFER_SIZE = 16;
const uint32_t SPEED_STALE_MS = 1000;
byte data[LEN_DATA];

char speed_rx_buffer[SPEED_RX_BUFFER_SIZE];
uint8_t speed_rx_index = 0;
float current_gps_speed_kmh = 0.0;
uint16_t current_gps_speed_centi_kmh = 0;
uint32_t last_speed_update_millis = 0;
bool gps_speed_valid = false;

uint32_t lastActivity;
uint32_t resetPeriod = 60;        // anchor timeout before retrying or moving to the next anchor
uint16_t replyDelayTimeUS = 8000;

uint32_t last_data_update_time = 0;
const uint32_t HANG_TIMEOUT = 12000;   // 湲곗〈 2000 ??5000
const uint32_t TAG_REGISTER_TIMEOUT = 4000;
const uint32_t TAG_RESPONSE_MIN_INTERVAL = 300;
const uint32_t STALE_DISTANCE_HOLD_MS = 1000;
const uint32_t FINAL_REPORT_REPEAT_DELAY_MS = 15;

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

void noteActivity()
{
  lastActivity = millis();
}

void clearSpeedRxBuffer()
{
  speed_rx_index = 0;
  speed_rx_buffer[0] = '\0';
}

void parseSpeedLine(char *line)
{
  if (line[0] == '\0')
  {
    return;
  }

  char *endPtr;
  float parsedSpeed = strtof(line, &endPtr);

  while (*endPtr == ' ' || *endPtr == '\t')
  {
    endPtr++;
  }

  if (endPtr == line || *endPtr != '\0' || parsedSpeed < 0.0 || parsedSpeed > 300.0)
  {
    return;
  }

  float speedCenti = parsedSpeed * 100.0 + 0.5;
  if (speedCenti > 65535.0)
  {
    speedCenti = 65535.0;
  }

  current_gps_speed_kmh = parsedSpeed;
  current_gps_speed_centi_kmh = (uint16_t)speedCenti;
  last_speed_update_millis = millis();
  gps_speed_valid = true;
}

void readSpeedSerialNonBlocking()
{
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();

    if (c == '\n')
    {
      speed_rx_buffer[speed_rx_index] = '\0';
      parseSpeedLine(speed_rx_buffer);
      clearSpeedRxBuffer();
    }
    else if (c != '\r')
    {
      if (speed_rx_index < SPEED_RX_BUFFER_SIZE - 1)
      {
        speed_rx_buffer[speed_rx_index++] = c;
      }
      else
      {
        clearSpeedRxBuffer();
      }
    }
  }

  if (gps_speed_valid && millis() - last_speed_update_millis > SPEED_STALE_MS)
  {
    gps_speed_valid = false;
    current_gps_speed_kmh = 0.0;
    current_gps_speed_centi_kmh = 0;
  }
}

uint16_t getCurrentSpeedForReport()
{
  if (!gps_speed_valid)
  {
    return 0;
  }

  return current_gps_speed_centi_kmh;
}

void transmitTagNo(byte requestAnchorId)
{
  data[0] = TAG_RESPONSE;
  data[16] = requestAnchorId;
  data[17] = MY_TAG_ID;

  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
  last_tag_response_millis = millis();
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

  DW1000Ng::forceTRxOff();
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}

void transmitFinalReport()
{
  data[0] = FINAL_REPORT;

  for (int i = 0; i < ANCHOR_NO; i++)
  {
    data[1 + i * 2] = distance_storage[i] >> 8;
    data[1 + i * 2 + 1] = distance_storage[i] & 0xFF;
  }

  
  uint16_t speedCentiKmh = getCurrentSpeedForReport();
  data[9] = speedCentiKmh >> 8;
  data[10] = speedCentiKmh & 0xFF;
  data[11] = gps_speed_valid ? 1 : 0;

// Anchor0?먭쾶 蹂대궡??理쒖쥌 嫄곕━ 由ы룷??
  data[16] = 0;
  data[17] = MY_TAG_ID;

  DW1000Ng::forceTRxOff();
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}

void fillRecentLostDistances()
{
  for (int i = 0; i < ANCHOR_NO; i++)
  {
    if (distance_storage[i] == DIST_INVALID &&
        last_good_distance[i] != DIST_INVALID &&
        millis() - last_good_distance_millis[i] <= STALE_DISTANCE_HOLD_MS)
    {
      distance_storage[i] = last_good_distance[i];
    }
  }
}

void transmitRange()
{
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

void resetDistanceStorage()
{
  for (int i = 0; i < ANCHOR_NO; i++)
  {
    distance_storage[i] = DIST_INVALID;
  }
}

void finishRangeCycle()
{
  fillRecentLostDistances();
  transmitFinalReport();
  delay(FINAL_REPORT_REPEAT_DELAY_MS);
  transmitFinalReport();

  current_target_id = 0;
  anchor_retry_count = 0;
  expectedMsgId = POLL_ACK;
  ranging_in_progress = false;
  tag_registered = true;
  last_registered_activity = millis();
  last_data_update_time = millis();
}

void advanceToNextAnchor()
{
  expectedMsgId = POLL_ACK;
  anchor_retry_count = 0;
  current_target_id++;

  if (current_target_id < ANCHOR_NO)
  {
    transmitPoll();
    noteActivity();
  }
  else
  {
    finishRangeCycle();
  }
}

void skipCurrentAnchor()
{
  if (current_target_id < ANCHOR_NO)
  {
    if (anchor_retry_count < MAX_ANCHOR_RETRY)
    {
      anchor_retry_count++;
      expectedMsgId = POLL_ACK;
      transmitPoll();
      noteActivity();
      return;
    }

    distance_storage[current_target_id] = DIST_INVALID;
    anchor_retry_count = 0;
    last_data_update_time = millis();
  }

  advanceToNextAnchor();
}

void receiver()
{
  DW1000Ng::forceTRxOff();
  DW1000Ng::startReceive();
}

void dwm1000_process()
{
  if (sentAck)
  {
    sentAck = false;
    receiver();
  }

  if (ranging_in_progress && !sentAck && !receivedAck && millis() - lastActivity > resetPeriod)
  {
    skipCurrentAnchor();
    return;
  }

  if (receivedAck)
  {
    receivedAck = false;
    DW1000Ng::getReceivedData(data, LEN_DATA);

    if (data[0] == BEACON)
    {
      if (data[1] == MY_TAG_ID)
      {
        if (!ranging_in_progress)
        {
          current_target_id = 0;
          anchor_retry_count = 0;
          expectedMsgId = POLL_ACK;
          ranging_in_progress = true;
          tag_registered = true;
          last_registered_activity = millis();
          resetDistanceStorage();

          DW1000Ng::forceTRxOff();
          transmitPoll();
          noteActivity();
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

      return;
    }

    if (data[0] == WARNING)
    {
      warning_millis = millis() + 500;
      noteActivity();
      receiver();
      return;
    }

    if (data[0] == TAG_REQUEST)
    {
      byte request_anchor_id = data[16];
      if (tag_registered && millis() - last_registered_activity > TAG_REGISTER_TIMEOUT)
      {
        tag_registered = false;
      }

      if (ranging_in_progress || tag_registered)
      {
        receiver();
        return;
      }

      if (millis() - last_tag_response_millis < TAG_RESPONSE_MIN_INTERVAL)
      {
        receiver();
        return;
      }

      delay(random(0, 30));
      DW1000Ng::forceTRxOff();
      transmitTagNo(request_anchor_id);

      noteActivity();
      return;
    }

    if (data[17] != MY_TAG_ID)
    {
      receiver();
      return;
    }

    if (data[0] == POLL_ACK && expectedMsgId == POLL_ACK && data[16] == current_target_id)
    {
      timePollSent = DW1000Ng::getTransmitTimestamp();
      timePollAckReceived = DW1000Ng::getReceiveTimestamp();

      expectedMsgId = RANGE_REPORT;
      transmitRange();
    }
    else if (data[0] == RANGE_REPORT && expectedMsgId == RANGE_REPORT && data[16] == current_target_id)
    {
      float curRange;
      memcpy(&curRange, data + 1, 4);

      if (curRange > 0 && curRange < 100.0 && data[16] < ANCHOR_NO)
      {
        distance_storage[data[16]] = (uint16_t)min(curRange * 1000.0, 65535.0);
        last_good_distance[data[16]] = distance_storage[data[16]];
        last_good_distance_millis[data[16]] = millis();
        anchor_retry_count = 0;
        last_data_update_time = millis();
      }
      else if (data[16] < ANCHOR_NO)
      {
        distance_storage[data[16]] = DIST_INVALID;
      }

      other_tag_id = data[17];
      other_tag_dist = ((uint16_t)data[18] << 8) | data[19];

      advanceToNextAnchor();
    }
    else
    {
      receiver();
    }

    noteActivity();
  }
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

  last_data_update_time = millis();
  for (int i = 0; i < ANCHOR_NO; i++)
  {
    last_good_distance[i] = DIST_INVALID;
    last_good_distance_millis[i] = 0;
  }

  receiver();
  noteActivity();
}

void loop()
{
  readSpeedSerialNonBlocking();

  if (warning_millis > millis())
  {
    digitalWrite(26, (millis() / 500) % 2);
  }
  else
  {
    digitalWrite(26, LOW);
  }

  if (millis() - last_data_update_time > HANG_TIMEOUT)
  {
    delay(100);
    ESP.restart();
  }

  dwm1000_process();
}

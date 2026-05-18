/**
 * setting.h
 * 각 앵커 보드별 고유 설정 파일
 */
#ifndef SETTING_H
#define SETTING_H

#include <Arduino.h>

// 보드에 따라 0, 1, 2, 3으로 수정하여 업로드하세요.
#define MY_ID 0 


// [유지] 기존 하드웨어 핀 설정 그대로 유지
const uint8_t PIN_RST = 15;
const uint8_t PIN_IRQ = 17;
const uint8_t PIN_SS = 4;

#endif

/**
 * setting.h
 * 각 태그 보드별 고유 설정 파일
 */
#ifndef SETTING_H
#define SETTING_H

// 각 보드에 맞게 수정 (예: 1 또는 22)
#define MY_TAG_ID 56

// 하드웨어 핀 설정 (보드가 동일하다면 수정 불필요)
const uint8_t PIN_RST = 15;
const uint8_t PIN_IRQ = 17;
const uint8_t PIN_SS = 4;

#endif

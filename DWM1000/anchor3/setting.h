/**
 * setting.h
 * 각 앵커 보드별 고유 설정 파일
 */
#ifndef SETTING_H
#define SETTING_H

// 보드에 따라 0, 1, 2, 3으로 수정하여 업로드하세요.
#define MY_ID 3

// 기타 하드웨어 핀 설정 (변경이 필요한 경우에만 수정)
const uint8_t PIN_RST = 15;
const uint8_t PIN_IRQ = 17;
const uint8_t PIN_SS = 4;

#endif

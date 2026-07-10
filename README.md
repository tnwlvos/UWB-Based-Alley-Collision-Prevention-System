# UWB-Based Alley Collision Prevention System

UWB 기반 십자형 교차로 및 골목 개인형 이동장치(PM) 측면 충돌 예방 시스템입니다.  
차량과 PM이 서로 시야 확보가 어려운 골목 및 교차로 환경에서 UWB 거리 측정과 속도 데이터를 기반으로 충돌 위험을 판단하고, 위험 단계에 따라 경고를 제공하는 것을 목표로 개발하였습니다.

## Project Overview

본 프로젝트는 UWB Anchor-Tag 구조를 이용하여 PM과 차량의 상대 위치를 추정하고, Android 앱에서 수집한 GPS 속도 데이터를 함께 활용하여 TTC(Time To Collision) 기반 위험 판단을 수행합니다.

다중 Anchor와 다중 Tag가 동시에 동작하는 환경에서 패킷 유실을 줄이면서도 불필요한 통신 지연이 발생하지 않도록 통신 순서와 메시지 구조를 설계하는 데 중점을 두었습니다.

## Main Features

- DWM1000 기반 UWB 거리 측정
- 다중 Anchor-Tag 기반 거리 측정 구조
- Main Anchor의 Active Tag List 관리
- 태그별 거리 측정 요청을 위한 Round-Robin Scheduling
- Slave Anchor 기반 Tag 응답 및 Final Report Relay 구조
- Android 앱 기반 GPS 속도 데이터 수집
- UWB 거리 + GPS 속도 기반 TTC 위험 판단
- 위험 단계별 LED, Buzzer, Android 앱 경고 출력
- PC Processing 기반 실시간 시각화 및 상태 모니터링

## System Architecture

- PC Processing
  - Anchor0와 UART Serial 통신
  - 거리 데이터 수신
  - 위치, 속도, TTC 기반 위험 단계 계산
  - 위험 단계 명령 전송

- Anchor0, Main Anchor
  - Active Tag List 관리
  - Round-Robin 방식 Tag별 거리 측정 요청
  - Slave Anchor와 Delegated Ranging 수행
  - 최종 거리 데이터를 PC로 전달

- Anchor1, Anchor2, Anchor3, Slave Anchor
  - Tag 탐색 보조
  - Delegated Ranging 수행
  - Tag 응답 및 Final Report Relay

- Tag
  - Anchor와 UWB 거리 측정
  - Android 앱으로부터 GPS 속도 수신
  - 거리 배열 및 속도 정보 전송
  - 위험 단계 수신 후 LED/Buzzer/App에 전달

- Android App
  - GPS 기반 속도 측정
  - USB Serial 통신으로 Tag에 속도 데이터 전달

## Communication Design

본 프로젝트에서는 통신 안정성과 지연 시간 사이의 균형을 고려했습니다.

메시지 재전송을 늘리면 패킷 유실은 줄어들지만 전체 응답 지연이 증가하고, 반대로 지연을 줄이면 일부 거리 데이터가 누락될 수 있습니다. 이를 완화하기 위해 Main Anchor가 Active Tag List를 관리하고, Tag별 거리 측정 요청을 Round-Robin 방식으로 수행하도록 구성했습니다.

또한 직접 통신이 어려운 경우 Slave Anchor가 Tag 응답 및 Final Report를 Main Anchor로 Relay하도록 구성하여 통신 가능 범위를 보완했습니다.

## Repository Structure

- DWM1000/
  - UWB Anchor 및 Tag 펌웨어 코드

- Tag_Application/
  - Android 기반 GPS 속도 측정 및 USB Serial 통신 앱

- processing/
  - PC 기반 실시간 시각화 및 위험 판단 프로그램

- client/
  - 통신 테스트 및 클라이언트 코드

- server/
  - 통신 테스트 및 서버 코드

## Tech Stack

- C/C++
- Processing
- Kotlin
- Android Studio
- Arduino IDE
- DWM1000
- ESP32
- UART Serial
- UWB Ranging
- TTC Algorithm

## Key Implementation Points

- UWB 기반 거리 측정 데이터 수집
- 다중 Anchor-Tag 통신 프로토콜 설계
- Active Tag List 기반 Tag 관리
- Round-Robin Scheduling 기반 거리 측정 요청
- Slave Anchor Relay 구조를 통한 통신 안정성 보완
- Android GPS 속도 데이터와 UWB 거리 데이터 결합
- TTC 기반 위험 단계 판단 알고리즘 구현

## Result

본 프로젝트를 통해 다중 노드 UWB 통신 구조, 실시간 거리 데이터 처리, GPS 속도 연동, TTC 기반 위험 판단, 경고 출력까지 이어지는 임베디드 안전 시스템 구조를 구현하였습니다.

특히 통신 안정성과 실시간성 사이의 균형을 고려하여 메시지 흐름과 노드별 역할을 설계했다는 점에서 의미가 있습니다.

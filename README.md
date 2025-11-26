# The_SSENs_FLIR_Seeker

## 📌 프로젝트 개요
The_SSENs_FLIR_Seeker는 ****적외선 표적 추적 시스템****을 모사하는 프로젝트입니다.  
본 프로젝트는 실제 미사일 시커(seeker)의 기능을 학습/실험하는 것을 목표로 하며, 임베디드 보드(Zynq, STM32 등)와 열화상 센서를 기반으로 동작합니다.

---

## 프로젝트 목표
- 가시광/적외선 영상 획득 및 송수신 구현
- 중기 유도 단계에서 가시광 영상 기반 영상 대조 항법 구현
- 종말 유도 단계에서 적외선 영상 기반 표적 추적 시스템 구현
- 종말 유도 단계에서 운용자가 표적을 지정할 수 있는 맨 인더 루프(Man In the Loop) 시스템 구현
- 표적 정보를 바탕으로 유도 명령 생성하고 구동부를 제어하여 유도조종기능 구현
- 미사일 운용에 필요한 기능과 상호작용 가능한 운용자 인터페이스 구현

---
## 프로젝트 운용 개념도
<img src="./image/운용 개념도.png">

---

## 📂 디렉토리 구조

```
The_SSENs_FLIR_Seeker/
│
│
├── src/ # 주요 소스 코드
│ ├── firmware/ # 펌웨어 (임베디드 보드용, 예: STM32, Zynq)
│ ├── host/ # 호스트 프로그램 (예: PC/Linux)
│ └── sim/ # 시뮬레이션 코드 (알고리즘 테스트)
│
├── hardware/ # 회로도, PCB 파일, 센서 연결 정보
│ └── schematics/
│
├── data/ # 테스트 데이터셋 (열화상 이미지, 로그 등)
│ └── sample/
│
├── scripts/ # 빌드, 분석, 자동화 스크립트
│
└── README.md # 프로젝트 설명 문서 (현재 문서)
```

---

## 🚀 주요 기능
- 열화상 카메라(FLIR Lepton 기반) 데이터 수집
- 임베디드 보드 상 실시간 처리 (Zynq + Petalinux)
- 타겟 탐색 및 추적 알고리즘
- 시뮬레이션 환경 제공 (C# WPF 기반)

---

## 🛠 개발 환경
- **임베디드 보드**: Zynq 7010, Arduino(atmega 328p)
- **센서**: FLIR Lepton 1.6
- **OS**: Petalinux
- **언어**: C, C++, C#
- **도구**: Vivado, Petalinux(OpenCV, gstreamer), Arduino IDE, WPF

---

## 👥 팀 정보
- **팀명**: The SSENs
- **프로젝트명**: 적외선 표적 추적 시스템 개발

📌 역할 분담

| 이름   | 역할                       | GitHub | 이메일 |
|--------|----------------------------|--------|--------|
| 신규현 | OpenCV 표적탐지            | [xiingyu](https://github.com/xiingyu) | tls3162@gmail.com |
| 정채우 | Linux 커널 드라이버 개발    | [drei2898](https://github.com/drei2898) | drei99816@gmail.com |
| 이동현 | WPF                        | [MerkavaV](https://github.com/MerkavaV) | 990503ldh@gmail.com |
| 한정민 | gstreamer, UDP 송수신 파이프라인 | [Sarahhan-one](https://github.com/Sarahhan-one) | sarahhan.one@gmail.com |
| 윤상훈 | OpenCV 표적탐지            | [topsanghoon](https://github.com/topsanghoon) | topsanghoon1@gmail.com |
| 김민준 | OpenCV 기반 전처리         | [hubo416](https://github.com/hubo416) | hubo0416@gmail.com |

---

## 📌 라이선스
(추후 라이선스 선택 예정, 예: MIT, GPL)

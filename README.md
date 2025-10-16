# The_SSENs_FLIR_Seeker

## 📌 프로젝트 개요
The_SSENs_FLIR_Seeker는 **FLIR(Forward Looking Infrared) 기반 탐색 및 추적 시스템**을 모사하는 프로젝트입니다.  
본 프로젝트는 실제 미사일 시커(seeker)의 기능을 학습/실험하는 것을 목표로 하며, 임베디드 보드(Zynq, STM32 등)와 열화상 센서를 기반으로 동작합니다.

---

## 📂 디렉토리 구조

```
The_SSENs_FLIR_Seeker/
│
├── docs/ # 문서, 기술 설명, 회의록
│ └── README.md
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
- 임베디드 보드 상 실시간 처리 (Zynq + RTOS / Linux)
- 타겟 탐색 및 추적 알고리즘
- 시뮬레이션 환경 제공 (Python/C++ 기반)

---

## 🛠 개발 환경
- **임베디드 보드**: Zynq 7010, STM32F4 계열
- **센서**: FLIR Lepton 3.0 (8Hz)
- **OS**: FreeRTOS, Embedded Linux (PetaLinux)
- **언어**: C, C++, Python
- **도구**: Vivado, CubeMX, OpenCV, QGIS (지형 데이터)

---

## 📜 진행 계획
1. 센서 데이터 획득 및 시각화
2. 간단한 추적 알고리즘 구현 (Python 기반)
3. 임베디드 보드 이식 (Zynq)
4. 실시간 처리 최적화
5. 전체 시스템 통합 및 시뮬레이션

---

## 👥 팀 정보
- **팀명**: The SSEN
- **프로젝트명**: FLIR Seeker

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

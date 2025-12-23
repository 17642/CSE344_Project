# CSE344: TinyML Gesture Remote & IoT Control System
**TinyML 기반 제스처 인식 및 저전력 통신(BLE) 최적화 제어 시스템**

본 프로젝트는 IMU 센서 데이터를 활용한 TinyML 제스처 인식 시스템과, 이를 통해 원격 기기를 제어하는 IoT 게이트웨이 시스템의 통합 구현을 목표로 합니다. 제한된 임베디드 자원 환경에서 통신 효율성과 안정성을 확보하기 위한 커스텀 프로토콜 설계를 포함합니다.

## 1. 프로젝트 개요
- **개발 기간**: 2025.12.06 - 2025.12.15 (지능형 IoT 응용 과제)
- **개발 인원**: 1인
- **핵심 목표**: 
    - IMU 기반 4축 제스처(상/하/좌/우) 인식 및 조합 명령 체계 구축
    - BLE/Serial 환경에서의 패킷 최적화 및 연결 신뢰성 확보
    - 후속 학습을 위한 데이터 증강(Augmentation) 및 자동화 파이프라인 설계

## 2. 주요 기술적 매커니즘

### 2.1 TinyML 기반 제스처 인식 및 조합 (Gesture Engine)
- **추론 모델**: IMU 센서 데이터를 입력으로 하여 상/하/좌/우 기본 제스처를 분류하는 신경망 모델을 디바이스 내부에 탑재(In-device Inference).
- **가변 명령 조합**: 단일 제스처에 그치지 않고, 인식된 제스처의 시퀀스를 조합하여 복합 명령어 블록을 생성하는 로직 구현.
- **데이터 파이프라인**: 학습 데이터 수집 시 JSON 포맷을 채택하여 데이터 인덱싱을 자동화하고, 학습 효율을 높이기 위한 데이터 증강(Augmentation) 스크립트 포함.

### 2.2 고효율 통신 프로토콜 설계
BLE의 제한된 페이로드 크기와 불안정한 연결 환경을 극복하기 위해 하드웨어 레벨의 통신 최적화를 수행했습니다.

- **명령어 해싱 (FNV-1a 기반)**:
    - 전체 명령어 문자열 대신 **FNV-1a 알고리즘**을 적용한 해시값만을 전송하여 패킷 오버헤드를 최소화.
    - 비트 마스크(Bitmask)를 결합하여 단일 패킷 내에서 명령어 검증 및 파라미터 전달을 동시에 수행.
- **연결 복구 매커니즘 (Robustness)**:
    - BLE 특유의 연결 불안정성에 대응하여, 통신 단절 시 상태 머신을 기반으로 자동 재연결(Reconnection) 및 세션 복구를 시도하는 예외 처리 로직 구현.

### 2.3 통합 제어 인터페이스
- **멀티 프로토콜 게이트웨이**: PC와 디바이스 간의 BLE 통신을 수신하여 시리얼(Serial) 명령으로 변환, 하위 타겟 기기에 전달하는 브릿지 기능 수행.
- **동적 명령 매핑**: Serial 통신을 통해 실시간으로 특정 제스처 조합에 할당된 명령어를 변경할 수 있는 가변 제어 체계 구축.

## 3. 기술 스택
- **Embedded**: C/C++ (Arduino/Embedded C), TinyML (TensorFlow Lite Micro)
- **Software**: Python (Bleak, pySerial), Jupyter Notebook (Model Training/Analysis)
- **Communication**: Bluetooth Low Energy (BLE), UART/Serial
- **Data**: JSON, NumPy (Data Processing)

<img width="637" height="361" alt="image" src="https://github.com/user-attachments/assets/d28551b6-f9df-4df7-8818-51c62b99e9d4" />
<img width="361" height="637" alt="image" src="https://github.com/user-attachments/assets/7ebc9bb6-7f10-461f-abdc-30627b116fee" />

[![Video Label](http://img.youtube.com/vi/iwWny2CQvXQ/0.jpg)](https://youtu.be/iwWny2CQvXQ)

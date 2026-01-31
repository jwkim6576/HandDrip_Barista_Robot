# ☕ Hand-drip Barista Robot (Doosan M0609)

<img width="2027" height="1041" alt="image" src="https://github.com/user-attachments/assets/7cb9bf21-05b0-4bc8-9f77-58d187e2faeb" />


<br>

## 🗂️ 목차

### 1. [Project Overview](#-project-overview)
### 2. [Team & Roles](#-team--roles)
### 3. [System Architecture](#-system-architecture)
### 4. [Tech Stack](#-tech-stack)
### 5. [Execution & Usage](#-execution--usage)
### 6. [Key Features & Logic](#-key-features--logic)
### 7. [Safety & Recovery System](#️-safety--recovery-system)
### 8. [Demo Video](#-demo-video)

<br>

---

## 🔍 Project Overview
**"핸드드립 커피의 맛을 언제나 일정하게, 피크타임에도 효율적으로."**

본 프로젝트는 두산로보틱스 협동로봇(M0609)을 활용하여 **핸드드립 커피 제조 공정을 자동화**한 시스템입니다.
Web UI를 통해 고객이 메뉴(Coffee/Cocoa)와 온도(Hot/Ice)를 선택하면, 로봇이 **Firebase 실시간 DB**를 통해 주문을 수신하고, 숙련된 바리스타의 **나선형 드립(Spiral Pouring)** 모션을 모사하여 커피를 추출합니다. 특히, 작업 중 외력이나 비상 정지가 발생하더라도 **작업을 처음부터 다시 하는 것이 아니라, 중단된 시점부터 재개(Resume)** 할 수 있는 고도화된 복구 시스템을 갖추고 있습니다.

<br>

## 👥 Team & Roles

| Name | Role | Responsibility |
|:---:|:---:|:---|
| **Kim Jung-wook** | **System Architect** <br> **& Integration** | - **System Architecture:** Web UI - Firebase - ROS2 간의 전체 통신 구조 설계 <br> - **Firebase Integration:** 실시간 DB 연동을 위한 ROS2 Listener 노드 개발 <br> - **Node Integration:** 제어(Control) 및 모니터링 노드 통합 및 최적화 |
| **Lee Kang-yeop** | Motion Control <br> & Safety | - **Robot Motion:** 드립, 그리핑 등 주요 로봇 모션 제어 알고리즘 구현 <br> - **Safety Logic:** 스마트 래핑(Smart Wrapping) 함수 기반의 안전/복구 로직 설계 |
| **Kim Da-bin** | UI & Logic Dev | - **Scenario Logic:** 메뉴별(Ice/Hot, 토핑) 분기 처리 및 공정 세부화 <br> - **Documentation:** 발표 자료 작성 및 시연 시나리오 기획 |
| **Kang Dong-hyuk** | Backend Support | - **Firebase Setup:** 클라우드 DB 환경 구축 및 초기 연동 테스트 |

<br>

## 🛠 System Architecture

<img width="2101" height="1085" alt="image" src="https://github.com/user-attachments/assets/76fb035a-f830-4eb1-830a-075e198f2688" />
<img width="1748" height="1047" alt="image" src="https://github.com/user-attachments/assets/a4742ab9-6b6e-49c4-8bbe-5a0faec30adf" />
<img width="1552" height="1078" alt="image" src="https://github.com/user-attachments/assets/73aac713-89e8-4fd6-a5ba-fa7f726b46f5" />
<img width="2074" height="1058" alt="image" src="https://github.com/user-attachments/assets/cfcce6a6-ac86-4b79-811b-4dadd5625414" />

본 시스템은 **사용자(Web)**, **클라우드(Firebase)**, **로봇 제어(ROS2)** 세 부분으로 구성됩니다.

1.  **Web Order (Client):** HTML/CSS/JS 기반의 웹페이지에서 고객이 메뉴를 주문하면 `Firebase Realtime DB`에 주문 정보(Order Type, Temp)가 업데이트됩니다.
2.  **Middleware (ROS2 Control Core):**
    * **Firebase Listener:** DB 변경 사항을 0.1초 단위로 감지하여 `ros2 service`를 호출합니다.
    * **DSR Control Function:** 로봇의 실제 움직임(MoveJ, MoveL)을 관장하며, 충돌을 방지합니다.
3.  **Robot Hardware:** 두산 M0609 로봇과 OnRobot RG2 그리퍼가 물리적인 제조 공정을 수행합니다.

<br>

## 💻 Tech Stack

| Category | Technology |
| :---: | :--- |
| **Hardware** | ![Doosan](https://img.shields.io/badge/Doosan_Robotics-M0609-005EB8?style=flat-square) ![Gripper](https://img.shields.io/badge/OnRobot-RG2-gray?style=flat-square) |
| **OS / Middleware** | ![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?style=flat-square&logo=ubuntu) ![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?style=flat-square&logo=ros) |
| **Backend / DB** | ![Firebase](https://img.shields.io/badge/Firebase-Realtime_DB-FFCA28?style=flat-square&logo=firebase) ![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=flat-square&logo=python) |
| **Frontend** | ![HTML5](https://img.shields.io/badge/HTML5-E34F26?style=flat-square&logo=html5) ![CSS3](https://img.shields.io/badge/CSS3-1572B6?style=flat-square&logo=css3) ![JS](https://img.shields.io/badge/JavaScript-F7DF1E?style=flat-square&logo=javascript) |

<br>

## ⚡ Execution & Usage

터미널을 순서대로 열어 아래 명령어들을 실행해 주세요.

### 1. 로봇 드라이버 및 시각화 (Launch)
실제 로봇(M0609)과 통신을 시작하고 제어 환경을 활성화합니다.

ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609


### 2. 로봇 동작 제어 노드 실행 (Controller)
Firebase로부터 명령을 수신하여 로봇의 핸드드립 동작을 제어합니다.

ros2 run dsr_rokey2 real_god_listener


### 3. 데이터 퍼블리셔 실행 (Publisher)
로봇의 실시간 상태 데이터(관절 각도 등)를 Firebase로 전송합니다.

ros2 run dsr_rokey2 firebase_publisher


### 4. 웹 UI 서버 구동 (Web Server)
사용자 주문 및 모니터링을 위한 웹 페이지를 로컬 서버로 띄웁니다. (`index.html`이 있는 폴더에서 실행)

python3 -m http.server 8000

* **접속 URL:** `http://localhost:8000`

### 🔧 유틸리티 명령어 (Direct Teaching)
로봇의 위치를 수동으로 조정하거나 직접교시 모드를 사용해야 할 때 유용한 명령어입니다.
직접교시 모드 활성화 (Manual Mode)

ros2 service call /dsr01/system/set_robot_mode dsr_msgs2/srv/SetRobotMode "{robot_mode: 0}"

자동 운전 모드 전환 (Auto Mode)

ros2 service call /dsr01/system/set_robot_mode dsr_msgs2/srv/SetRobotMode "{robot_mode: 1}"


<br>

## 🚀 Key Features & Logic

### 1. Real-time Order & Monitoring System
별도의 키오스크 장비 없이 웹페이지를 통해 주문을 받고, 로봇의 현재 상태(준비 중, 드립 중, 완료 등)를 웹 화면에 실시간 프로그레스 바(Progress Bar)로 시각화했습니다.

<img width="1991" height="1028" alt="image" src="https://github.com/user-attachments/assets/8d8d4d7a-f96a-4ce8-868f-43c04b1a9938" />


### 2. Hand-drip Algorithm (Spiral Motion)
균일한 맛을 내기 위해 바리스타의 기술인 '나선형 드립(Spiral Pouring)'을 로봇 모션으로 구현했습니다.
* **Step 1:** 필터 린싱 및 원두 도징 (Pick & Place)
* **Step 2:** 불림(Blooming) 및 1~3차 나선형 주수
* **Step 3:** 드리퍼 제거 및 서빙 (Ice/Hot 분기 처리)

<br>

## 🛡️ Safety & Recovery System

현장에서 가장 중요한 **'작업 연속성'**을 보장하기 위해 **Smart Wrapping** 기술을 적용했습니다.

<img width="2049" height="1064" alt="image" src="https://github.com/user-attachments/assets/2d670209-8ec3-42f7-9f8c-9dee6f4224a1" />


* **Problem:** 기존 로봇 시스템은 비상 정지(E-Stop) 후 재가동 시, 처음부터 작업을 다시 시작해야 해서 재료 낭비와 시간 손실이 발생했습니다.
* **Solution:** 모든 로봇 이동 함수에 `check_stop_signal()`과 `save_step_index()` 로직을 래핑(Wrapping)했습니다.
    * 로봇이 멈춘 정확한 공정 단계(Step Index)를 기억합니다.
    * 복구 명령(Resume) 수신 시, **중단된 그 지점부터** 즉시 작업을 재개합니다.
    * 협동 로봇의 장점을 살려 외력 충돌 감지 시 자동으로 작업을 일시 정지하고 대기합니다.

<br>

## 🎥 Demo Video

[https://youtu.be/9yftWRMPXNc](https://youtu.be/9yftWRMPXNc)

[https://youtu.be/R9iazbjtELI](https://youtu.be/R9iazbjtELI)

[https://youtu.be/fxtUGFrPTQ0](https://youtu.be/fxtUGFrPTQ0)

[https://youtu.be/fL8Ub9j5NYI](https://youtu.be/fL8Ub9j5NYI)

[https://youtu.be/-GVCgUJVy90](https://youtu.be/-GVCgUJVy90)

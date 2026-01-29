# ☕ Hand-drip Barista Robot (Doosan M0609)

![Project Banner](https://placeholder.com/wp-content/uploads/2018/10/placeholder.com-logo1.png)

<br>

## 🗂️ 목차

### 1. [Project Overview](#-project-overview)
### 2. [Team & Roles](#-team--roles)
### 3. [System Architecture](#-system-architecture)
### 4. [Tech Stack](#-tech-stack)
### 5. [Key Features & Logic](#-key-features--logic)
### 6. [Safety & Recovery System](#-safety--recovery-system)
### 7. [Demo Video](#-demo-video)

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

> **[여기에 PPT 9페이지의 '시스템 아키텍처' 다이어그램을 넣으세요]**

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

## 🚀 Key Features & Logic

### 1. Real-time Order & Monitoring System
별도의 키오스크 장비 없이 웹페이지를 통해 주문을 받고, 로봇의 현재 상태(준비 중, 드립 중, 완료 등)를 웹 화면에 실시간 프로그레스 바(Progress Bar)로 시각화했습니다.
> **[여기에 PPT 23페이지의 '웹 주문 화면' UI 이미지를 넣으세요]**

### 2. Hand-drip Algorithm (Spiral Motion)
균일한 맛을 내기 위해 바리스타의 기술인 '나선형 드립(Spiral Pouring)'을 로봇 모션으로 구현했습니다.
* **Step 1:** 필터 린싱 및 원두 도징 (Pick & Place)
* **Step 2:** 불림(Blooming) 및 1~3차 나선형 주수
* **Step 3:** 드리퍼 제거 및 서빙 (Ice/Hot 분기 처리)

<br>

## 🛡️ Safety & Recovery System (Core Tech)

현장에서 가장 중요한 **'작업 연속성'**을 보장하기 위해 **Smart Wrapping** 기술을 적용했습니다.

> **[여기에 PPT 26페이지의 '복구 알고리즘' 코드나 표 이미지를 넣으세요]**

* **Problem:** 기존 로봇 시스템은 비상 정지(E-Stop) 후 재가동 시, 처음부터 작업을 다시 시작해야 해서 재료 낭비와 시간 손실이 발생했습니다.
* **Solution:** 모든 로봇 이동 함수에 `check_stop_signal()`과 `save_step_index()` 로직을 래핑(Wrapping)했습니다.
    * 로봇이 멈춘 정확한 공정 단계(Step Index)를 기억합니다.
    * 복구 명령(Resume) 수신 시, **중단된 그 지점부터** 즉시 작업을 재개합니다.
    * 협동 로봇의 장점을 살려 외력 충돌 감지 시 자동으로 작업을 일시 정지하고 대기합니다.

<br>

## 🎥 Demo Video

> **[여기에 시연 영상 GIF나 유튜브 링크를 넣으세요]**

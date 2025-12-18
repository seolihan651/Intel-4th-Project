

---

### 추천 `README.md` (복사해서 사용하세요)

```markdown
# 🐢 Intel 8th Gen - 4th Team Project

인텔 에지 AI SW 아카데미 8기, 4번째 팀 프로젝트 저장소입니다.  
ROS2 기반의 TurtleBot 제어 및 Qt GUI, 그리고 별도의 통신 모듈을 포함하고 있습니다.

## 📂 디렉토리 구조 (Directory Structure)

```text
Intel-4th-Project
├── 📂 comm_pkg                   # 비 ROS2 통신망 소스코드 (Non-ROS2 Codes)
└── 📂 ros2_turtlebot_createdpkg  # ROS2 패키지 모음
    ├── 📂 relay_bot_pkg          # 자율 주행 노드 (Nav2 미사용, 센서 기반 조향)
    └── 📂 robot_Qt               # Robot 제어용 Qt GUI 프로젝트

```

## 📦 패키지 상세 설명

### 1. `ros2_turtlebot_createdpkg`

ROS2 환경에서 동작하는 핵심 패키지들이 포함된 폴더입니다.

* **`relay_bot_pkg`**
* **기능:** Nav2 패키지를 사용하지 않고, 기본적인 조향 알고리즘과 센서 토픽 수신 기능만으로 터틀봇을 주행시키는 노드입니다.
* **포함:** 시뮬레이션용 노드 및 관련 설정 파일.
* **참고:** 시뮬레이션 환경 설정을 위한 수정 사항은 해당 폴더 내 별도 `README.md`를 참고하세요.


* **`robot_Qt`**
* 로봇 제어 및 모니터링을 위한 Qt 어플리케이션 소스 코드입니다.



### 2. `comm_pkg`

ROS2 미들웨어를 거치지 않는 별도의 통신망 구축을 위한 소스 코드 모음입니다.

* 이 폴더에서 직접 작업하거나, 작업 완료 후 이곳에 복사하여 Push 해주세요.

---

## 🚀 설치 및 세팅 (Installation)

터미널을 열고 워크스페이스의 `src` 폴더로 이동하여 저장소를 클론합니다.

```bash
# 1. 워크스페이스 src 폴더로 이동
cd ~/turtlebot3_ws/src/

# 2. 저장소 Clone
git clone [https://github.com/seolihan651/Intel-4th-Project.git](https://github.com/seolihan651/Intel-4th-Project.git)

# 3. 의존성 설치 및 빌드 (필요시)
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/local_setup.bash

```

## 🛠️ 개발 환경

* **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
* **ROS2:** Humble Hawksbill
* **Device:** TurtleBot3, Raspberry Pi, STM32 (예정)

```


# ROS2 Lane Detection Example

이 프로젝트는 ROS2와 OpenCV를 활용하여 영상에서 차선을 인식하고, 그 결과를 시각화하며, GUI를 통해 파라미터를 실시간으로 튜닝하는 예제입니다.

## 📂 프로젝트 구조

```
ros2_vision_example_2/
├── src/
│   ├── lane_detection/       # 메인 패키지
│   │   ├── lane_detection/
│   │   │   ├── video_publisher.py   # 영상 송출 노드
│   │   │   ├── lane_processor.py    # 차선 인식 처리 노드
│   │   │   ├── visualizer_node.py   # 시각화 노드
│   │   │   ├── gui_node.py          # 통합 GUI 대시보드
│   │   │   └── param_node.py        # (GUI 미사용 시) 파라미터 관리
│   │   └── launch/
│   │       └── lane_detection.launch.py # 전체 실행 런처
│   └── lane_msgs/            # 커스텀 메시지 패키지
│       ├── msg/
│       │   ├── LaneParams.msg       # 파라미터 제어 메시지
│       │   └── LaneData.msg         # 차선 정보 메시지
├── sample/
│   └── camera_open.mp4       # 테스트용 영상 파일
├── run_lane_detection.sh     # Linux/WSL 실행 스크립트
└── run_lane_detection.bat    # Windows 실행 스크립트
```

## 🚀 실행 방법

### 1. 필수 요구사항
*   ROS2 (Humble/Foxy 등)
*   Python 3
*   OpenCV (`pip install opencv-python`)
*   PyQt5 (`pip install PyQt5`)

### 2. 빌드 및 실행
프로젝트 루트에서 다음 스크립트를 실행하면 빌드 후 자동으로 실행됩니다.

**Windows (CMD/PowerShell):**
```cmd
run_lane_detection.bat
```

**Linux / WSL:**
```bash
./run_lane_detection.sh
```

## 🛠️ 시스템 아키텍처 (Data Flow)

1.  **Input (`video_publisher`)**
    *   `sample/camera_open.mp4` 파일을 읽습니다.
    *   `/camera_open` 토픽으로 이미지 메시지를 발행합니다.

2.  **Control (`gui_node`)**
    *   사용자가 GUI 슬라이더로 HSV, Canny, ROI 값을 조절합니다.
    *   `/lane_params` 토픽으로 설정값을 실시간 전송합니다.

3.  **Process (`lane_processor`)**
    *   `/camera_open` 영상과 `/lane_params` 설정을 수신합니다.
    *   **전처리**: Gaussian Blur -> HSV 변환 -> ROI 자르기.
    *   **변환**: Perspective Transform (Bird's Eye View).
    *   **인식**: 히스토그램 분석으로 차선 위치 추출.
    *   `/lane_data` (차선 정보)와 `/lane_debug_img` (처리된 영상)를 발행합니다.

4.  **Visualization (`visualizer_node`)**
    *   `/lane_debug_img` 위에 `/lane_data`를 기반으로 **초록색 화살표**를 그립니다.
    *   `/lane_visual` 토픽으로 최종 결과 이미지를 발행합니다.

5.  **Display (`gui_node`)**
    *   원본 영상, 처리된 영상, 시각화 결과를 한 화면에 보여줍니다.

## 🎛️ GUI 사용법

실행 시 나타나는 대시보드 창에서 다음을 조절할 수 있습니다:

*   **HSV Thresholds**: 차선 색상(흰색/노란색)을 분리하기 위한 색상 범위 조절.
*   **Canny Edge**: 엣지 검출 민감도 조절.
*   **Geometry**:
    *   `ROI Height %`: 이미지 상단에서 잘라낼 비율.
    *   `Warp Offset`: Bird's Eye View 변환 시 시야각 조절.

---
**Tip**: 영상이 너무 빠르거나 느리다면 `video_publisher.py`의 타이머 주기를 조절하세요.

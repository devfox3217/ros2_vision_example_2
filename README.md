# ROS2 Lane Detection System

## 1. 영상 데이터 처리 흐름

### 1.1. 입력 (Input)
카메라에서 들어온 원본 데이터를 처리 가능한 형태로 변형합니다.
- Input: /camera_open 토픽에서 수신
- Process: 
  - cv_bridge를 사용하여 ROS 이미지 메시지를 OpenCV(BGR) 포맷으로 변환
  - 연산 속도를 위하여 이미지 리사이징 및 ROI 설정 (현재 코드에서는 파라미터로 상단 영역 Crop 수행)
- Data: RAW Pixel Data (BGR)

### 1.2. 처리 (Process)
픽셀 데이터에서 ‘차선’ 이라고 부를 수 있는 특징점을 찾아냅니다.
- Preprocessing: 
  - 가우시안 블러(Gaussian Blur)로 고주파 노이즈 제거
  - HSV 색상 변환을 통해 조명 변화에 강한 특징 추출 (흰색/노란색 차선 강조)
- Binarization: 
  - HSV Thresholding을 통해 차선은 흰색(1), 나머지는 검은색(0)으로 분리하는 마스크 생성
- Perspective Transform: 
  - Bird's Eye View로 변환하여 원근감을 제거하고 차선을 위에서 내려다보는 평면 좌표로 변환 (곡률 계산의 정확도 향상)

### 1.3. 데이터 수치화 (Feature Extraction)
이미지 상의 흰색 점들을 제어에 쓸 수 있는 수학적 정보로 추출합니다.
- Sliding Window Algorithm:
  - 히스토그램 분석으로 차선 시작점 탐색
  - 9개의 윈도우를 이용하여 곡선 도로에서도 차선 픽셀을 놓치지 않고 추적
  - Parameters: sw_margin(검색 너비), sw_minpix(최소 픽셀 수)를 조절하여 민감도 튜닝 가능
- Curve Fitting:
  - 추적된 픽셀들을 2차 함수(x = ay^2 + by + c)로 피팅하여 차선의 곡률 모델링
- Calculation:
  - lane_center: 화면 중앙 대비 차선 중심의 편차 계산 (-1.0 ~ 1.0)
  - curve_radius: 상단 중심점과 하단 중심점의 차이를 이용해 조향각(Steering Angle) 계산

### 1.4. ROS2 메시지 정의 및 발행
가공된 정보를 다음 노드(판단/제어 노드)가 쓸 수 있도록 구조화하여 보냅니다.
- Custom Message (LaneData.msg): 
  - 단순히 이미지를 다시 보내는 것이 아니라, 수치 데이터를 보냅니다.
  - float32 lane_center: 차선 중심 편차
  - float32 curve_radius: 조향각 계산을 위한 픽셀 차이 값
  - bool lane_detected: 차선 검출 여부
- Publisher: 
  - lane_processor 노드가 위 데이터를 담아 /lane_data 토픽으로 발행
  - 이후 gui_node나 visualizer_node가 이 토픽을 구독하여 시각화 및 제어에 사용

---

## 2. 시스템 실행 방법

프로젝트 루트에 포함된 스크립트를 사용하여 빌드 및 실행을 한 번에 수행할 수 있습니다.

1. 실행 권한 부여 (최초 1회):
   ```bash
   chmod +x run_lane_detection.sh
   ```

2. 시스템 실행:
   ```bash
   ./run_lane_detection.sh
   ```
   * 스크립트가 자동으로 `colcon build`, `source setup.bash`, `ros2 launch`를 순차적으로 수행합니다.
   * 모든 노드(Processor, GUI, Visualizer)가 자동으로 실행됩니다.

## 3. 파라미터 튜닝 가이드 (GUI)

GUI 대시보드의 Trackbar를 사용하여 다음 값들을 실시간으로 조절할 수 있습니다.

### 일반 설정
- Debug View: 이미지 처리 및 시각화 기능을 켜고 끕니다. (OFF 시 연산 부하 감소)
- Reset Params: 모든 설정을 초기 기본값으로 되돌립니다.

### 색상 필터링 (HSV)
- H/S/V Min/Max: 차선 색상(흰색/노란색)을 배경과 분리하기 위한 임계값 조절

### 영역 및 변환 (Geometry)
- ROI Height %: 불필요한 상단 배경(하늘 등)을 제거하는 비율
- Warp Offset: Bird's Eye View 변환 시 도로 폭 조절

### Sliding Window 설정
- SW Margin: 윈도우의 좌우 검색 너비 (값이 클수록 휘어진 차선을 잘 찾지만 잡음에 취약)
- SW MinPix: 윈도우 이동을 위한 최소 픽셀 수 (값이 클수록 노이즈에 강함)

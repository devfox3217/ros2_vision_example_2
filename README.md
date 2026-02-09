# ROS2 Lane Detection System

ROS2와 OpenCV를 활용한 차선 인식 및 조향각 계산 시스템입니다. Sliding Window 알고리즘을 적용하여 곡선 도로에서의 인식률을 높였으며, 실시간 튜닝이 가능한 GUI 대시보드를 제공합니다.

## 🏗 시스템 구조 (Architecture)

시스템은 크게 3개의 노드로 구성되어 있습니다.

### 1. Lane Processor (`lane_processor.py`)
핵심 영상 처리 및 차선 위치 계산을 담당합니다.
- **알고리즘**:
  1. **Preprocessing**: Gaussian Blur & HSV Color Thresholding.
  2. **Warping**: Perspective Transform을 통해 Bird's Eye View 생성.
  3. **Sliding Window**: 히스토그램 분석 및 9개의 윈도우를 이용한 차선 픽셀 추적.
  4. **Curve Fitting**: 2차 함수(`ax^2 + bx + c`) 피팅.
- **Output**: 
  - 차선 중심 위치 (`lane_center`)
  - 곡률/회전량 (`curve_radius`)
  - 디버깅 이미지 (Sliding Window 박스 시각화)

### 2. GUI Node (`gui_node.py`)
사용자 제어 패널 및 통합 모니터링 대시보드입니다.
- **기능**:
  - **3단 뷰어**: 원본 영상, 처리 영상(Sliding Window), 최종 시각화 영상을 한눈에 확인.
  - **Parameter Tuning**: HSV 값, ROI, Canny Edge 등 파라미터를 실시간 조절 (Trackbar).
  - **Steering Angle**: 계산된 조향각을 하단에 큰 텍스트로 표시 (직진/회전 상태에 따라 색상 변경).
  - **Reset**: 파라미터 초기화 기능 제공.

### 3. Visualizer Node (`visualizer_node.py`)
데이터 시각화 및 정보 오버레이를 담당합니다.
- **기능**:
  - 처리된 이미지 위에 수치 데이터(Center, Curve, Angle) 텍스트 표시.
  - 차선 미검출 시 경고 메시지 출력.

## 📡 데이터 흐름 (Data Flow)

| Topic Name | Type | Description | Publisher | Subscriber |
|------------|------|-------------|-----------|------------|
| `/camera_open` | `sensor_msgs/Image` | 원본 카메라 영상 | Camera Node | Processor, GUI |
| `/lane_params` | `lane_msgs/LaneParams` | 튜닝 파라미터 | GUI | Processor |
| `/lane_data` | `lane_msgs/LaneData` | 계산된 차선 정보 | Processor | GUI, Visualizer |
| `/lane_debug_img` | `sensor_msgs/Image` | Sliding Window 처리 영상 | Processor | GUI, Visualizer |
| `/lane_visual` | `sensor_msgs/Image` | 최종 정보 오버레이 영상 | Visualizer | GUI |

## 🚀 사용 방법 (Usage)

1. **빌드**:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **실행**:
   각 노드를 별도의 터미널에서 실행하거나 launch 파일을 사용합니다.
   ```bash
   # 1. 차선 처리 노드
   ros2 run lane_detection lane_processor
   
   # 2. 시각화 노드
   ros2 run lane_detection visualizer_node
   
   # 3. GUI 대시보드 (최종 확인용)
   ros2 run lane_detection gui_node
   ```

## ⚙️ 파라미터 상세 가이드 (Parameter Tuning Guide)

GUI 대시보드의 Trackbar를 사용하여 다음 값들을 실시간으로 조절할 수 있습니다.

### 🎨 색상 필터링 (HSV)
차선의 색상을 정확하게 추출하기 위한 설정입니다.
- **H Min / H Max (Hue)**: 색상(색조)의 범위를 지정합니다.
  - *Tip*: 흰색 차선은 색조의 영향이 적으므로 범위를 넓게(0~180) 잡아도 되지만, 노란색 차선은 특정 범위(20~30 등)로 좁혀야 합니다.
- **S Min / S Max (Saturation)**: 채도(색의 선명도) 범위를 지정합니다.
  - *Tip*: 흰색은 채도가 매우 낮으므로 `S Max`를 낮게(예: 50 이하) 설정하면 흰색만 잘 걸러낼 수 있습니다.
- **V Min / V Max (Value)**: 명도(밝기) 범위를 지정합니다.
  - *Tip*: 그림자나 어두운 도로 바닥을 제외하고 밝은 차선만 남기려면 `V Min`을 높게(예: 200 이상) 설정하세요.

### 📐 영역 및 변환 (Geometry)
- **ROI Height %**: 관심 영역(Region of Interest)의 높이 비율입니다.
  - *설명*: 이미지의 위쪽(하늘, 먼 배경)을 얼마나 잘라낼지 결정합니다. 값이 60이면 위에서 40%를 자르고 아래 60%만 사용합니다.
- **Warp Offset**: Bird's Eye View 변환 시 좌우 여백을 조절합니다.
  - *설명*: 이 값이 클수록 변환된 이미지에서 도로가 더 좁게(멀리) 보이고, 작을수록 더 넓게 보입니다. 차선이 평행하게 보이도록 조절하세요.

### 🔍 기타
- **Canny Low / High**: (현재 로직에서는 HSV가 주력이므로 보조적 역할) 엣지 검출의 민감도를 조절합니다.
- **Reset Params**: 모든 설정을 초기 기본값으로 되돌립니다.

## 📐 조향각 계산 (Steering Angle)

- **원리**: Bird's Eye View 상에서 차선의 상단 중심점($x_{top}$)과 하단 중심점($x_{bottom}$)의 차이를 이용합니다.
- **공식**: $\theta = \arctan(\frac{x_{top} - x_{bottom}}{height/2})$
- **표시**: GUI 하단에 각도(deg)로 표시되며, 0도에 가까울수록 직진, 값이 커질수록 회전이 필요함을 의미합니다.

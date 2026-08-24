# Reactive Gap Follow Ver2

LiDAR로 주행 가능한 공간을 찾고 Ackermann 조향·속도 명령을 생성하는 ROS 2 패키지입니다. `/odom` localization과 최초 `/global_path`를 받아, 좌우 gap이 모두 안전할 때 path 방향의 gap을 우선합니다. path가 아직 없거나 localization이 오래되면 기존 LiDAR-only gap follower로 자동 복귀합니다.

이 구현은 기본 Follow-the-Gap에 다음 기능을 추가합니다.

- 차량 반폭과 벽 여유를 반영한 장애물 확장
- 깊이와 주변 개방도를 함께 사용하는 목표 방향 점수
- 좌우 방향이 애매할 때 한쪽으로 너무 일찍 진입하지 않도록 하는 분기 판단
- 현재 조향각을 반영한 Ackermann 궤적 충돌검사
- LiDAR 안전조건을 통과한 후보에만 적용되는 reference path 조향 preference
- 장애물이 가까워도 완전히 정지하지 않고 저속으로 계속 움직이는 속도 제어
- RViz용 목표점, 예상 궤적, 차량 footprint 디버그 마커

## 동작 구조

```mermaid
flowchart LR
    A[LaserScan] --> B[유효 범위 정리 및 시야각 제한]
    B --> C[장애물 경계 검출]
    C --> D[차량 반폭만큼 장애물 확장]
    D --> E[중앙값 필터]
    E --> F[각 후보 방향 점수 계산]
    F --> G[LiDAR로 안전한 gap 후보 제한]
    K[odom + 최초 global_path] --> L[lookahead path 방향]
    L --> G
    G --> M[양쪽이 안전하면 path 방향 우선]
    M --> H[좌우 분기 및 막힌 경로 재검사]
    H --> I[Ackermann 궤적 충돌검사]
    I --> J[조향·속도 명령 발행]
```

### 1. LiDAR 전처리

`/scan`의 `NaN`, `Inf`, 측정 범위 밖 값은 센서의 `range_min` 또는 `range_max`로 정리합니다. 이후 전방 `-max_scan_angle`부터 `+max_scan_angle`까지만 사용합니다.

인접 포인트의 거리 차이가 `obstacle_edge_threshold`보다 크면 장애물 경계로 판단합니다. 각 경계는 다음 두 크기로 각각 확장됩니다.

- 기본 scan: `vehicle_width / 2`
- 선호 scan: `vehicle_width / 2 + gap_safety_margin`

먼저 선호 scan으로 목표를 찾고, 충분한 공간이 없으면 기본 scan으로 다시 찾습니다. 따라서 평소에는 벽과 여유를 두지만 좁은 통로가 실제로 통과 가능하면 멈추지 않고 진입할 수 있습니다.

### 2. 목표 방향 점수

각 LiDAR 후보 방향 주변 `±path_check_angle`에서 두 값을 계산합니다.

- `safe_distance`: 주변 거리값을 정렬한 뒤 `safety_level` 위치에서 선택한 거리
- `guard_clearance`: 같은 주변 거리에서 `candidate_safety_level` 위치를 선택한 후보 통과 기준
- `width_score`: 주변 거리값을 `gap_score_distance_cap`으로 제한한 뒤 계산한 평균

최종 점수는 다음과 같습니다.

```text
score = (1 - gap_width_preference) * capped_safe_distance
        + gap_width_preference * width_score
```

`width_score`는 gap의 실제 폭을 미터로 측정한 값이 아니라, 후보 방향 주변이 얼마나 넓게 열려 있는지를 나타내는 평균 거리입니다.

`safety_level`의 의미는 다음과 같습니다.

| 값 | 선택되는 주변 거리 |
|---:|---|
| `0.0` | 최소 거리, 가장 보수적 |
| `0.2` | 아래쪽 20% 위치의 거리 |
| `0.5` | 중앙값 |
| `1.0` | 최대 거리, 가장 공격적 |

`safety_level`과 `candidate_safety_level`은 반드시 `0.0`부터 `1.0` 사이로 설정해야 합니다. 높은 `safety_level`은 기존 gap 점수와 속도 특성을 유지하고, 더 낮은 `candidate_safety_level`은 좁은 정적 장애물이 주변의 긴 ray에 묻혀 안전 후보로 잘못 통과하는 것을 막습니다.

### 3. 좌우 분기 판단

선택된 목표가 `path_check_angle`보다 바깥에 있으면 반대편에서 가장 좋은 gap도 함께 계산합니다. 다음 조건을 모두 만족하면 아직 방향이 애매한 것으로 판단합니다.

- 선택 방향과 반대 방향 모두 `gap_fallback_distance`보다 깊음
- 선택 점수가 반대 점수보다 `ambiguous_gap_score_margin` 이상 확실하게 높지 않음

반대편에 더 바깥쪽으로 열린 후보가 있으면 `ambiguous_gap_opposite_weight`를 적용하여 조향에 반영합니다. 그렇지 않으면 현재 선택 방향을 유지하되 조향 크기를 `path_check_angle` 안으로 제한하여 반대편 공간이 더 보일 때까지 강조향을 늦춥니다.

이 로직은 넓고 깊어 보이지만 막혀 있는 옆 공간으로 차량이 너무 일찍 들어가, 실제 트랙 방향을 시야에서 잃는 현상을 줄이기 위한 보조 로직입니다.

### 4. Reference path 조향 preference

`/odom`의 현재 위치·yaw와 최초로 수신한 `/global_path`로부터 `path_lookahead_distance` 앞의 목표점을 구합니다. 이 목표 방향은 속도 계산에는 사용하지 않으며 gap 후보의 조향 preference에만 사용합니다.

path 가산점은 다음 LiDAR 안전조건을 모두 만족한 후보에만 적용됩니다.

- 후보 `guard_clearance`가 공통 `candidate_min_clearance` 이상
- LiDAR-only 최적 gap 점수의 `path_candidate_min_score_ratio` 이상

따라서 path 앞이 막히고 반대쪽만 안전하면 기존 gap follower처럼 반대쪽으로 회피합니다. 좌우가 모두 위 조건을 만족할 때만 path와 가까운 방향이 선택됩니다. 경로에서 `path_rejoin_distance`보다 멀어지면 `path_rejoin_weight`가 적용되어 안전 후보 사이에서 복귀 방향 preference가 강해집니다.

유효한 `/global_path`는 pose가 두 개 이상이고 `header.frame_id`가 있어야 합니다. 노드는 최초 유효 메시지를 내부에 복사한 뒤 잠그므로 이후 path 메시지는 모두 무시합니다. 기본 QoS는 한 번 발행된 latched path를 받을 수 있도록 `reliable + transient_local`입니다. `/global_path.header.frame_id`와 `/odom.header.frame_id`가 다르면 TF로 임의 변환하지 않고 path preference를 비활성화합니다.

### 5. 조향 궤적 충돌검사

최신 `/ackermann_cmd` 조향에서 새 목표 조향까지 `steering_transition_time` 동안 점진적으로 변하는 Ackermann 궤적을 `trajectory_check_step` 간격으로 검사합니다. 조향 피드백이 오래됐으면 직전 gap follower 명령을 시작 조향으로 사용합니다. 각 예측 pose에서 LiDAR 포인트를 차량 좌표로 변환하고, 다음 직사각형 안에 들어오면 충돌로 판단합니다.

```text
half_length = vehicle_length / 2 + trajectory_safety_margin
half_width  = vehicle_width / 2 + trajectory_safety_margin
```

이상적인 Ackermann 곡률에 workspace의 `0818`, `0819_1`, `test_12` bag에서 실측한 속도별 곡률 이득을 곱합니다. 명령 조향이 0.30초 이상 안정되고 wheel 속도가 안정된 구간을 0.25초 블록으로 나눈 147개 표본 중, IMU bias 영향을 덜 받는 25° 이상 조향 표본 97개를 robust fitting했습니다.

```text
ideal_curvature = tan(steering) / wheel_base
curvature_gain(v) = clamp(
    1 / (curvature_model_offset + curvature_model_speed_squared_gain * v²),
    curvature_gain_min,
    curvature_gain_max)
modeled_curvature = ideal_curvature × curvature_gain(v)
```

속도 bin별 실측 gain 중앙값은 약 `1.22 (0.35~0.6 m/s)`, `0.98 (0.6~0.9)`, `0.65 (0.9~1.2)`, `0.64 (1.2~1.5, 표본 3개)`였습니다. 1.2 m/s를 넘는 안정 코너 표본이 부족하므로 고속 외삽은 `curvature_gain_min: 0.60`에서 제한합니다. 이 모델은 충돌검사 궤적, RViz footprint 궤적, 마찰 기반 코너 속도 상한에 공통 사용됩니다.

같은 30° 조향에서 모델이 사용하는 대표 회전반경은 다음과 같습니다.

| 속도 | 곡률 이득 | 예측 회전반경 |
|---:|---:|---:|
| `0.50 m/s` | `1.17` | `0.48 m` |
| `0.75 m/s` | `0.96` | `0.58 m` |
| `1.00 m/s` | `0.77` | `0.73 m` |
| `1.20 m/s` | `0.64` | `0.88 m` |
| `1.50 m/s 이상` | `0.60` | `0.94 m` |

선택한 궤적에서 검사 horizon 안의 충돌점이 하나라도 발견되면 즉시 전체 조향 범위를 다시 검사합니다. `candidate_safety_level`로 계산한 `guard_clearance`가 공통 `candidate_min_clearance` 이상이고, 직사각형 전환 궤적이 horizon 끝까지 완전히 충돌 없는 조향만 회피 후보가 됩니다. 통과한 안전 후보들의 LiDAR clearance·조향 변화 점수에 path alignment 가산점을 더하므로 양쪽 회피가 모두 안전하면 path에 가까운 쪽을 우선합니다. 모든 조향에 충돌점이 있을 때만 속도 상한을 계산합니다.

장애물을 피해 한쪽 조향을 선택하면 `avoidance_direction_hold_time` 동안 같은 방향을 유지합니다. 이후 정면 안전거리가 `2 × candidate_min_clearance` 이상인 상태가 `0.5 × avoidance_direction_hold_time` 동안 이어지면 고정을 해제합니다. 단, 고정된 쪽에서 안전한 궤적을 찾지 못하면 즉시 반대쪽 회피 또는 crawl을 허용합니다.

최소 유지시간이 지난 뒤 global path 방향의 직사각형 전환 궤적이 안전하면 추가 정면-clear 대기 없이 회피 방향 고정을 해제하고 path 방향으로 복귀합니다. path 방향이 안전하지 않으면 기존 회피 방향을 계속 유지하므로 복귀 preference가 장애물 안전검사를 우회하지 않습니다.

### 6. 속도 제어

기본 목표 속도는 안전거리에 `speed_factor`를 곱해 계산하며, 먼 공간에서는 `speed_increase_factor`만큼 점진적으로 증가합니다. 조향이 클 때는 마찰계수와 실측 속도별 곡률로 계산한 선회 속도 한계를 적용합니다.

제동거리는 `steering_transition_time`, `/odom` 속도, `normal_deceleration`, `trajectory_safety_margin`으로 계산합니다. 전환 궤적이 horizon 끝까지 안전한 조향이 하나라도 있으면 감속하지 않습니다. 모든 조향이 위험하면 충돌거리에서 `normal_deceleration`으로 정지 가능한 속도를 즉시 역산하여 완만하게 감속하고, TTC가 `steering_transition_time`보다 짧아진 실제 비상상황에서만 `max_deceleration`으로 `minimum_crawl_speed`를 향해 감속합니다.

## ROS 인터페이스

launch 실행 시 노드 전체 이름은 `/gap_follow_ver2/gap_follow_ver2`입니다. 기존 `gap_follow`와 동시에 실행할 수 있도록 노드 이름, namespace, 실행파일 및 출력 토픽을 분리했습니다.

| 구분 | 기본 토픽 | 메시지 | 설명 |
|---|---|---|---|
| Subscribe | `/scan` | `sensor_msgs/msg/LaserScan` | 전방 장애물 및 gap 계산 입력 |
| Subscribe | `/odom` | `nav_msgs/msg/Odometry` | path preference용 localization 입력 |
| Subscribe | `/global_path` | `nav_msgs/msg/Path` | 최초 메시지만 고정 사용하는 reference path |
| Publish | `/drive_gf2` | `ackermann_msgs/msg/AckermannDriveStamped` | 조향각, 속도, 가속도 명령 |
| Publish | `/gap_follow_ver2/target_waypoint` | `geometry_msgs/msg/PointStamped` | 선택한 로컬 목표점, `debug: true`일 때 발행 |
| Publish | `/gap_follow_ver2/debug_markers` | `visualization_msgs/msg/MarkerArray` | 목표 방향, 예상 궤적, footprint, 상태 표시 |

`target_waypoint`는 지도상의 global waypoint가 아니라 현재 LiDAR frame 기준의 최종 gap 목표점입니다.

## 빌드 및 실행

ROS 2 Humble 환경을 기준으로 합니다.

```bash
source /opt/ros/humble/setup.bash
cd /home/rcv/Desktop/gap_follow
rosdep install --from-paths . --ignore-src -r -y
colcon build --packages-select gap_follow_ver2 --symlink-install
source install/setup.bash
ros2 launch gap_follow_ver2 gap_follow.launch.py
```

설정 파일은 [`config/gap_follow.yaml`](config/gap_follow.yaml)입니다. launch 파일이 `/gap_follow_ver2/gap_follow_ver2` 노드에 이 설정을 자동으로 적용합니다.

path 기능만 잠시 끄려면 `enable_path_guidance: false`로 설정하면 됩니다.

## rosbag 및 RViz 확인

터미널 1에서 노드를 실행합니다.

```bash
source /opt/ros/humble/setup.bash
cd /home/rcv/Desktop/gap_follow
source install/setup.bash
ros2 launch gap_follow_ver2 gap_follow.launch.py
```

터미널 2에서 bag을 재생합니다.

```bash
source /opt/ros/humble/setup.bash
ros2 bag info <bag_path>
ros2 bag play <bag_path>
```

터미널 3에서 RViz를 실행합니다.

```bash
source /opt/ros/humble/setup.bash
rviz2
```

RViz에는 다음 display를 추가합니다.

| Display | Topic | 용도 |
|---|---|---|
| `LaserScan` | `/scan` | 원본 LiDAR 확인 |
| `MarkerArray` | `/gap_follow_ver2/debug_markers` | 목표 방향과 충돌검사 궤적 확인 |
| `PointStamped` | `/gap_follow_ver2/target_waypoint` | 선택 목표점 확인 |

`Fixed Frame`은 bag의 `/scan` 메시지에 기록된 frame을 사용합니다. 일반적으로 `base_link` 또는 LiDAR frame입니다. 마커가 보이지 않으면 `debug: true`, 토픽 이름, Fixed Frame과 TF 연결을 확인합니다.

## 파라미터

### 주행 방향 핵심 튜닝

| 파라미터 | 현재값 | 의미 및 조정 방향 |
|---|---:|---|
| `gap_width_preference` | `0.4` | 작을수록 깊이 우선, 클수록 주변이 넓은 방향 우선 |
| `gap_score_distance_cap` | `5.0` m | 이 거리보다 먼 값에는 추가 점수를 주지 않음 |
| `ambiguous_gap_score_margin` | `1.8` | 클수록 좌우 결정을 늦추고 반대 gap을 더 오래 비교 |
| `ambiguous_gap_opposite_weight` | `2.0` | 클수록 늦게 보이는 반대편 바깥 gap을 강하게 반영 |

### Reference path 조향 preference

| 파라미터 | 현재값 | 설명 |
|---|---:|---|
| `enable_path_guidance` | `true` | global path와 localization이 유효할 때 path 조향 preference 활성화 |
| `localization_topic` | `/odom` | `Odometry` localization 토픽 |
| `odom_timeout` | `0.3` s | path pose와 TTC 속도에 공통 적용하는 odom 유효시간 |
| `global_path_topic` | `/global_path` | `nav_msgs/msg/Path` 입력 토픽 |
| `global_path_transient_local` | `true` | latched path 수신용 transient-local QoS |
| `path_is_closed` | `true` | 폐곡선 트랙 여부 |
| `path_lookahead_distance` | `1.2` m | path 진행방향의 조향 목표 거리 |
| `path_guidance_weight` | `0.6` | 평상시 안전 gap 사이의 path preference |
| `path_rejoin_weight` | `2.0` | path 이탈 시 안전 gap 및 안전 회피 후보 사이의 복귀 preference |
| `path_rejoin_distance` | `0.5` m | 복귀 preference 적용 시작 거리 |
| `path_candidate_min_score_ratio` | `0.85` | LiDAR-only 최적 gap 대비 최소 기본점수 비율 |
| `path_alignment_sigma` | `35.0` deg | path 방향 preference의 각도 폭 |
| `path_max_guidance_angle` | `85.0` deg | 이보다 뒤쪽인 path 목표는 무시 |
| `path_heading_search_weight` | `0.5` | 교차 경로에서 진행방향이 같은 segment 선호도 |

### 조향 및 속도

| 파라미터 | 현재값 | 설명 |
|---|---:|---|
| `max_steering_angle` | `45.0` deg | 명령 및 충돌검사에 사용하는 최대 조향각 |
| `steering_smooth_window` | `3` | 최근 조향 명령 평균 개수 |
| `speed_factor` | `0.24` | 안전거리에서 기본 속도로 변환하는 계수 |
| `max_speed` | `3.0` m/s | 최대 속도 |
| `narrow_gap_max_speed` | `1.5` m/s | 기본 차량 반폭만으로 재탐색한 좁은 gap의 속도 제한 |
| `minimum_non_emergency_speed` | `1.0` m/s | 비상 충돌 궤적이 아닐 때 유지하는 일반 최저 속도 |
| `minimum_crawl_speed` | `0.3` m/s | 실제 비상 충돌 궤적에서 유지하는 crawl 속도 |
| `speed_increase_start` | `3.0` m | 추가 속도 배율이 시작되는 안전거리 |
| `speed_increase_end` | `15.0` m | 추가 속도 배율이 최대가 되는 안전거리 |
| `speed_increase_factor` | `1.5` | 먼 공간에서 적용할 최대 속도 배율 |
| `min_acceleration` | `0.5` m/s² | 가까운 구간의 가속도 제한 |
| `max_acceleration` | `6.0` m/s² | 열린 구간의 최대 가속도 제한 |
| `normal_deceleration` | `3.0` m/s² | 회피 불가능 장애물에 미리 대응하는 평상시 감속도 |
| `max_deceleration` | `6.0` m/s² | TTC 비상상황에만 사용하는 최대 감속도 |

### 차량 형상 및 gap 안전 기준

| 파라미터 | 현재값 | 설명 |
|---|---:|---|
| `friction` | `0.75` | 선회 속도 계산에 사용하는 마찰계수 |
| `wheel_base` | `0.324` m | Ackermann wheelbase |
| `curvature_model_offset` | `0.71` | 실측 곡률 이득 모델의 상수항 |
| `curvature_model_speed_squared_gain` | `0.59` | 속도 증가에 따른 실제 곡률 감소 계수 |
| `curvature_gain_min` | `0.60` | 표본 부족 고속 구간의 최소 곡률 이득 |
| `curvature_gain_max` | `1.25` | 저속 구간의 최대 곡률 이득 |
| `vehicle_width` | `0.30` m | base_link 중심 직사각형 차량 폭 |
| `vehicle_length` | `0.55` m | base_link 중심 직사각형 차량 길이 |
| `lidar_offset_x` | `0.27` m | `base_link` 기준 LiDAR x 위치 |
| `lidar_offset_y` | `0.0` m | `base_link` 기준 LiDAR y 위치 |
| `gap_safety_margin` | `0.1` m | 선호 gap을 만들 때 차량 반폭에 추가하는 벽 여유 |
| `gap_fallback_distance` | `0.4` m | 선호 gap이 부족할 때 기본 반폭 scan으로 재탐색하는 기준 |
| `path_check_angle` | `30.0` deg | 후보 주변 안전거리 검사 반각이자 애매한 분기의 조향 제한 |
| `safety_level` | `0.8` | gap 점수와 속도에 사용할 주변 거리 분포 위치 |
| `candidate_safety_level` | `0.4` | path 및 회피 후보의 안전 통과 여부에 사용할 분포 위치 |
| `candidate_min_clearance` | `0.5` m | path 및 회피 후보에 공통 적용할 최소 안전거리 |

### LiDAR 및 충돌검사

| 파라미터 | 현재값 | 설명 |
|---|---:|---|
| `obstacle_edge_threshold` | `0.15` m | 장애물 경계로 판단할 인접 ray 거리 차이 |
| `scan_filter_window` | `5` | 중앙값 필터 창 크기 |
| `max_scan_angle` | `90.0` deg | 전방 기준 한쪽 시야각. 실제 사용 범위는 ±90도 |
| `avoidance_steering_step` | `2.0` deg | 대체 조향 탐색 간격 |
| `avoidance_steering_change_penalty` | `0.5` | 기존 gap 목표에서 크게 벗어나는 조향 억제값 |
| `avoidance_direction_hold_time` | `0.6` s | 선택한 회피 방향을 최소 유지하는 시간 |
| `avoidance_direction_min_angle` | `10.0` deg | 회피 방향 고정을 시작하고 유지할 최소 조향각 |
| `trajectory_check_distance` | `1.0` m | 현재 조향 궤적을 검사할 거리 |
| `trajectory_check_step` | `0.05` m | 궤적 샘플 간격 |
| `trajectory_safety_margin` | `0.05` m | 직사각형 전후·좌우와 제동거리에 공통 적용하는 여유 |
| `steering_feedback_topic` | `/ackermann_cmd` | 전환 궤적의 시작 조향으로 사용할 최종 명령 |
| `steering_transition_time` | `0.20` s | 현재→목표 조향 전환시간이자 비상 TTC 기준 |
| `initial_update_time` | `0.05` s | 첫 제어 주기의 가감속 계산 시간 |

### ROS 및 디버그

| 파라미터 | 현재값 | 설명 |
|---|---:|---|
| `default_qos` | `1` | publisher와 subscriber의 QoS depth |
| `debug` | `true` | 목표점과 디버그 마커 발행 여부 |
| `lidar_scan_topic` | `/scan` | LiDAR 입력 토픽 |
| `drive_topic` | `/drive_gf2` | Ackermann 명령 출력 토픽 |
| `target_waypoint_topic` | `/gap_follow_ver2/target_waypoint` | 로컬 목표점 토픽 |
| `debug_marker_topic` | `/gap_follow_ver2/debug_markers` | RViz 마커 토픽 |
| `command_frame_id` | `base_link` | drive 명령 header의 frame ID |

## 튜닝 가이드

### 넓지만 막힌 옆 공간으로 빠질 때

1. `gap_width_preference`를 낮춰 깊이 항의 비중을 높입니다.
2. 올바른 방향이 좁지만 길게 열려 있다면 `gap_score_distance_cap`을 늘립니다.
3. 올바른 반대 방향이 늦게 보이면 `ambiguous_gap_score_margin` 또는 `ambiguous_gap_opposite_weight`를 조금씩 높입니다.

분기 파라미터를 너무 크게 하면 정상 코너에서도 방향 결정을 늦추거나 반대 방향을 과하게 반영할 수 있습니다.

### 벽에 너무 가까이 붙을 때

- 평상시 목표 선택 여유: `gap_safety_margin` 증가
- 예상 궤적의 충돌 여유: `trajectory_safety_margin` 증가
- 주변의 가까운 ray를 더 강하게 반영: `safety_level` 감소

`vehicle_width`와 `vehicle_length`는 실제 차량 형상이므로 단순 튜닝값으로 변경하기보다 먼저 두 margin을 조절하는 것이 좋습니다.

### 조향이 너무 흔들릴 때

- `steering_smooth_window`를 늘립니다.
- 너무 크게 설정하면 코너 진입이 늦어질 수 있습니다.

### 장애물 근처 속도가 너무 빠르거나 느릴 때

- 최저 속도: `minimum_crawl_speed`
- 좁은 gap 속도: `narrow_gap_max_speed`
- 평상시 감속 강도: `normal_deceleration`
- TTC 비상 감속 강도: `max_deceleration`

## 한계

- 현재 한 장의 전방 LiDAR scan을 중심으로 판단하므로 아직 가려진 코너의 연결 방향을 확정할 수 없습니다.
- localization이 끊기거나 global path가 아직 없으면 기존 LiDAR-only 판단으로 돌아가므로, 센서상 동일한 두 공간의 global 진행방향은 구분할 수 없습니다.
- `/global_path`와 `/odom`의 좌표 원점·축·진행방향이 일치해야 올바른 preference가 적용됩니다.
- rosbag 재생은 기록 차량의 실제 움직임에 따라 다음 scan이 들어오는 open-loop 검증입니다. 알고리즘이 출력한 조향대로 차량 시점이 변하는 실제 closed-loop 결과와 다를 수 있습니다.
- `path_check_angle`은 안전거리 검사 범위와 애매한 분기의 조향 제한에 함께 사용되므로 변경 시 두 동작이 동시에 달라집니다.

## 파일 구조

```text
gap_follow/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── gap_follow.yaml
├── launch/
│   └── gap_follow.launch.py
└── src/
    ├── gap_follow.cpp
    └── gap_follow.hpp
```

- `src/gap_follow.cpp`: LiDAR 처리, gap 선택, 조향·속도 제어 및 ROS 입출력
- `src/gap_follow.hpp`: `ReactiveGapFollow` 클래스 선언
- `config/gap_follow.yaml`: 차량, 주행, 안전, 토픽 파라미터
- `launch/gap_follow.launch.py`: 설정 파일을 적용해 노드를 실행하는 launch 파일

# gap_follow_ver2 튜닝 가이드

이 문서는 `config/gap_follow.yaml`의 최종 파라미터를 기준으로 증상별 진단과 조정 순서를 정리합니다.

안전 관련 값은 한 번에 여러 개를 변경하지 말고, 거리 `0.05 m`, 시간 `0.02 s`, weight `0.1~0.2` 정도의 작은 단위로 조정한 뒤 같은 bag으로 비교하는 것을 권장합니다.

## 1. 디버그 상태부터 확인하기

`/gap_follow_ver2/debug_markers`의 텍스트에 다음 값이 표시됩니다.

| 표시 | 의미 |
|---|---|
| `CLEAR` (초록) | 선택 궤적에 충돌이 없어 TTC 감속 없음 |
| `BRAKE` (주황) | horizon 끝까지 충돌 없는 조향을 찾지 못함. 계산된 안전 속도로 완만하게 감속 |
| `STOP` (빨강) | TTC가 `steering_transition_time` 이하인 비상상황. crawl로 감속 |
| `steer_state=FOLLOW` | 일반 gap/path 목표를 조향 중 |
| `steer_state=AVOID_LEFT` (청록) | 충돌 없는 왼쪽 회피 방향을 선택하거나 latch하여 조향 중 |
| `steer_state=AVOID_RIGHT` (청록) | 충돌 없는 오른쪽 회피 방향을 선택하거나 latch하여 조향 중 |
| `speed` | 2.6 보정이 적용된 하나의 실제 차량속도 |
| `steer` | gap follower 목표 조향 |
| `current` | `/ackermann_cmd`에서 받은 현재 조향 |
| `collision` | 전환 궤적상의 예상 충돌거리 |
| `ttc` | 예상 충돌까지 남은 시간 |
| `brake` | 보정된 실제 차량속도에서 계산한 제동거리 |
| `path` | global path 목표 방향 |
| `error` | path와 차량 사이의 횡방향 오차 |

`collision=inf`, `ttc=inf`이면 검사 범위 내 충돌이 없다는 뜻입니다.
`CLEAR steer_state=AVOID_*`는 감속 없이 안전한 회피 조향 중인 상태이며,
`BRAKE/STOP steer_state=AVOID_*`는 회피 방향은 유지하지만 그 방향에도 충돌 위험이 있어 동시에 감속하는 상태입니다.

RViz의 색상 궤적은 목표 조향을 전체 검사거리 동안 고정한 원호가 아닙니다.
최신 조향 명령을 실제 바퀴각으로 즉시 간주하지 않고 `steering_transition_time`으로
현재 조향을 먼저 추정한 뒤, 그 `current`에서 `steer`로 변하는 현재 제어주기의
물리 궤적만 표시합니다. 파란 화살표는
그 이후의 선택 목표 방향입니다. `detect`는 LiDAR 탐지거리, `rollout`은 화면에
표시된 전환 궤적의 길이입니다.

속도가 느릴 때는 상태에 따라 원인을 구분합니다.

- `CLEAR`인데 느림: 일반 속도 파라미터 문제
- `BRAKE`가 많아서 느림: 안전한 회피 후보 조건이 엄격하거나 전환 궤적이 실제보다 보수적
- `STOP`이 많아서 느림: TTC가 실제로 짧거나 `speed_calibration_factor`, 현재 조향, 차량 형상 설정이 잘못됨

### 속도 단위 통일

현재 타이어/encoder가 보정되지 않아 test_29에서 localization 이동속도가 `/odom_wheel.linear.x`보다 중앙값 기준 약 `2.57배` 컸습니다. 제어에서는 다음 한 번의 변환만 사용합니다.

```text
실제 차량속도[m/s] = /odom_wheel.linear.x × speed_calibration_factor(2.6)
wheel 출력 명령     = 실제 목표속도 ÷ speed_calibration_factor(2.6)
```

TTC, 제동거리, 조향 전환거리, 속도별 회전반경과 모든 YAML 속도·가속도 파라미터는 실제 SI 단위입니다. `/odom`은 위치와 yaw에만 사용하며 twist 속도는 사용하지 않습니다. `/odom_wheel`이 오래되면 최근 wheel 명령을 같은 2.6배로 환산해 fallback하므로 계산 중간에 raw 단위와 실제 m/s가 섞이지 않습니다. 기존 체감 속도를 유지하기 위해 예전 wheel 단위 파라미터도 `×2.6`해 옮겼으므로, 안전 판정이 같다면 실제 wheel 출력은 이전과 같습니다.

global path의 상대적인 가감속 프로파일은 유지하되 전체 속도를 조정하려면 `path_speed_scale`만 바꿉니다.

```yaml
path_speed_scale: 0.70  # 원본 /global_path.z보다 30% 낮게 주행
```

전체적으로 여전히 빠르면 `0.70 → 0.60`, 지나치게 느리면 `0.70 → 0.80`처럼 `0.05~0.10` 단위로 조정합니다. `max_speed`는 절대 상한이므로 path 전체를 비율로 낮추는 용도로는 `path_speed_scale`이 더 적절합니다.

## 2. 현재 제동거리 기준

제동거리는 다음 식으로 계산됩니다.

```text
제동거리 =
    속도 × steering_transition_time
    + 속도² / (2 × normal_deceleration)
    + trajectory_safety_margin
```

현재 설정은 다음과 같습니다.

```yaml
steering_transition_time: 0.20
normal_deceleration: 7.8
trajectory_safety_margin: 0.05
```

대략적인 제동거리는 다음과 같습니다.

| 보정된 실제 속도 | 제동거리 |
|---:|---:|
| `1.0 m/s` | `0.31 m` |
| `2.0 m/s` | `0.71 m` |
| `3.0 m/s` | `1.23 m` |

검사 horizon 안에 충돌점이 하나라도 있으면 전체 조향 범위에서 회피를 즉시 탐색합니다. 최종 조향은 path 목표속도로 다시 검사합니다. 목표속도가 위험하면 같은 조향이 충돌 없이 주행 가능한 최대 속도를 탐색하며, 조향과 목표속도가 모두 안전할 때만 감속하지 않습니다.

## 3. 장애물을 제대로 회피하지 못할 때

### 정적 장애물을 안전하다고 판단하고 들이받을 때

다음 순서로 하나씩 조정합니다.

1. `candidate_safety_level: 0.4 → 0.35`
2. 그래도 부족하면 `candidate_min_clearance: 0.5 → 0.55`
3. 궤적 자체가 장애물을 놓치면 `trajectory_safety_margin: 0.05 → 0.07`
4. 계산 간격이 너무 거칠면 `trajectory_check_step: 0.05 → 0.03`

| 파라미터 | 변경 | 효과 |
|---|---|---|
| `candidate_safety_level` | 감소 | 주변의 가까운 LiDAR ray를 더 강하게 반영 |
| `candidate_min_clearance` | 증가 | path와 회피 후보 모두 더 넓은 공간만 허용 |
| `trajectory_safety_margin` | 증가 | 직사각형 footprint와 제동거리 모두 증가 |
| `trajectory_check_step` | 감소 | 더 촘촘하게 충돌검사, 계산량 증가 |

가장 먼저 `candidate_safety_level`을 `0.05` 단위로 조정합니다.

### 현재 조향과 반대 방향으로 갑자기 꺾을 때

```yaml
avoidance_steering_change_penalty: 0.3
avoidance_direction_hold_time: 1.0
```

- `avoidance_steering_change_penalty`를 올리면 현재 조향과 기존 목표에서 크게 벗어나는 후보가 덜 선택됩니다.
- `avoidance_direction_hold_time`을 올리면 한번 선택한 회피 방향을 더 오래 유지합니다.

`avoidance_steering_change_penalty`를 크게 올리면 곡선 장애물에서 clearance가 큰 회피각보다 여유가 작은 작은 각을 선택할 수 있습니다. 최소 회피각을 강제하거나 이 penalty를 먼저 올리지 말고, 실제 조향 추정값과 후보 궤적을 확인합니다. hold time도 너무 높이면 유지한 쪽이 막혔을 때 반대쪽 전환이 늦어질 수 있습니다.

현재 방향 고정 해제 조건은 공통 파라미터에서 자동 계산됩니다.

```text
최소 방향 유지시간: avoidance_direction_hold_time = 0.4 s
정면 해제 안전거리: 2 × candidate_min_clearance = 1.0 m
정면 clear 유지시간: 0.5 × avoidance_direction_hold_time = 0.2 s
반대 방향 안전 확인시간: steering_transition_time = 0.2 s
긴급 회피 후 path 복귀 확인시간: steering_transition_time = 0.2 s
```

일반 회피 방향의 좌우 재선택은 같은 장애물에서 한 번만 허용합니다. 다만
고정 방향이 위험해 긴급하게 반대편으로 피한 뒤에는 global path 방향의
직사각형 전환 궤적이 `steering_transition_time` 동안 연속으로 안전하면,
장애물 통과점 해제를 기다리지 않고 별도의 종료 기동으로 path에 복귀합니다.
이 복귀는 추가 회피 방향 변경으로 계산하지 않으므로 오른쪽 회피 후 왼쪽
복귀 같은 S자 기동을 막지 않습니다.

회피를 시작시킨 장애물 포인트는 localization용 `/odom` 좌표로 고정됩니다. 정면 궤적뿐 아니라 global path 궤적이 막힌 경우에도 안전한 detour 방향을 조기에 고정합니다. 차량이 옆으로 비켜 장애물이 현재 정면 궤적에서 사라져도, 저장한 포인트가 차량 후방으로 충분히 지나가기 전에는 처음 선택한 회피 방향을 계속 유지합니다. TTC·제동거리·속도별 회전반경에는 `/odom_wheel × 2.6`으로 보정한 실제 속도를 공통 사용합니다.

고정 방향이 위험해지면 반대쪽의 완전한 충돌 없는 상태가 `steering_transition_time` 동안 유지되어야 전환합니다. 현재 방향의 TTC가 이 시간 이하이면 안전한 반대쪽으로 즉시 전환하지만, 같은 장애물에 대한 좌우 보정은 한 번만 허용합니다. 이후 양쪽 모두 위험하면 기존 방향을 유지한 채 BRAKE/STOP으로 감속합니다.

### 갑자기 나타난 장애물에 반응이 늦을 때

먼저 조향 smoothing을 줄입니다.

```yaml
steering_smooth_window: 3  # 필요하면 2로 감소
```

강조향 결정 자체가 늦으면 다음 값을 조금 낮춥니다.

```yaml
ambiguous_gap_score_margin: 1.8  # 1.5 정도까지 감소 검토
```

`steering_transition_time`을 단순히 반응이 빨라 보이게 하려고 낮추면 안 됩니다. 이 값은 실제 차량의 조향 전환시간을 모델링하는 안전값입니다.

## 4. 회피할 공간이 있는데 너무 감속할 때

### 디버그가 `CLEAR`인 경우

TTC 로직 때문에 느린 것이 아닙니다. 다음 일반 속도값을 확인합니다.

```yaml
speed_factor: 0.78
minimum_non_emergency_speed: 2.6
narrow_gap_max_speed: 5.2
max_speed: 9.1
```

일반 속도를 조금 올리려면 `speed_factor`를 `0.78 → 0.85`처럼 `0.05~0.08` 단위로 올립니다.

안전거리가 짧아도 비상이 아닐 때 속도를 높이려면 `minimum_non_emergency_speed`를 `2.6 → 2.8 m/s`로 조정합니다.

기본 차량 반폭으로 재탐색한 좁은 gap에서만 느리면 `narrow_gap_max_speed`를 `5.2 → 5.7 m/s`로 조정합니다.

### 디버그가 `BRAKE`인 경우

horizon 끝까지 충돌 없는 전환 궤적 후보가 없다는 의미입니다. 실제로 옆 공간이 충분히 안전한 것이 확인됐다면 다음 순서로 완화합니다.

1. `avoidance_steering_step: 1.0 → 0.5`
2. `candidate_min_clearance: 0.5 → 0.45`
3. `candidate_safety_level: 0.4 → 0.5`
4. 실제 조향이 더 빠르다는 측정 근거가 있을 때만 `steering_transition_time: 0.20 → 0.18`

가장 먼저 `avoidance_steering_step`을 줄이는 방법이 안전합니다. 검사할 후보 수는 증가하지만 안전 기준 자체는 낮아지지 않습니다.

`candidate_safety_level`을 올리거나 `candidate_min_clearance`를 내리면 정적 장애물을 다시 놓칠 수 있으므로 bag에서 50초와 110초 구간을 반드시 재확인해야 합니다.

회피가 끝난 뒤 넓은 옆 트랙으로 계속 빠지는 경우에는 `candidate_safety_level`보다 회피 방향 고정 해제를 먼저 확인합니다. 최소 유지시간 이후 global path 방향의 직사각형 전환 궤적이 안전하면 즉시 path 복귀가 허용되며, 안전하지 않으면 기존 회피 방향이 유지됩니다.

### 디버그가 `STOP`인 경우

현재 조향 전환을 포함한 TTC가 `0.20 s` 이하입니다. 이 경우는 안전한 회피 후보가 없는 비상 판정입니다.

`minimum_crawl_speed`를 올려도 STOP 발생 빈도는 줄지 않고 비상 중 속도만 올라갑니다.

```yaml
minimum_crawl_speed: 0.78
```

차량이 너무 느려 조향 자체가 불가능한 경우에만 `0.78 → 1.0 m/s`를 검토합니다.

## 5. path 추종 문제

### 안전한 양쪽 gap 중 path 방향을 잘 선택하지 않을 때

다음 순서로 조정합니다.

```yaml
path_guidance_weight: 0.6  # 0.8까지 증가 검토
path_rejoin_weight: 2.4    # 2.7까지 증가 검토
```

path에서 벗어난 뒤 복귀가 너무 늦으면 `path_rejoin_distance: 0.3 → 0.25`로 내립니다. 더 작은 이탈에서도 rejoin weight가 적용됩니다.

LiDAR 최적 gap보다 조금 점수가 낮은 path 방향도 허용하려면 `path_candidate_min_score_ratio: 0.85 → 0.80`으로 내립니다. `candidate_safety_level`과 `candidate_min_clearance` 안전조건은 그대로 적용됩니다.

### path를 너무 강하게 따라가려고 할 때

```yaml
path_guidance_weight: 0.6       # 0.4까지 감소 검토
path_rejoin_weight: 2.4         # 2.0까지 감소 검토
path_candidate_min_score_ratio: 0.85  # 0.9까지 증가 검토
```

`path_candidate_min_score_ratio`를 높이면 LiDAR-only 최적 gap과 거의 비슷한 후보에만 path 가산점을 허용합니다.

장애물이 path 위에 있을 때 들이받는 문제를 path weight만 낮춰 해결하면 안 됩니다. 이 경우에는 `candidate_safety_level`과 `candidate_min_clearance`를 먼저 확인합니다.

### `path_alignment_sigma` 조정법

현재 값은 `35°`입니다.

```text
path와 0° 차이   → 가산점 100%
path와 35° 차이  → 약 61%
path와 70° 차이  → 약 14%
```

- 값을 줄이면 path 방향과 정확히 가까운 후보에만 가산점이 집중됩니다.
- 값을 늘리면 같은 쪽의 넓은 범위에 path preference가 부드럽게 퍼집니다.

| 증상 | 조정 |
|---|---|
| path 방향으로 조향이 너무 날카롭게 붙음 | `35 → 40~45°` |
| path와 다른 방향도 비슷한 가산점을 받음 | `35 → 25~30°` |
| 안전한 회피 gap을 선택하면서 대략 path 쪽을 유지하고 싶음 | `35°` 유지 |

path 추종 자체를 강하게 하고 싶으면 sigma보다 `path_guidance_weight`를 먼저 올리는 것이 명확합니다.

## 6. 코너와 좌우 gap 선택 문제

### 깊지만 좁은 막다른 공간을 선택할 때

```yaml
gap_width_preference: 0.4  # 0.5까지 증가 검토
gap_score_distance_cap: 5.0  # 4.0까지 감소 검토
```

- `gap_width_preference`를 올리면 깊이보다 주변이 넓게 열린 방향을 선호합니다.
- `gap_score_distance_cap`을 낮추면 아주 깊은 한두 개 ray가 계속 점수를 얻는 것을 제한합니다.

### 정상 코너에서 방향 결정을 너무 늦게 할 때

`ambiguous_gap_score_margin: 1.8 → 1.5`처럼 낮춥니다. 값이 낮을수록 한쪽 gap이 조금만 우세해도 빨리 방향을 결정합니다.

### 늦게 보이는 올바른 반대편 코너를 놓칠 때

`ambiguous_gap_opposite_weight: 2.0 → 2.2`처럼 올립니다.

반대로 정상 코너에서 반대쪽으로 과하게 끌리면 `2.0 → 1.7`로 내립니다. `0.2~0.3` 단위로 조정합니다.

## 7. 벽과 좁은 통로 문제

### 평상시 벽에 너무 붙을 때

```yaml
gap_safety_margin: 0.3  # 0.35까지 증가 검토
```

preferred scan의 장애물 확장이 커져 벽에서 더 떨어진 gap을 선호합니다.

### 좁은 통로를 통과하지 못하고 계속 BRAKE가 걸릴 때

preferred scan에서 기본 scan으로 더 일찍 전환하도록 `gap_fallback_distance: 0.4 → 0.5`로 올립니다.

그래도 안전한 회피 후보가 없으면 `candidate_min_clearance: 0.5 → 0.45`를 검토합니다.

`vehicle_width` 또는 `vehicle_length`를 줄여 해결하면 실제 차량보다 작은 궤적을 허용할 수 있으므로 권장하지 않습니다.

## 8. 조향이 흔들릴 때

다음 순서로 조정합니다.

```yaml
avoidance_steering_change_penalty: 0.3  # 0.5까지 증가
steering_smooth_window: 3               # 4까지 증가
avoidance_direction_hold_time: 0.4      # 0.6까지 증가
```

- `avoidance_steering_change_penalty`: 안전 후보 중 현재 조향과 가까운 방향 선택
- `steering_smooth_window`: 최종 조향값을 시간 평균
- `avoidance_direction_hold_time`: 장애물 회피 방향의 좌우 부호 유지

`steering_smooth_window`을 너무 키우면 갑자기 나타난 장애물에 늦게 반응하므로 보통 `2~4` 범위가 적당합니다.

작은 코너에서도 방향 고정이 자주 걸리면 `avoidance_direction_min_angle: 10 → 15°`로 올립니다. 반대로 방향 고정이 잘 시작되지 않으면 `10 → 7°`로 내립니다.

## 9. 코너 속도 문제

코너에서 너무 빠르면 `friction: 0.75 → 0.65`로 낮춥니다.

코너에서 지나치게 느리면 실제 접지력이 충분하다는 조건에서 `friction: 0.75 → 0.8`을 검토합니다.

`wheel_base`는 실제 차량 치수이므로 튜닝값으로 변경하면 안 됩니다.

예상 궤적이 실제 차보다 안쪽으로 도는 경우에는 먼저 속도별 곡률 모델을 확인합니다.

```text
curvature_gain(v_actual) = clamp(1 / (0.71 + 0.0873 × v_actual²), 0.60, 1.25)
```

이 값은 workspace의 `0818`, `0819_1`, `test_12` bag에서 raw wheel 속도로 fitting한 `0.59`를 실제 m/s 단위로 환산한 값입니다. 기존 raw 속도 구간 `0.35~0.6 / 0.6~0.9 / 0.9~1.2`는 실제 속도 `0.91~1.56 / 1.56~2.34 / 2.34~3.12 m/s`에 해당합니다. 보정 전후 회전반경은 동일합니다. 새 타이어, encoder scale, 조향 링크 또는 wheelbase가 바뀌지 않았다면 임의 조정하지 않습니다.

가속이 너무 느리면 `max_acceleration: 5.2 → 6.5`처럼 조금씩 올릴 수 있습니다. 이 값이 너무 크면 짧은 CLEAR 구간에서 속도가 상승한 직후 강조향이 필요해져 실제 차량이 선회 속도 상한까지 감속할 시간이 없어집니다.

`normal_deceleration`과 `max_deceleration`의 역할은 분리되어 있습니다.

- `normal_deceleration: 7.8`: 안전한 회피 조향이 없을 때 미리 적용하는 감속도이자 제동거리 계산값
- `max_deceleration: 15.6`: TTC가 `steering_transition_time` 이하인 실제 비상상황 감속도

회피 조향을 선택한 뒤에는 path 목표속도로 전환 궤적을 다시 검사합니다. 그 속도에서도 완전히 충돌 없고 선회 속도 한계 이하라면 장애물 옆의 짧은 LiDAR 거리만으로는 감속하지 않습니다. 위험하면 같은 조향에서 안전한 최대 속도를 탐색하고, 계산된 `turn_cap`, `traj_cap`, 제동 cap을 최종 명령의 절대 상한으로 적용합니다. 따라서 안전한 회피인데 느린 경우에는 `speed_factor`보다 RViz debug의 `turn_cap`과 `traj_cap` 중 어느 값이 제한하는지 먼저 확인합니다.

평상시 감속이 너무 급하면 `normal_deceleration: 7.8 → 6.5`로 낮추면 더 일찍, 더 완만하게 감속합니다. `max_deceleration`은 충돌 직전 비상용이므로 실제 차량이 낼 수 있는 최대 감속도 이내로 유지해야 합니다.

## 10. 센서 노이즈와 계산량

### LiDAR가 튀거나 wheel 속도가 순간적으로 비정상일 때

```yaml
scan_filter_window: 5
```

이 값은 LiDAR median filter의 ray 개수에만 사용됩니다. `/odom_wheel`은 TTC와 회전반경이 실제 가속을 늦게 따라가지 않도록 최신 보정 속도를 바로 사용합니다. 값을 올리면 LiDAR 노이즈에는 강해지지만 작은 장애물을 늦게 반영할 수 있으므로 일반적으로 `5`를 유지합니다.

### 계산량이 너무 많을 때

다음 순서로 완화합니다.

```yaml
avoidance_steering_step: 1.0  # 2.0까지 증가
trajectory_check_step: 0.05   # 0.06까지 증가
```

- steering step을 키우면 검사할 후보 수가 줄어듭니다.
- trajectory step을 키우면 후보당 궤적 검사 횟수가 줄어듭니다.
- 너무 크게 하면 좁은 안전 조향이나 작은 장애물을 놓칠 수 있습니다.

현재 `test_13` 2배속에서 9,044 scan 중 8,954개를 처리했으므로 현재 계산량은 실시간 주행에 충분한 수준입니다.

## 11. 임의로 변경하면 안 되는 값

다음 값은 실측값이므로 성능 튜닝용으로 임의 변경하지 않습니다.

```yaml
wheel_base: 0.324
curvature_model_offset: 0.71
curvature_model_speed_squared_gain: 0.0873
curvature_gain_min: 0.60
curvature_gain_max: 1.25
speed_calibration_factor: 2.6
vehicle_width: 0.36
vehicle_length: 0.55
lidar_offset_x: 0.27
lidar_offset_y: 0.0
max_steering_angle: 45.0
steering_feedback_topic: /ackermann_cmd
```

특히 `lidar_offset_x`가 잘못되면 실제 충돌 위치와 계산 궤적이 어긋나 `CLEAR`인데 충돌하는 상황이 생길 수 있습니다.

`steering_transition_time`도 실측 기반 값입니다. 이 값은 다음 동작에 공통 사용됩니다.

- 최신 actuator 명령에서 현재 실제 조향각을 추정하는 응답시간
- 현재에서 목표까지의 조향 전환시간
- 조향 feedback 유효시간
- 제동 반응시간
- STOP 판정 TTC

단순히 감속을 줄이기 위해 낮추면 안 됩니다.

## 12. 권장 튜닝 절차

1. 동일 코드·동일 초기화 조건으로 한 상황을 2~3회 반복해 재현성을 먼저 확인합니다.
2. 직선 중앙, 곡선 직후, 좁고 비대칭인 구간처럼 서로 다른 조건의 증상을 함께 기록합니다.
3. 개별 충돌마다 파라미터를 붙이지 말고, 여러 조건에 반복되는 공통 원인부터 분류합니다.
4. 공통 원인을 해결하는 한 묶음의 변경을 적용한 뒤 같은 전체 test matrix를 다시 실행합니다.
5. 거리값은 `0.05 m`, 시간은 `0.02 s`, weight는 `0.1~0.2` 단위로 바꾸되 안전 gate를 약화했다면 반드시 모든 조건을 재검증합니다.
6. 충돌 여부뿐 아니라 방향 반전, path 복귀, 최저속도 비율을 함께 비교합니다.

현재 `test_13` 기준점은 다음과 같습니다.

| 지표 | 현재 |
|---|---:|
| 전체 평균속도 | `1.214 m/s` |
| gap_obs 평균속도 | `1.102 m/s` |
| gap_obs `0.8 m/s` 이하 | `9.06%` |
| gap_obs `0.5 m/s` 이하 | `5.95%` |
| gap_obs 큰 즉시 방향 반전 | `0회` |
| 처리 scan | `8,954 / 9,044` |

튜닝 후에는 최소한 `gap_obs 큰 즉시 방향 반전 0회`를 유지해야 합니다. 또한 50초와 110초 구간에서 `current`와 `steer`가 반대인데 `CLEAR`로 즉시 통과하지 않는지 확인해야 합니다.

rosbag 재생은 기록된 차량 움직임을 사용하는 open-loop 검증입니다. 최종 판단은 실차 또는 시뮬레이션의 closed-loop 주행으로 확인해야 합니다.

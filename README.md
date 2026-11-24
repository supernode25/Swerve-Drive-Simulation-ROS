# midpro_nav2_ws — midpro 1.4 자율주행 스택의 ROS2 Humble 이식 (시뮬레이션 1단계)

ROS1 `loas_planner`(loas_local_planner + loas_controller)의 로직을 그대로 옮긴 Nav2 컨트롤러 플러그인과,
eband 자리를 대신하는 Smac Hybrid-A* 우회 플래너, Gazebo 없는 2D 시뮬레이터, SLAM/AMCL 설정으로 구성됩니다.
자동주차·도킹·fleet은 범위에서 제외했습니다.

> **빌드 상태**: 이 워크스페이스는 ROS2가 없는 환경에서 Humble 소스(navigation2-humble, 1.1.20)에 맞춰 작성됐고
> **아직 컴파일·실행되지 않았습니다.** Python/YAML/XML은 문법 검사만 통과했습니다. 첫 빌드에서 수정이 필요한
> 부분이 나올 수 있으며, 그 목록을 아래 "확인 필요"에 적어 두었습니다.

## 구성

| 패키지 | 내용 |
|---|---|
| `midpro_nav2_controller` | `nav2_core::Controller` 플러그인 `MidproFsmController`. straight/side_front/front_side/line/detour 로컬 경로 생성 + FOLLOW_PATH/ROTATE_IN_PLACE/FINAL_APPROACH/FINAL_ROTATE FSM + 출력 필터 |
| `midpro_sim` | `fake_robot`(cmd_vel 적분 + 축별 1차 지연, 지도 레이캐스팅 /scan, /clock, 장애물 주입), `detour_bridge`(우회 요청 → Hybrid-A* 액션 → /midpro/detour_path) |
| `midpro_bringup` | `nav2_params.yaml`, BT XML, slam_toolbox 설정, 테스트 지도, RViz 설정, launch |
| `midpro_description` | 스웨브 베이스 URDF(xacro, Gazebo 태그 없음) |

```
fake_robot ── /odom /tf /scan /clock ──▶ Nav2
   ▲                                      ├ planner_server (Smac 2D, 글로벌)          ─ /plan ─┐
   │ /cmd_vel                             ├ controller_server ─ FollowPath: MidproFsmController ◀┘
velocity_smoother ◀── /cmd_vel_nav ───────┤      │ detour 상태: /midpro/detour_request
                                          │      ▼
                                          ├ detour_bridge ──▶ /detour/planner_server (Smac Hybrid-A*, 롤링 costmap)
                                          │      ◀── /midpro/detour_path
                                          ├ behavior_server (wait / backup / spin)   ◀ FollowPath 실패 시 BT 복구
                                          └ amcl + map_server  또는  slam_toolbox
```

## ROS1 → ROS2 대응

| ROS1 (midpro 1.4) | ROS2 (이 워크스페이스) |
|---|---|
| `move_server` / `navi_client` goal 흐름, `goal_info`, `cancel`, `replan` | `NavigateToPose` 액션 + BT (`midpro_nav_to_pose.xml`) |
| `move_base/GlobalPlanner/plan` | `planner_server` (Smac 2D) → `/plan` |
| `loas_local_planner` 상태기계 | 플러그인 내부 `plannerTick()` (20 Hz의 1/2 = 10 Hz) |
| `EBandPlannerROS/global_plan` 경로 참조 | `detour` 상태에서 `/midpro/detour_request` → `detour_bridge` → `/detour/planner_server`(Hybrid-A*) → `/midpro/detour_path` |
| `loas_controller` FSM, 20 Hz | 플러그인 `controlTick()`, `controller_frequency: 20` |
| `loas_costmap` (10 m, 0.05, static/obstacle/inflation, 100/99/95) | `local_costmap` (costmap_2d). 플러그인이 raw cost를 0..100 규약으로 환산해 `near_obstacle_value: 95`를 그대로 사용 |
| yaw 1e-10 비교로 홀로노믹/애커만 판정 | 같은 규칙을 경로 생성 시 1회 계산(`oblique_yaw_tolerance`), `/midpro/path_mode`로 발행 |
| `ctrl_state`, `ctrl_transition`, `path_mode`, `path_transition` | `/midpro/ctrl_state` 등 동일 의미의 토픽 |
| 컨트롤러 내부 LPF/rate limit | 유지 + `velocity_smoother`(가속 0.3 m/s²) 추가 |

## 빌드

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-collision-monitor ros-humble-slam-toolbox \
                 ros-humble-xacro ros-humble-robot-state-publisher python3-pil
cd midpro_nav2_ws
colcon build --symlink-install
source install/setup.bash
```

## 실행

```bash
# 기본: 지도 + AMCL, detour 상태에서는 Nav2 글로벌 경로를 그대로 추종 (추가 서버 없음, 대기 없음)
ros2 launch midpro_bringup sim_nav.launch.py

# FSM만 떼어 보기: 시뮬 ground-truth 측위 (AMCL 없음) — midpro 1.4와 가장 가까운 조건
ros2 launch midpro_bringup sim_nav.launch.py localization:=gt

# Hybrid-A* detour 플래너 서버를 켜서 비교 (CPU 부하 큼)
ros2 launch midpro_bringup sim_nav.launch.py detour:=true

# SLAM (slam_toolbox)로 지도를 만들며 주행
ros2 launch midpro_bringup sim_nav.launch.py slam:=true
```

| 인자 | 기본 | 의미 |
|---|---|---|
| `sim_time` | false | false = 모든 노드 벽시계(권장, 끊김 없음). true = 시뮬이 `/clock` 발행(배속 실행 때만) |
| `detour` | false | detour(ex eband) 경로 출처. false = Nav2 글로벌 경로(기본), true = Hybrid-A* 서버(`/detour`) |
| `localization` | amcl | `amcl` 또는 `gt`(시뮬이 map→odom 직접 발행) |
| `slam` | false | slam_toolbox 사용(지도·AMCL 대신) |
| `world_map` / `map` | ecoproBM_matterport | 시뮬 월드 / Nav2 지도 yaml (test_hall.yaml로 바꿀 수 있음) |
| `init_x` `init_y` `init_yaw` | 34.3 / 0.95 / 0 | 시뮬 초기 위치 = AMCL 초기 추정 |
| `obstacles` | 없음 | 장애물 시나리오 yaml |
| `rviz` | true | RViz 실행 |
| `dash` | true | 속도 대시보드(midpro_dash) 자동 실행 |
| `rviz_dark` | true | 어두운 RViz 뷰 설정(midpro_dark.rviz). 패널까지 어둡게 하려면 아래 qt5ct 설정 |

RViz **Nav2 Goal**로 목표를 주면 주행합니다. **Publish Point** 클릭은 반경 0.15 m 장애물을 추가합니다.
Views 패널에서 `TopDown` / `Orbit` / `FollowRobot` 뷰를 전환할 수 있습니다.
`ros2 service call /sim/reset std_srvs/srv/Trigger`로 초기 위치로 되돌립니다.

상태 확인:
```bash
ros2 topic echo /midpro/ctrl_state        # IDLE / FOLLOW_PATH / ROTATE_IN_PLACE / FINAL_APPROACH / FINAL_ROTATE
ros2 topic echo /midpro/path_mode         # straight|side_front|front_side|line|detour / holonomic|ackermann
ros2 topic echo /midpro/path_transition
ros2 topic echo /clock --once             # 10 s 뒤 다시 찍어 실시간으로 흐르는지 확인
```

### detour 상태의 의미

ROS1에서 eband는 "직선이 막혔을 때 장애물을 피해 변형한 경로"를 FSM에 주는 역할이었습니다. 여기서는
그 역할을 **Nav2 글로벌 경로(Smac 2D, BT가 1 Hz로 재계획)** 가 대신합니다. 글로벌 경로는 이미
장애물을 피해 있고, FSM은 이 경로의 접선 yaw를 따라 애커만식으로 추종합니다(곡선) 또는 홀로노믹으로
평행이동합니다(직선). `detour:=true`로 켜는 Hybrid-A*는 회전 반경 제약이 있는 경로가 꼭 필요할 때의
선택지이며, 설정에 수 초가 걸리고 CPU를 많이 씁니다.

RViz로 먼 목표를 한 번에 찍으면 직선 가시선 검사가 거의 항상 막혀 바로 `detour`(글로벌 경로 추종)로
갑니다. midpro 1.4의 `move_server`가 주던 짧은 웨이포인트 입력과 같은 거동을 보려면 가까운 목표를
찍거나 `NavigateThroughPoses`로 경유점을 주세요.

## 회귀 테스트

절차·판단 기준: `TESTING.md`. 요약: `ros2 run midpro_dash run_variants --repeats 1`로 v028과 v028_narrow, 기능별 변형을 무인 비교.

## 주행 대시보드 (PyQt5 + pyqtgraph)

탭 3개: **그래프**(속도·가속·조향·프로파일), **이벤트(실시간)**(문제가 생기는 순간 시각과 권장 파라미터를 한 줄씩), **장애물 편집**(지도를 클릭해 원·박스 장애물, 왕복·순환 이동 장애물을 시뮬 라이다에 넣음 — costmap이 아니라 스캔에만 존재).

```bash
sudo apt install python3-pyqt5 python3-pyqtgraph        # 한 번만
ros2 launch midpro_bringup sim_nav.launch.py              # dash:=true 기본 → 대시보드가 함께 뜸
ros2 run midpro_dash speed_dash                           # 따로 띄울 때 (--light 밝게, --size 1000x650 크기)
ros2 run midpro_dash speed_dash --window 60 --csv run1.csv   # CSV 기록
```
그래프: 속도(컨트롤러 출력 / 최종 cmd_vel / 실제), 가속도·저크, 조향(cmd wz / 실제 wz / cmd vy), 계획 속도 프로파일 v(s)와 현재 진행 위치.
점수표: 경로 추종(경로와의 거리 RMS)·가속·저크·조향 반전·속도 효율·정지 정밀을 0~100점으로, 종합 등급 A~D와 가장 약한 항목의 조치 제안. goal을 받으면 새로 집계하고 도착하면 고정됩니다(자동 모드). '집계: 수동'으로 바꾸면 시작·정지 버튼으로 원하는 구간만 집계합니다(여러 goal을 잇는 코스, 주행 중에 GUI를 켠 경우).
튜닝 리포트: '튜닝 리포트 생성'을 누르면 이번 주행의 증거를 규칙으로 진단해 어느 파라미터를 얼마로 바꿀지(현재→제안), 이유, 기대 효과, 주의점을 정리한 HTML을 `~/midpro_reports/`에 저장하고 창으로 띄웁니다. 한 번에 하나씩 적용하고 같은 코스를 3회 재측정하는 순서를 권합니다. 리포트는 목표 점수(`--target 90` 기본, `--target 95`처럼 숫자 지정)에 도달하거나 점수가 수렴하면 '고정'을 권하고 멈추며, 같은 진단이 2회 이상 재현될 때만 적용을 권합니다. 파라미터로 안 잡히는 항목에는 코드 수정 위치와 방법을 적습니다.
수치: FSM 상태, 경로 상태, 속도 제한 사유, 마지막 전이, 실제·명령 속도, 가속도, |a| 최대(5 s), 가속·저크 RMS(10 s), 조향 반전/분, 경로 진행 거리, 목표 후 경과, 현재 프로파일 상한. 일시정지·통계 초기화·밤 모드·창 길이 조절.

## 방향 인지 안전 계층 (midpro_safety/obstacle_guard)

```
controller -> /cmd_vel_nav -> obstacle_guard -> /cmd_vel_cm -> velocity_smoother -> /cmd_vel
```
- 각 방향(전진·후진·좌·우·좌회전·우회전)마다 "그 방향으로 차체를 밀면 몇 m 뒤 닿는가"를 계산해 그 방향 속도만 제한
- 전진 허용 속도 v = √(2·brake_acc·(앞 거리 − stop_front − 지연)) → 항상 stop_front(10 cm) 앞에서 멈출 수 있는 속도
- 옆 벽은 전진·후진을 제한하지 않음. 옆걸음과 그 벽 쪽으로 차체가 휘는 회전만 제한
- 설정: `src/midpro_safety/config/obstacle_guard.yaml` (footprint는 costmap과 같게)
- 확인: `ros2 topic echo /obstacle_guard/state` (방향별 허용 속도·거리), RViz 'Obstacle Guard'
- 기존 nav2 collision_monitor로 비교: `safety:=monitor`

## RViz 멈춤 방지

RViz는 그리기와 메시지 처리를 한 스레드에서 하므로, 무거운 디스플레이가 몇 초씩 막으면 "응답 없음"이 뜹니다.
- Path 디스플레이의 **Pose Style은 None**(Lines만). Arrows는 162 m 경로에서 3천 개 넘는 화살표를 재계획마다 다시 만듭니다. 필요하면 'Local Plan poses (debug, heavy)'만 잠깐 켜기
- **글로벌 costmap 디스플레이는 필요할 때만** 켜기(180만 셀). 켤 때 Update Topic(`/global_costmap/costmap_updates`) 유지
- 원격 데스크톱(RustDesk 등)에서는 소프트웨어 렌더링이 되기 쉬움: `glxinfo -B | grep -i renderer` 결과가 `llvmpipe`면 GPU를 안 쓰는 상태 → 로컬 모니터에서 띄우거나 위 두 디스플레이를 끄기
- `--symlink-install`에서 RViz **Save**는 src의 설정 파일을 덮어씀. 개인 설정은 `File > Save Config As`로 다른 이름에 저장

## RViz 어둡게 보기

뷰(지도 영역)는 `rviz_dark:=true`(기본)로 어두워집니다. 툴바·패널 같은 Qt 위젯은 RViz 옵션이 아니라 Qt 테마를 따르므로 한 번만 설정합니다.
```bash
sudo apt install qt5ct
qt5ct            # Appearance → Style: Fusion, Palette: 'darker' 선택 후 저장
```
launch가 `QT_QPA_PLATFORMTHEME=qt5ct`를 넘기므로 그 다음부터 패널도 어둡게 뜹니다. GNOME이면 `gsettings set org.gnome.desktop.interface color-scheme prefer-dark`로도 비슷한 효과가 납니다.

## 실제 현장 지도(ecoproBM_matterport)로 돌리기

`midpro_bringup/maps/`에 올려주신 `ecoproBM_matterport.{yaml,pgm}`(1975×915 px, 0.05 m → 98.75 m × 45.75 m,
origin (−28.65, −39.05))을 넣어 두었습니다. 시뮬 월드와 Nav2 지도에 같은 yaml을 씁니다.
```bash
ros2 launch midpro_bringup sim_nav.launch.py \
    world_map:=$(ros2 pkg prefix midpro_bringup)/share/midpro_bringup/maps/ecoproBM_matterport.yaml \
    map:=$(ros2 pkg prefix midpro_bringup)/share/midpro_bringup/maps/ecoproBM_matterport.yaml \
    init_x:=34.3 init_y:=0.95 init_yaw:=0.0 obstacles:=""
```
- `init_*`는 pgm에서 계산한 자유 공간 안의 한 지점(가운데 통로)입니다. RViz **2D Pose Estimate**로 옮겨도 됩니다.
- 시뮬은 지도의 미확인(회색, 전체의 25%) 영역을 벽으로 취급합니다(`unknown_as_occupied: true`).
- `ecoproBM_matterport_clean.{yaml,pgm}`은 6 px 미만의 고립된 점(921 px, 전체 점유 셀의 0.4%)을 지운 사본입니다.
  통로 안 점 잡음이 inflation으로 부풀어 경로를 막으면 이쪽을 쓰세요. 원본과 거동 차이는 거의 없을 것입니다.
- `ecoproBM_matterport.pcd`는 **Git LFS 포인터(134 B)** 입니다. 실제 포인트클라우드(약 199 MB)는 원본 저장소에서
  `git lfs pull`로 받아야 하며, 3D 로컬라이제이션 테스트는 그 파일이 있어야 가능합니다.

## 시나리오 (obstacles_example.yaml)

1. 직선 위 원형 장애물 → `side_front` 또는 `front_side` 우회 후 `straight` 복귀
2. 복도 일부를 막는 박스 → `line` 통과 시도
3. 복도 전체 차단(주석 해제) → `detour`(Hybrid-A*) → 5 s 안에 경로가 없으면 FollowPath 실패 → BT 복구(costmap clear → wait → backup → spin)

## 확인 필요 (빌드/첫 실행 때)

- [ ] 플러그인 컴파일 (ROS2 없는 환경에서 작성). 특히 `tf_->transform(...)`, `LifecyclePublisher` 활성화 순서
- [ ] `inflation_layer.cost_scaling_factor`: 외접 반경 지점의 비용이 0..100 스케일로 약 95가 되도록 조정. 우회 판정이 ROS1과 같은 거리에서 일어나는지 비교
- [ ] `detour.DetourPlanner.minimum_turning_radius: 0.40` [가정]. 애커만 모드 조향 한계로 교체
- [ ] 두 번째 `planner_server`(네임스페이스 `/detour`)의 토픽 remap과 lifecycle 전환이 정상인지
- [ ] `fake_robot`의 `tau_*`, `acc_*` [가정]. CAN 로그로 조향 정렬 지연을 추정해 교체
- [ ] AMCL 초기 pose가 시뮬 초기 pose와 같은지 (둘 다 1.5, 1.5, 0)
- [ ] SLAM 모드: Nav2 global costmap이 slam_toolbox `/map`을 받는지 (`map_subscribe_transient_local: true`)
- [ ] `FINAL_ROTATE` 종료와 Nav2 `goal_checker`(0.02 m / 0.025 rad) 중 어느 쪽이 먼저 끝나는지 확인

## 의도적으로 바꾼 점

- eband → Hybrid-A* 우회 경로 (경로만 받아 FSM이 추종하는 구조는 동일)
- goal_info 문자열 대조 제거 (Nav2에서는 경로의 마지막 pose가 곧 goal)
- 홀로노믹/애커만 판정을 경로 생성 시 1회 계산하고 토픽으로 발행 (규칙은 동일, 1e-10 → 1e-4 rad)
- 모든 토픽 이름 `/midpro/...`로 정리

## WSL 메모

WSL2 + Ubuntu 22.04. Windows 11이면 WSLg로 RViz가 바로 뜹니다. 렌더링이 깨지면 `export LIBGL_ALWAYS_SOFTWARE=1`.
실기나 Windows 쪽 프로세스와 DDS 통신이 필요하면 mirrored networking 모드가 필요합니다.

## 변경 이력

- v0.81 (path conditioner)
  - 경로 정리기(`path_conditioner_enabled`, 기본 켬, 대시보드 체크박스): setPlan 시 글로벌 costmap(`/global_costmap/costmap_raw`)과 실제 footprint로 플래너 경로를 1회 정리 — ① 출발 구간이 옆(45~135°)으로 나가면 헤딩 유지 직선 평행 이동으로 교체(끼인 출발은 처음 0.3 m 탈출 허용) ② 통로 직선화(여유 6→3→1 cm) ③ 목표점 회전이 불가하고 경로 위에 준비 지점이 없으면 목표 축(뒤/좌/우) 후보로 경로 끝을 재구성 → 컨트롤러의 목표 전 정렬이 그 지점을 사용. 결과는 `/midpro/conditioned_plan`(RViz 'Conditioned Plan', 연파랑)
  - 회귀 시나리오 `p_u_pocket_park`(ㄷ자 안쪽 주차), `p_wall_facing_slot`(벽 정면 자리, 경로 밖 준비 지점 필요)
  - 오프라인 검증: `src/midpro_nav2_controller/tools/conditioner_twin/run.py`(같은 알고리즘의 파이썬 버전, 4개 배치)
- v0.80 (sideways shift first)
  - 경로가 옆(45~135°)으로 나가고 그 옆걸음이 footprint 검사를 통과하면, 회전 공간이 있어도 **헤딩 유지 옆걸음을 먼저**(이전: 주행 중엔 크랩이 마지막 순위라 전진/후진 정렬 호를 먼저 시도 → 랙 옆에서 차체가 돌며 끼임). 통로 중앙선으로 옮겨 가는 글로벌 경로의 첫 구간(헤딩과 120° 차이)에서 반대 방향으로 틀다 끼던 문제
- v0.79 (side/front false trigger)
  - side_front/front_side는 막힌 지점이 로봇 `side_trigger_dist`(2.5 m) 안일 때만. 멀리 있는 벽 모서리가 목표까지의 직선을 스치면(목표가 멀 때) 주변에 장애물이 없어도 직각 경로를 그렸다 복귀하던 버그 — midpro는 목표가 가까운 경유점이라 '직선 막힘 = 코앞 장애물'이었음
- v0.78
  - 컴파일 오류 수정: const 함수(speedGovernor)에서 쓰는 `tight_now_` 캐시를 mutable로
- v0.77 (holonomic 'looking around')
  - 홀로노믹 경로의 방향 기준(next_yaw)도 10 cm 앞 점에서(이동 방향은 v0.69에서 바꿨으나 방향 기준은 0.4 m 앞을 써서, side 경로 끝에서 목표 방향으로 튀어 회전↔추종이 틱마다 왕복)
  - 목표 전 정렬 회전 단계의 최소 각속도 하한 제거(허용 오차 근처 리밋 사이클), 정렬 완료 기준 1.7°
- v0.76 (tight-passage tracking)
  - 좁은 구간(앞 경로 여유 < tight_clearance)에서는 애커만 조향 대신 **방향 유지 + 옆걸음 보정** 법칙(`tight_track_*`): 차체를 통로 방향에 고정(≤0.15 rad/s 미세 보정), 좌우 오차는 vy(≤0.08 m/s)로. 조향으로 맞추면서 긴 차체가 벽 쪽으로 휘어 '왼쪽으로 튕기며 못 가던' 문제 — ROS1 직선/line 구간의 홀로노믹 추종을 좁은 구간에 한정해 복원
- v0.75 (narrow passages)
  - 통과 판정은 패딩 없는 기하(`footprint_padding` 0.0, narrow·current), 물리적 여유 3 cm는 obstacle_guard가 라이다 점 그대로 지킴(이전엔 격자 2.5 cm + 패딩 3 cm + 가드 여유가 이중으로 쌓여 폭+10 cm 틈이 간헐적으로 '막힘')
  - 좁은 구간 판정 기준을 패딩과 분리(`tight_clearance` 0.06)
  - 컨트롤러가 `/obstacle_guard/state`의 정면 거리(점 기반, 정확)를 구독: costmap이 정면 막힘이라 해도 가드 거리가 30 cm 이상 더 길면 통과(`trust_guard`; 진행 방향 ±10° 안의 직선/line 판정에만)
  - 컴파일 경고 2건 제거
- v0.74 (review fixes)
  - 목표 전 정렬: 준비 지점 단계의 로컬 경로를 준비 지점에서 끝냄(이전엔 목표까지 이어 붙여 기준점이 목표 쪽을 미리 겨냥 → 모서리를 잘라 정지 실패), 정렬 중 ROTATE_IN_PLACE 허용(틱마다 상태가 왕복하던 문제)
  - 안전 가드 옆 정지 여유 5 → 3 cm(통과 판정 패딩과 동일; 벽 5 cm 옆 주차 목표에 가드가 막지 않도록)
- v0.73 (goal staging)
  - 목표 전 정렬(`goal_staging`, 기본 켬, 대시보드 체크박스): 도착 방향 → 목표 방향 회전이 목표점에서 닿으면, 경로를 0.6~3 m 거슬러 '거기서 돌 수 있고 방향 유지한 채 목표까지 밀고 들어갈 수 있는' 가장 가까운 준비 지점에서 정지 → 회전 → 슬라이드 진입. 외벽 측면 주차·ㄴ/ㄷ자 구간에서 목표점 제자리 회전으로 끼던 문제
  - FINAL_ROTATE 안전장치: 회전 궤적이 닿으면 돌지 않고 경고(이전엔 무조건 회전)
  - 회귀 시나리오 `p_wall_side_park`
- v0.72 (side/front back on)
  - midpro side_front/front_side를 주행 중 다시 사용(`side_front_enabled`, 기본 켬, 대시보드 체크박스). v0.41에서 대각선 끼임 때문에 막았던 분기 — 원인이던 조준점은 v0.69에서 복원됐으나 분기는 그대로 막혀 있어 홀로노믹이 사실상 동작하지 않던 문제
  - side/front 구간 충돌 검사를 실제 이동 자세(방향 유지 크랩)의 footprint로(이전엔 진행 방향으로 돌린 footprint → 0.9 m 길이를 0.5 m로 보고 앞뒤 걸림을 놓침)
  - `transit_holonomic`은 '직선까지 홀로노믹'(midpro 원래 직선 크랩) 의미로 분리, 기본 꺼짐
- v0.71
  - v0.63의 'RViz 2D Pose Estimate로 시뮬 로봇 순간이동' 기능 제거(위치 문제의 원인). 2D Pose Estimate는 Nav2 원래대로 AMCL 위치만 재설정. 시뮬 로봇 이동은 `/sim/set_pose`(레이아웃 불러오기·회귀 러너)로만
- v0.70 (teleport / localisation)
  - 순간이동 후 로컬·글로벌 costmap 비우기를 서비스가 응답할 때까지 3 s 재시도(이전: 준비 안 됐으면 건너뜀 → 옛 위치 장애물이 odom 좌표로 새 위치 주변에 잔류)
  - AMCL 모드: 순간이동을 마친 0.3 s 뒤 실제 위치로 `/initialpose` 재발행 → AMCL이 옛 위치 스캔으로 초기화되는 경합 제거
  - `/initialpose` 좌표계가 map이 아니면 순간이동하지 않고 경고(AMCL도 무시하므로 어긋남 방지)
  - 대시보드 상태 '측위 오차'(AMCL 추정 vs 시뮬 실제 위치, 20 cm·5° 초과 시 경고)
- v0.69 (ROS1 holonomic law restored)
  - 홀로노믹(oblique) 경로는 `oblique_lookahead`(0.10 m, ROS1 path[1]) 앞 점을 향해 주행 — 두 구간 경로(옆→직진, 측면 진입)를 구간별로. 애커만은 0.4 m 기준점 유지. 이식 때 기준점 추종으로 통일하면서 모서리를 대각선으로 지르던 퇴행 복원
  - 분석 문서 `docs_sim_issue_analysis.html`: 원본 설계 vs 이식본, 시뮬 복합 이슈 → 근본 원인 → 조치 버전, 남은 과제
- v0.68 (orthogonal manoeuvre)
  - 직각 회피(`orthogonal_avoid`, 기본 켬, 대시보드 체크박스): 코앞(1.2 m, 정지 중이면 2.5 m)에 장애물이 있고 목표가 그 너머일 때, 방향을 유지한 채 옆으로 필요한 만큼만 평행 이동 → 모서리에서 정지(바퀴 재정렬) → 직진 → 목표가 보이면 정상 로직 복귀. midpro side_front를 순수 직각으로 다시 구현(기존 대각 크랩이 모서리에 끼던 문제 없음). 옆 이동 거리는 양쪽에서 가장 작은 값, 목표 쪽 우선. 이동체로 판정된 장애물에는 동작 안 함(대기 우선)
  - 각 단계는 실제 footprint로 검사(옆 경로·직진 구간), 막히면 즉시 중단 후 정상 로직. 25 s 시간 제한, 탐색 2 Hz
- v0.67
  - description.launch: URDF를 셸 명령(`xacro <경로>`) 대신 xacro 파이썬 API로 생성 → 작업 폴더 경로에 공백·괄호가 있어도 동작(예: 브라우저 중복 다운로드로 생긴 "midpro_nav2_ws (6)")
- v0.66 (direction-aware safety: midpro_safety)
  - 새 패키지 `midpro_safety` / `obstacle_guard`: ROS1 obstacle_detector·obstacle_filter·laser_filter·obstacle_stop 설계를 ROS2로 이식·보완. 장애물 쪽으로 가는 움직임만 제한(옆 벽은 전진을 막지 않음). nav2 collision_monitor(방향 무관 전체 정지)를 대체해 cmd 체인에 기본 탑재(`safety:=guard`), `safety:=monitor`로 기존 방식
  - 보완점: 각도 구역(라이다 원점 기준) → base_link 기하(라이다 오프셋 반영), 구역 켜짐/꺼짐 → 방향별 '닿기까지 거리', 시간 기반 감속 → 거리 기반 v=sqrt(2a(d−여유)), 대각 이동 합성 방향 검사, 회전은 실제 회전 스윕 각도(멀어지는 회전은 제한 안 함), 한도는 즉시 감소·천천히 회복, 차체 필터는 바깥 3 cm 유지, 스캔 끊기면 정지, 60 s 차단 알람
  - RViz 'Obstacle Guard'(footprint·정지 영역·막힌 방향 화살표), 대시보드 상태 '안전 가드'
- v0.65
  - 대시보드 KeyError 'obstacle' 수정(장애물 대응이 가장 약한 항목일 때 안내 문구 누락 → 매 갱신 예외)
  - 경로 당겨 펴기를 틈 폭에 맞춰 단계적으로(여유 6 → 3 → 1 cm), 로봇이 이미 틈 안이어도 동작 → 폭+10 cm 틈에서 틈 축 직선 경로·진입 전 정렬. 틈 안 S자 경로를 따라가다 차체가 틀어져 끼던 문제
  - 궤적 필터의 끼임 탈출 후보: 틈 방향으로 미세 정렬(≤0.12 rad/s), 차체 축 방향 직진 후진/전진(8 cm/s)
- v0.64 (paint-like editing)
  - 장애물 편집: 장애물 클릭 → 선택(파란 강조 + 손잡이), 끌어서 이동, 모서리 손잡이(양방향)·변 손잡이(가로/세로)로 크기, 둥근 손잡이로 회전, 원은 반지름 손잡이. 빈 곳 클릭은 선택 해제 후 추가. 편집도 Ctrl+Z로 되돌림. 이동 장애물은 이동 경로가 함께 옮겨짐
  - 시뮬 `update` 명령(위치·크기·각도). 지도 기본 우클릭 메뉴 끔(우클릭 = 삭제)
- v0.63
  - 시뮬: RViz '2D Pose Estimate'(/initialpose)로 시뮬 로봇이 실제로 그 위치로 이동(`teleport_on_initialpose`, 기본 true). 이전엔 gt 모드에선 무시, AMCL 모드에선 측위만 옮겨져 실제 위치와 어긋남
  - 순간이동(/initialpose·/sim/set_pose) 직후 로컬 costmap 비움(odom 기준이라 옛 위치의 장애물이 새 위치 주변에 남던 문제). 같은 요청이 두 토픽으로 오면 1회만 처리
- v0.62 (obstacle layouts)
  - 장애물 편집 '레이아웃 저장 / 불러오기': 장애물 전체(모양·크기·각도·이동)·로봇 위치·마지막 목표를 `~/midpro_reports/obstacle_layouts/<시각>_<이름>.yaml`로 저장. 불러오기 시 배치 재생성, 로봇 이동(선택), 목표 전송(선택) → 같은 테스트 재현. Ctrl+Z 한 번에 불러오기 전 배치로 복귀. 더블클릭 불러오기, 파일 삭제
  - 저장 파일 = 회귀 시나리오 파일: `ros2 run midpro_dash run_regression --scenarios <파일>`로 자동 반복. 러너는 시작 위치 없는 시나리오도 처리
- v0.61 (string pulling)
  - 추종 경로 당겨 펴기(`shortcut_dist` 3.5 m): 앞 3.5 m 안에서 차체가 6 cm 여유로 직선 주행 가능한 가장 먼 경로점까지를 직선으로 교체(매 틱). 통로 폭이 박스 등으로 좁아졌다 넓어질 때 그리드 경로 중앙선이 출렁여 조향·감속 장치가 연쇄 반응하던 문제
  - 궤적 필터 예측 1.0 → 0.7 s (좁은 통로에서 비관적 감속 감소)
- v0.60
  - 장애물 편집: Ctrl+Z 되돌리기(추가 취소·삭제 복원·전체 삭제 복원, 여러 단계), '되돌리기' 버튼. 편집기가 ID를 직접 부여
  - 선택 삭제 개선: 목록 항목에 ID 저장(글자 파싱 제거), 선택 시 지도에서 강조·목록 하이라이트, 선택 없이 누르면 안내, 1.5 s 뒤 시뮬 반영 확인 결과 표시. Delete 키, 지도 우클릭 삭제
  - 'costmap 정리' 버튼: 지운 장애물 자국을 로컬·글로벌 costmap에서 즉시 제거
- v0.59
  - 대시보드: 그룹 상자에 테두리·제목 영역(제목이 첫 줄과 겹치던 문제), 집계 버튼 '자동 집계/수동 집계'로 축약(잘림), 오른쪽 패널 300 px
- v0.58
  - 장애물 편집: 모양에 '사각형(가로·세로·각도 지정, 클릭 1회)', 팔레트 프리셋 1.2x1.0 / 1.2x0.8(EUR) / 1.1x1.1(T11) 추가. 모든 모양에 정지·왕복·순환 이동 적용(사각형은 각도 유지). '팔레트 투하 시나리오' 버튼(로봇 4 m 앞, 진행 방향에 직각). 목록에 크기·각도 표시
- v0.57
  - '장애물에서 떨어지기' 동작(`back_off`): 회전 공간이 없고 방향 오차가 클 때 차체 근처 가장 가까운 장애물 반대 방향으로 0.12 m/s 옆걸음해 회전 공간 확보 후 정렬. 장애물 옆에 붙은 채 틀어진 자세에서 전/후진 정렬이 모두 막혀 boxed_in으로 끝나던 문제
  - 궤적 안전 필터를 narrow·current에서 다시 켬(v0.33 이후 경로를 잘 따라갈 땐 개입 안 함), 대시보드 주행 옵션 '궤적 안전 필터' 체크박스(실시간)
- v0.56 (obstacle tuning feedback)
  - 컨트롤러 `/midpro/block_event`(감지·이동체 판정·우회 전환[고정물/지속]·해소·실패, 거리·대기 시간). 대시보드: 수집 + 라이다 실시간 차체 여유, 점수표 7번째 '장애물 대응'(늦은 판단·대기 후 우회·실패·근접 감점)
  - 실시간 이벤트: 판단 늦음(<2.5 m), 대기 후 우회, 긴 대기 후 해소, 대안 없음 실패, 너무 가까움 → 각각 파라미터 현재→제안
  - 튜닝 리포트: 장애물 대응 진단을 기존 주행 진단과 한 목록·한 우선순위로, '장애물 대응' 절(통계 + 이벤트 타임라인), 합격 기준표에 항목 추가
  - 회귀: 판단 거리·대기 후 우회·실패·장애물 점수 열
- v0.55 (early decision)
  - 막힘 판단을 앞 4.5 m에서(`block_check_dist`, 이전 2 m), 1 s 관찰로 분류: 충돌 footprint 안 장애물 셀 중 로봇에 가장 가까운 셀이 움직이지 않으면 고정물(팔레트·구조물) → 대기 없이 그 자리에서 우회 경로로 전환, 움직이면 이동체 → 일찍 감속, 정지는 1.2 m 앞(`yield_stop_margin`), 3 s 넘게 남으면 우회
  - 스위칭 꺼짐으로 무시하던 BT의 최신 경로도 보관 → 글로벌 costmap이 멀리서 본 팔레트에 대한 우회 경로를 막힘 감지 즉시 사용. 보관 경로가 같은 막힘을 지나면 사용 안 함
  - 회귀 시나리오 `w_pallet_drop`(주행 중 4.6 m 앞 팔레트 등장)
- v0.54
  - yield 인계 수정: 대기 중 BT가 보낸 우회 경로를 보관했다가 3 s 뒤에도 막혀 있으면 실패 처리 없이 그 경로로 전환. 이전엔 BT가 이미 유효한 우회 경로를 보냈다고 여겨 재전송하지 않아 FollowPath 실패 → 로컬 costmap 비우기(장애물 순간 소실) → 옛 경로 재개 → 재대기 → 복구 동작까지 이어져 10 s 이상 정지
- v0.53
  - RViz 경로 색: 전역 경로 골드 옐로 #F2B705(0.05 m 띠), 로컬 경로 라임 #A6E22E(0.04 m), Hybrid 우회 경로 코럴 오렌지 #FF8A3D(0.03 m). Billboards 띠, 화살표 없음
- v0.52
  - 대시보드 그래프 세로축: 최소 표시 범위 고정(속도·프로파일 0~vmax+0.1, 가속 ±0.6, 조향 ±0.3, 데이터가 넘을 때만 확장), 왼쪽 축 폭 40 px 통일. 자동 확대가 0.5999929… / 1e-36 같은 잡음까지 확대해 눈금이 길어지고 열이 어긋나던 문제
- v0.51 (RViz freeze)
  - RViz 멈춤 완화: 모든 Path 디스플레이 `Pose Style: None` 명시(긴 경로의 Arrows는 재계획마다 수천 개 메시 재생성), 글로벌 costmap 디스플레이는 기본 꺼짐 + Update Topic 사용, goal footprint 마커는 결과가 바뀔 때만 발행
  - RViz 사용 기준: README '## 방향 인지 안전 계층 (midpro_safety/obstacle_guard)

```
controller -> /cmd_vel_nav -> obstacle_guard -> /cmd_vel_cm -> velocity_smoother -> /cmd_vel
```
- 각 방향(전진·후진·좌·우·좌회전·우회전)마다 "그 방향으로 차체를 밀면 몇 m 뒤 닿는가"를 계산해 그 방향 속도만 제한
- 전진 허용 속도 v = √(2·brake_acc·(앞 거리 − stop_front − 지연)) → 항상 stop_front(10 cm) 앞에서 멈출 수 있는 속도
- 옆 벽은 전진·후진을 제한하지 않음. 옆걸음과 그 벽 쪽으로 차체가 휘는 회전만 제한
- 설정: `src/midpro_safety/config/obstacle_guard.yaml` (footprint는 costmap과 같게)
- 확인: `ros2 topic echo /obstacle_guard/state` (방향별 허용 속도·거리), RViz 'Obstacle Guard'
- 기존 nav2 collision_monitor로 비교: `safety:=monitor`

## RViz 멈춤 방지' 참고
- v0.50 (goal footprint)
  - 목표 footprint 투영 노드 `goal_footprint`(launch `goal_viz:=true` 기본): 플래너 `/plan` 끝점(허용 오차 반영된 실제 도착점)에 로봇 footprint를 그리고 글로벌 costmap으로 진입 판정 — 파랑 OK(여유 ≥ straight_clearance), 주황 TIGHT(패딩은 통과), 빨강 BLOCKED. 마지막 1.5 m 접근 구간 footprint 3개, 패딩 윤곽, 방향 화살표, 여유 거리 텍스트. RViz 디스플레이 'Goal Footprint', 대시보드 상태에 '목표 진입'
  - 버그: 장애물 상태(String)와 RViz 마커(MarkerArray)가 같은 토픽 `/sim/obstacles`를 쓰던 충돌 → 상태는 `/sim/obstacle_state`
- v0.49 (yield)
  - 경로가 막히면 바로 우회하지 않고 대기: 막힘이 `block_persist_time`(3 s) 동안 계속될 때만 재계획·우회. 그 전에는 경로 유지, 막힌 지점 0.6 m 앞까지 감속·정지(사유 `yield`). 왕복하는 보행자 때문에 우회↔원경로가 번복되던 문제
- v0.48 (goal ambiguity loop)
  - 목표가 얇은 장애물 바로 너머일 때 '정확한 목표(먼 우회 162 m)' ↔ '허용 오차 안 가까운 점(8.8 m)' 경로가 교대로 나오며 갇히던 루프 수정: 끝점 0.6 m 이내는 같은 목표(`goal_same_tol`), 같은 목표에서 새 경로가 절반 이하면 스위칭 꺼짐이어도 채택(`plan_switch_force_ratio`), 채택 시 컨트롤러 최종점도 새 끝점으로(정지·리셋 없음)
  - 전역 costmap 마킹·지우기 범위를 11.5 m로 통일(10/12의 비대칭이 범위 경계에서 장애물을 깜빡이게 함)
  - 긴 경로에서 20 Hz 미달: 변환한 전역 경로 캐시(0.5 s) + 가까운 점 창 탐색
- v0.47
  - launch 기본 프로파일 `v028` → `v028_narrow`(footprint 최소 구성). 비교 기준은 `profile:=v028` 명시
- v0.46 (steering limit cycle)
  - 최소 각속도 하한(`min_angular_vel` 0.05)을 제자리·도착 회전에서만 적용(`min_angular_in_place_only`). 주행 중에는 0.01 미만 보정만 0(`steer_deadband`). 직진 중 ±0.05 네모파 조향 진동(약 1 s 주기)의 원인
  - 리포트·이벤트: 조향 명령이 ±min_angular_vel에 붙어 있으면 '하한 진동'으로 진단(이전엔 k_prop 하향을 잘못 제안)
- v0.45
  - 장애물 편집 탭: 오른쪽 상태 패널 숨김, '수치'를 장애물 목록 아래로 이동, 목록 높이 축소, 측면 열 250 px 스크롤 — 창 690x430 유지
- v0.44 (compact dashboard)
  - 대시보드 기본 창 690x430(기존의 1/4), `--size WxH`로 지정. 오른쪽 패널은 스크롤 영역(작은 창에서도 안 잘림), 제어·리포트·주행 옵션을 맨 위로. 라벨 축약(설명은 툴팁), 9 pt 글꼴, 어두운 배경에서 체크박스 표시
- v0.43 (curve vs straight)
  - line 제한(목표 근처만) 제거. 직선이 통과 가능하지만 여유(15 cm) 부족일 때: 글로벌 경로가 앞 3 m에서 여유를 유지하면 곡선 추종, 곡선도 좁으면(좁은 복도) 직선 통과(line, midpro 원래 목적) — `prefer_comfortable_curve`
- v0.42 (straight clearance)
  - footprint 모델에서 직선(straight)을 '통과 가능'(차체+6 cm)만으로 고르던 문제: 선택은 여유 기준 `straight_clearance` 0.15 m로, 통과 가능 판정은 그대로(3 cm). detour→straight 복귀도 여유 기준. 최소 여유 직진(line)은 목표 1.5 m 안에서만(`line_only_in_goal_zone`, v028 기준 프로파일은 끔) — 장애물에 붙어 직진하던 원인
- v0.41 (holonomic policy, switching)
  - 주행 중·장애물 회피는 정렬 주행(애커만, 경로 접선): 직선이 막히면 midpro의 측면/전방 2구간 홀로노믹 경로 대신 글로벌 경로(곡선)를 정렬해서 추종. 홀로노믹은 출발 시 NEAR 탈출, 목표 1.5 m 안(장애물 옆·측면 주차), 그리고 회전·정렬 다 안 될 때 최후 수단(크랩)만. `transit_holonomic: true`면 이전 동작
  - 경로 스위칭 기본 꺼짐(`plan_switching: false`): 같은 목표의 재계획은 기존 경로가 막히거나 벗어났을 때만 교체. 켜면 15 % 즉시 / 5 % 2회 연속 교체
  - 두 스위치를 런타임 변경 가능(플러그인 동적 파라미터 콜백). 대시보드 '주행 옵션' 체크박스, 또는 `ros2 param set /controller_server FollowPath.plan_switching true`
- v0.40
  - 장애물이 사라져도 최단 경로로 복귀하지 않던 회귀 수정: (1) 모든 costmap 스캔 소스에 `inf_is_valid: true` — 시뮬 라이다는 12 m 안 무반사 시 inf를 내고, Nav2는 기본적으로 inf 광선을 무시해 70 m 통로 축 방향 셀(장애물이 있던 자리)이 영원히 안 지워졌음 (2) 히스테리시스: 15 % 이상 짧으면 즉시, 5 % 이상 짧은 상태가 재계획 2회 연속이면 교체
  - 회귀: 주행 중 장애물 제거(`remove_at_s`, `id`), 시나리오 `d_obstacle_vanish`
- v0.39 (concerns)
  - 회귀 지표: 최소여유(라이다 점–footprint 최단 거리), 끼임(정체·boxed_in), 우회비율(주행/직선, 시나리오별 상한). 주행 중 장애물 주입(`at_s`), `--group base|wedge|detour`
  - 시나리오 7개 추가: 회피 중 끼임 위험 3개(w_*), 통과 가능한데 우회 4개(d_*). 판단→조정 표는 TESTING.md §6
- v0.38
  - 회전 공간 없음 + 큰 방향 오차일 때 동작을 footprint 1 s 롤아웃으로 선택: 경로가 차체 옆·뒤로 나가면 yaw 유지 크랩(`crab_no_room`) → 아니면 주행하며 정렬(전진/후진, `align_on_move(_rev)`) → 다 안 되면 정지(`boxed_in`, BT 복구). 랙 모서리와 장애물 사이에서 애커만 정렬을 고집해 끼던 문제
- v0.37
  - 전역 경로가 틈을 두고 먼저 돌아가던 문제: Smac 2D `cost_travel_multiplier` 6.0 → 2.5, 전역 팽창 0.9/2.0 → 0.7/4.0(로컬 costmap은 그대로). 6.0에서는 1.2 m 틈 3 m가 자유 공간 약 12 m로 계산돼 +9 m 우회까지 선호했음
- v0.36
  - v028 계열 경로 번복 수정: 재계획 직전 글로벌 costmap 비우기 삭제(비운 직후 계획 → 마킹 전 지도로 경로 → 좌우 교대). 비우기는 실제 실패 시 복구 단계에서만
  - 경로 히스테리시스를 모든 프로파일에서 켬. circle 모델에서도 기존 경로 앞 3 m가 99/100에 걸리면 '막힘'으로 판정해 교체
- v0.35 (regression)
  - 회귀 러너 `ros2 run midpro_dash run_regression`: 시나리오(직진·코너·측면 정렬·정적 장애물·0.9 m 통로·폭+10 cm 틈·동적 교차·베이 탈출)를 순간이동·장애물 주입·goal 전송으로 자동 반복, 지표·점수·통과 판정 CSV/요약
  - `compare_regression`(결과 비교), `run_variants`(v028 / v028_narrow / 기능 하나씩 끈 변형을 무인 실행 후 비교)
  - 컨트롤러 스위치 `rotation_check`, `align_gate_enabled`. 시뮬 `/sim/set_pose`(순간이동). 절차와 판단 기준은 TESTING.md
- v0.34 (profile)
  - `profile:=v028_narrow`: 차체 폭 + 10 cm 틈 통과용. v028_tight에 footprint 판정(패딩 3 cm), 회전 공간 검사·주행 중 정렬, 진입 전 정렬 게이트, NEAR 탈출 래치, 로컬 costmap 2.5 cm, 전역 반경 0.28만 켬. 궤적 필터·경로 히스테리시스·Hybrid 우회는 끈 상태
  - `profile:=v028_tight`: v0.28 동작 + 간격 축소(전역 반경 0.35, NEAR 0.55, 정지 폴리곤 = footprint, 접근 제동 0.6 s, 팽창 감속 문턱 85)
  - `profile:=v028`(기본): costmap·충돌 모델 변경 이전(v0.28) 동작 재현 — circle 모델, 전역 obstacle layer + 재계획 전 clear, 반경 0.45, Hybrid 국소 우회·궤적 필터·정렬 게이트·경로 히스테리시스 끔, NEAR 탈출 상시, 정지 폴리곤 v0.28 값. 이후 버그 수정(경로 이탈 감시, detour 복귀)과 대시보드는 유지
  - `profile:=current`: v0.33 동작. 코드 스위치 `escape_latch`, `plan_hysteresis` 추가
- v0.33 (trust the plan)
  - 정렬 게이트: '현재 yaw vs 2 m 앞 좁은 지점의 경로 yaw'가 아니라 '로봇 위치에서의 경로 접선 대비 추종 오차'로 판단. 곡선을 잘 따라가는 중에 멈칫하던 원인
  - 궤적 안전 필터: 로봇이 경로 위(횡오차 8 cm·방향 9° 이내)이고 도달할 경로 포즈들이 footprint로 비어 있으면 그대로 통과. 경로에서 벗어났을 때만 1 s 예측
- v0.32 (heading in narrow aisles)
  - NEAR 탈출(yaw 고정 슬라이드)은 계획 시작 시점에 이미 NEAR 안일 때만 래치. 주행 중 좁은 통로에 들어선 경우는 계속 조향해 차체를 경로 방향으로 정렬 — 통로를 yaw 고정 대각선으로 지나다 끼던 원인
  - 제자리 회전 공간이 없을 때는 제자리 회전만 금지하고 '주행하며 정렬'(`align_move_speed` 0.2, 애커만 조향). 기존 '회전 공간 없음 → yaw 고정 슬라이드' 제거
  - 정렬 게이트 1 → 2 m(공간이 있을 때 미리 정렬), 회전 공간이 없으면 감속해 조향이 따라잡게
- v0.31 (kinematic prediction)
  - 궤적 안전 필터: FSM이 낸 최종 명령을 1 s 전방 적분(스웨브 운동학)해 매 0.1 s footprint 검사. 충돌 예측 시 축소 → 회전 제거 → 경로 쪽 홀로노믹 슬라이드 → 회전만 → 정지 순으로 대안 선택. 접촉 상태에서는 멀어지는 움직임 허용(탈출). 사유 `traj_safety …`
  - collision_monitor 정지 폴리곤을 실제 footprint로(접촉 시에만), approach 0.6 s. 패딩된 정지 구역이 멀어지는 움직임까지 막아 끼면 못 빠져나오던 문제
- v0.30 (consistency)
  - 글로벌 costmap = 정적 지도만, 재계획 전 clear 삭제 → 글로벌 경로 결정적(번복·유령 셀 제거). 동적 장애물은 로컬 창(14 m) Hybrid-A*가 footprint로 우회(eband 역할)
  - 경로 히스테리시스: 같은 목표 재계획은 기존 경로가 footprint로 통과 가능하고 새 경로가 15% 이상 짧지 않으면 무시
  - 좁은 구간 진입 전 정렬 게이트(1 m 앞, 7° 이상 틀어지면 정렬 회전), footprint 검증의 0.5 m 예외는 로봇이 이미 끼어 있을 때만
  - 대가: 통로가 완전히 막혀 다른 통로로 돌아가야 하는 경우는 로컬 창에서 못 찾으면 복구 동작으로(동적 레이어 단계에서 해결)
- v0.29 (collision model)
  - `collision_model: footprint`(기본): 직선 가시선·측면/전방 우회점·detour 경로 검증을 포즈 yaw의 footprint 검사로. 사전 필터(외접+2·패딩 밖이면 검사 생략, 내접 안이면 즉시 차단)로 연산량 억제. `circle`로 v0.28 동작 복귀
  - 전역은 Smac 2D 유지, robot_radius 0.45 → 0.30(0.6 m 통로 계획 가능). 경로가 footprint로 안 맞는 구간(긴 차체가 못 도는 코너 등)만 국소 Hybrid-A*로 우회(`hybrid_fallback`, launch 기본 on)
  - 회전 가능 검사(12방향): 회전이 안 들어가는 자리에서는 yaw 유지 홀로노믹으로 진행, 회전 중 스윕이 막히면 중단
  - 측위 오차 대비: `footprint_padding` 0.05(AMCL이면 0.08), 여유가 패딩 2배 미만인 구간 `tight_passage_speed` 0.3
- v0.28
  - 리포트 목표를 숫자로(`--target 95`, 기본 90). 목표 미만 항목에는 규칙에 안 걸려도 항상 미세 조정 제안. '합격 기준(절대값)' 표 추가 — 점수가 아니라 경로 이탈 RMS·가속 RMS·조향 반전/분·도착 오차의 실제 수치로 판정
- v0.27
  - 동적 장애물 복귀: 재계획 전에 글로벌 costmap을 비움(사라진 장애물의 유령 셀이 우회로를 유지시키던 문제). 글로벌 costmap 2 Hz, 관측 유지 1 s, `PathExpiringTimer` 5 s
- v0.26
  - detour 복귀 수정: 직선이 다시 열리면(연속 5틱) 즉시 straight로 복귀. 기존에는 '목표 0.8 m 이내'와 AND라 동적 장애물이 사라져도 우회로를 끝까지 따라감
  - 경로 이탈 감시 `max_path_deviation: 1.5`: 글로벌 경로에서 벗어나면 FollowPath 실패 → BT 재계획. BT에 `PathExpiringTimer 8 s` 추가(유효하지만 낡은 경로 정리)
  - `side_search_max` 5.0 → 2.0: 로컬 우회가 경로 근처에 머물도록
- v0.25
  - 대시보드 탭: 그래프 / 이벤트(실시간) / 장애물 편집. 이벤트 탭은 발생 즉시 시각·현상·권장 파라미터(현재→제안)·이유를 한 줄씩(같은 종류 10 s 간격), CSV 저장, 리포트에 타임라인 포함
  - 장애물 편집 탭: 지도 클릭으로 원/박스, 정지/왕복/순환 이동(속도 지정), 선택·전체 삭제, 보행자 교차 시나리오 버튼. 시뮬에 `/sim/obstacle_cmd`(JSON)로 전달, 센서(스캔) 레벨에서만 존재
  - 열린 공간 goal은 yaw 차이와 무관하게 정렬 진입 후 도착 지점에서 회전(`side_parking_holonomic: false`). 홀로노믹 진입은 목표 1 m 안 NEAR 셀이 있을 때만
- v0.24
  - 베이(ㄴ·ㄷ자) 탈출: NEAR 밴드 안에서는 제자리 회전 금지, 로봇 yaw 유지한 홀로노믹 탈출(`escape_speed` 0.25). detour 분기에도 NEAR 연속 구간 로봇 yaw 적용. 0.4 m 기준점이 탈출 구간 밖을 가리켜 회전이 걸리던 문제 수정
  - 목표 근처 홀로노믹은 필요할 때만(`holonomic_only_when_needed`): 목표 1 m 안 NEAR 셀 또는 측면 주차(30° 이상)일 때만 goal yaw 고정, 열린 공간은 정렬 진입
  - collision_monitor 정지 폴리곤을 footprint+2 cm로(0.63 m 반대각이 NEAR 여유 0.59를 넘어 회전을 막던 문제)
- v0.23
  - 튜닝 리포트 수렴 장치: 목표 등급(`--target`, 기본 B) 달성 시 '튜닝 완료·고정', 최근 3회 변동 < 3점이면 '수렴', 직전 변경 후 하락이면 '되돌리기', 올렸다 내렸다 반복한 파라미터 고정 권장. 주행 이력 `~/midpro_reports/history.json`
  - 재현성 게이트: 같은 진단이 최근 3회 중 2회 이상일 때만 '적용 권장'(한 리포트에 1개), 1회는 '관찰'. 구조적 원인에는 파일·함수·수정 내용의 코드 피드백
- v0.22
  - 점수 집계 모드: 자동(목표별, 기본) / 수동(집계 시작·정지 버튼). 상태줄에 대기·집계 중·고정 표시
- v0.21
  - 튜닝 리포트: 대시보드 '튜닝 리포트 생성' 버튼 → controller_server 파라미터를 읽고, 주행 증거(구간별 이탈, 스파이크 주기, 명령→실제 지연, 제한 사유 점유·전환, collision_monitor 감속 시간, 프로파일 격차, 재계획·회전 횟수)를 규칙 10개로 진단해 파라미터 현재→제안·이유·기대 효과·주의를 HTML로 저장(~/midpro_reports)
- v0.20
  - 대시보드 점수표: 경로 추종·가속·저크·조향·속도·정지 정밀 6항목 0~100점 + 등급(A~D) + 한 줄 평가. goal마다 재집계, 도착 시 고정. `--vmax`로 속도 점수 기준 지정
  - 애커만 조향 곡률 피드포워드 옵션(`curvature_feedforward`, 기본 false). k_prop 1.2 채택(0.4 m 기준점으로 실효 이득이 줄어 곡선에서 이탈하던 문제)
- v0.19
  - `midpro_dash` 패키지: PyQt5/pyqtgraph 실시간 대시보드(`ros2 run midpro_dash speed_dash`)
- v0.18
  - 경로 기반 속도 프로파일: setPlan마다 곡률(√(a_lat/κ))·장애물 근접·목표 제동 상한 + 전후 가감속 패스. FOLLOW_PATH 상한으로 적용, `/midpro/speed_profile`(s, v 쌍) 발행. 거버너는 반응형(NEAR·목표 제동)만 담당
- v0.17
  - 직선 폴백 플래너(`MidproStraightPlanner`) + BT 폴백: 글로벌 경로가 없어도 ROS1처럼 직선 목표로 FSM이 출발. 따라가는 글로벌 경로 앞 2 m에 99/100이 있으면 FollowPath 실패 → 복구(안전 검사). Smac 2D tolerance 0.5
- v0.16
  - 글로벌 costmap robot_radius 0.59 → 0.45(1.18 m 미만 틈이 전부 막히던 문제), 다운샘플링 해제, cost_travel_multiplier 6.0. 기본 지도를 점 잡음 제거본(_clean)으로
- v0.15
  - 진단 로그: setPlan 때 goal yaw, FINAL_ROTATE 도착 때 goal/robot yaw
  - RViz 부하 완화: 글로벌 costmap 0.5 Hz·변경분 발행, 로컬 2 Hz, local_plan 화살표 표시 기본 해제, Frame Rate 20
- v0.14
  - 조향 기준점을 5 cm 앞(ROS1)에서 `heading_lookahead 0.4 m` 앞 경로점으로: 격자 경로 접선 흔들림에 의한 좌우 진동 제거. `k_prop` 0.8로 복귀. 글로벌 플래너 다운샘플링·계획 시간 8 s
- v0.13
  - 애커만 법칙 `linear.x = v·cos(ratio)`에 하한 `min_turn_speed_ratio 0.67`(조향 중 0.4 이상), `max_linear_y 0.4`. 감속 하한 0.4로 통일
- v0.12
  - 거버너 진단 토픽 `/midpro/speed_limit`(상태 | 활성 제한 | cap). 곡률 판정을 1 m 구간 방향 변화로 변경(격자 경로 점별 흔들림 제거), 팽창 감속 문턱 50→80(벽 0.65 m 안에서만), 접근 제동 1.0 s
- v0.11
  - 감속 중첩 수정: collision_monitor `CircleSlow`(×0.17) 제거, 거버너 하한 `governor_floor: 0.1`(midpro deceleration_min_vel), 팽창 감속은 비용 50 이상에서만. 조향 `k_prop 1.2`, 각속도 0.8
  - BT: 1 Hz 무조건 재계획 → `IsPathValid`/goal 변경 시에만 재계획(경로 흔들림 제거). Hybrid-A* 브리지 0.5 Hz, 경로 유효 3 s
- v0.10
  - midpro obstacle_detector 체인(정지 구역 out_rec, 감속 반경 1.0 m → 0.1 m/s)을 `nav2_collision_monitor`로 이식. 체인: 컨트롤러 → collision_monitor → velocity_smoother. footprint 기반 접근 제동(approach) 추가
- v0.9
  - 기본값을 v0.4 거동으로 복귀: `waypoint_mode: final` + `detour_source: global_plan`(`detour:=false`). 코너 정지 방식은 직선 분절만 만들어 곡선 추종을 없앴으므로 옵션으로만 유지
  - 코너 각도는 FSM이 아니라 Smac 2D 스무더(`w_smooth`)로 조절
- v0.8
  - 기본 구성 복귀: `tracking_mode: midpro`(ROS1 점대점 FSM 법칙) + `detour_source: planner`(Smac Hybrid-A*, `detour:=true` 기본) + `waypoint_mode: corners`
  - 속도 0.6 m/s, 가속 0.4 m/s², 접근 구간 0.8 m. 1.0 m/s는 추종·도착 정밀도가 불안정해 폐기
  - neo_local_planner2 연동(`controller:=neo`, 오버레이 yaml) 제거. lookahead 모드는 코드에만 남김(`tracking_mode: lookahead`, 실험용)
- v0.7
  - `holonomic_policy: goal_zone`(기본): 주행 중은 차량형(vy=0, pure pursuit 조향), 목표 1.5 m 안에서만 홀로노믹(목표 yaw로 미리 정렬하며 진입). ㄴ·ㄷ자 직각/직진 주차와 측면 정렬을 한 규칙으로 처리
- v0.6
  - `tracking_mode: lookahead`(기본): FOLLOW_PATH/FINAL_APPROACH를 lookahead 경로 추종으로 교체. 로컬 경로 = 글로벌 경로(항상), 속도 벡터는 lookahead 점, 차체 yaw는 접선 P제어, 제자리 회전은 70° 이상에서만. ROS1 점대점 법칙은 `tracking_mode: midpro`로 유지
  - `waypoint_mode: final`(기본): 코너 정지 없이 통과. 코너 정지가 필요하면 `corners`
- v0.5
  - 웨이포인트 시퀀서: 글로벌 경로를 Douglas–Peucker(0.10 m)로 코너 분해, ROS1 move_server처럼 순서대로 FSM에 제공. 같은 목표의 재계획은 진행 상태 유지. 직선/곡선 경로가 번갈아 나오던 원인 제거
  - 중간 웨이포인트는 코너까지 구간 방향 유지(`waypoint_orient_dist: 0.0`) → 코너에서 제자리 회전. 최종 목표만 ROS1의 0.8 m 규칙
  - 글로벌 costmap `robot_radius: 0.59`(ROS1 NEAR 밴드): Smac 2D 경로가 99 밴드 근처로 가던 문제 해결
  - 속도 거버너(neo_local_planner2 개념): 1.0 m/s, 가속 0.8 / 감속 0.6, 전방 비용·곡률·정지거리 상한. velocity_smoother·시뮬 가속 한계 상향
  - `controller:=neo`로 neo_local_planner2를 FollowPath에 끼워 A/B 비교 가능(neo_ws source 필요)
  - 기본 지도 ecoproBM_matterport, 초기 위치 (34.3, 0.95). `docs_sequence.html` 시퀀스 다이어그램 추가
- v0.4
  - 스웨브 플랫폼 모델 v2: 하부 섀시 블록, 스윙 암, 조향 너클, 휠 허브 커버, 전방 센서 포드, 비상정지 버튼으로 구조 완결(외형 복제는 아님)
  - NEAR(95) 밴드를 거리 기준으로 복원: `near_obstacle_distance`(0.59 m)에서의 inflation 비용 이상을 95로 양자화. 회전·주행 시 footprint가 장애물에 걸리던 문제의 원인(지수 감쇠 팽창에서 95가 0.26 m에 해당했음) 수정
  - inflation_radius 0.9 / cost_scaling 2.0, Smac 2D cost_travel_multiplier 4.0
- v0.3
  - 기본을 벽시계로 전환(`sim_time:=false`): `/clock` 결합으로 인한 타이머 떨림 제거. 시뮬 물리는 전용 스레드에서 정밀 페이싱
  - URDF: `base_footprint`를 `base_link`의 자식으로(TF 이중 부모 오류 해결), 스웨브 플랫폼 xacro 추가(`model:=swerve`)
- v0.2
  - detour 기본 동작을 "Nav2 글로벌 경로 추종"으로 변경(`detour_source: global_plan`). Hybrid-A* 서버는 `detour:=true`일 때만 실행
  - 시뮬레이터: 벽시계 기준 시간 진행(부하와 무관하게 실시간 유지), numpy 벡터화 레이캐스팅(스캔당 ~9 ms), `use_sim_time: False`(`/clock` 생산자), 초기 위치 인자, `unknown_as_occupied`, ground-truth 측위 모드
  - detour lifecycle manager `bond_timeout: 0`, Hybrid-A* 룩업 테이블·각도 분할 축소(설정 8 s → 수백 ms 예상), detour 목표를 글로벌 경로 상 lookahead 지점으로 선택
  - launch: `detour`, `localization`, `init_*` 인자. BT를 모드별 2개로 분리
  - RViz: Orbit / FollowRobot 뷰 추가
  - ecoproBM_matterport 지도 동봉(원본, 점 잡음 제거본)
- v0.1 초기 작성 (미빌드)

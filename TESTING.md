# 테스트 절차와 판단 기준 (v0.39)

목적: `v028`(circle 모델)과 `v028_narrow`(footprint 최소 구성)를 같은 시나리오로 비교해, footprint 판정을 켰을 때
제어가 나빠지는지, 나빠진다면 어떤 기능 때문인지 숫자로 정한다.

## 0. 준비
```bash
cd ~/midpro_nav2_ws && rm -rf src build install log && unzip -o ~/Downloads/midpro_nav2_ws.zip
colcon build --symlink-install && source install/setup.bash
```

## 1. 빠른 확인 (프로파일 하나, 수동)
```bash
# 터미널 1
ros2 launch midpro_bringup sim_nav.launch.py localization:=gt model:=swerve profile:=v028      # 기본은 v028_narrow
# 터미널 2 (시나리오 8개 x 1회, 약 10분)
ros2 run midpro_dash run_regression --label v028 --repeats 1
```
`profile:=v028_narrow`로 바꿔 같은 명령을 `--label v028_narrow`로 한 번 더. 비교:
```bash
ros2 run midpro_dash compare_regression ~/midpro_reports/regression/v028_2* ~/midpro_reports/regression/v028_narrow_2*
```

## 2. 자동 비교 (권장, 무인 실행)
```bash
ros2 run midpro_dash run_variants --repeats 3        # 8개 변형 x 8 시나리오 x 3회 ≈ 3~4시간
ros2 run midpro_dash run_variants --repeats 1 --variants v028,v028_narrow,narrow_no_align   # 빠른 판
```
변형마다 launch를 RViz·대시보드 없이 띄우고 회귀를 돌린 뒤 끄고 다음으로 넘어간다. 결과:
`~/midpro_reports/regression/variants_<시각>/` 아래 변형별 `summary.md`, 마지막에 `compare.md`(v028 기준 비교).

| 변형 | 내용 |
|---|---|
| v028 | 기준선 (circle) |
| v028_narrow | footprint 최소 구성 전부 켬 |
| narrow_no_align | 진입 전 정렬 게이트만 끔 |
| narrow_no_rotchk | 회전 공간 검사만 끔 |
| narrow_no_latch | NEAR 탈출 래치만 끔 (v028처럼 NEAR 안이면 항상 탈출) |
| narrow_no_tight | 좁은 구간 감속만 끔 |
| narrow_res05 | 로컬 costmap 5 cm (2.5 cm 효과 확인) |
| current | 최신(v0.33) 전체 |

## 3. 시나리오 (ecoprobm_regression.yaml)
straight_20m · corner · side_align · static_obstacle · aisle_0p9(0.9 m 통로 6 m) · gap_w10(폭+10 cm 틈) · dynamic_cross · bay_exit.
좌표는 지도 여유 거리로 검증했고, 장애물은 시뮬 라이다에만 주입된다. 다른 지도는 같은 형식으로 파일을 만들어 `--scenarios`로 지정.

## 4. 한 회 판정
- **통과** = 도착 + 종합 ≥ 70 + 모든 항목 ≥ 40 + 끼임 없음(10 s 동안 5 cm 미만 이동이 없을 것) + (틈 시나리오는) 틈으로 통과
- **G1** = 모든 시나리오가 3회 중 2회 이상 통과하고 최저 60 이상

## 5. 결과에 따른 다음 방향
| 결과 | 결정 |
|---|---|
| v028_narrow가 v028과 비슷하거나 낫고, gap_w10 통과 | **v028_narrow를 1단계 기준선으로 확정** → MobED 제원 반영 → AMCL·실기 단계 |
| v028_narrow가 특정 시나리오에서 악화, 어떤 narrow_no_* 변형에서 회복 | 그 기능이 원인. 그 기능만 고치거나 끈 상태로 기준선 확정 |
| 모든 narrow 변형이 v028보다 악화 | footprint 판정 자체(직선·우회 판정)의 문제 → 판정 함수 수정, 그 전까지 v028_tight로 운용하고 폭+10 cm 요구는 보류 |
| v028도 G1 미달 | 기본 제어(속도·조향) 튜닝이 먼저. 리포트·이벤트의 제안으로 v028부터 B 확보 |
| gap_w10만 실패(narrow) | 정렬·속도 문제인지(정렬 게이트 이벤트, 끼임 여부) 확인 → 패딩·정렬 허용각 조정 |
| dynamic_cross·static_obstacle만 악화 | 동적 장애물 처리(재계획·우회)의 문제 → 2단계 동적 레이어로 넘김, 1단계 판단에서는 제외 가능 |

## 6. 두 가지 우려를 재는 시나리오 (v0.39)

```bash
# 끼임 위험 3개 + 불필요 우회 4개만, 프로파일 하나
ros2 run midpro_dash run_regression --label narrow_concerns --group wedge,detour --repeats 3
# 변형 비교까지
ros2 run midpro_dash run_variants --group wedge,detour --repeats 2 --variants v028,v028_narrow,narrow_no_align,narrow_no_rotchk
```

| 시나리오 | 재현하는 상황 |
|---|---|
| w_squeeze_inject | 1.6 m 통로 진입 직전 장애물 등장 → 벽 쪽 0.75 m로 비켜 지나가기 |
| w_corner_avoid | 튀어나온 랙 모서리 옆에 장애물 등장 → 모서리 쪽 회피 후 복귀 |
| w_insert_close | 순항 중 1.5 m 앞 장애물 등장 → 급회피 |
| w_pallet_drop | 순항 중 4.6 m 앞 팔레트 등장 → 멈추지 않고 멀리서 우회 (yield 횟수 0이 이상적) |
| d_gap_choice_w20 | 0.70 m 틈 vs 약 6 % 먼 넓은 개구부 (정책 선택) |
| d_gap_choice_w40 | 0.90 m 틈 vs 넓은 개구부 (틈으로 가야 정상) |
| d_gap_full_w20 | 0.70 m 틈만 있음 (못 지나가면 먼 우회/타임아웃) |
| d_gap_offset_w40 | 0.90 m 틈이 직선에서 0.9 m 옆 (측면 이동으로 찾아 들어가기) |
| d_obstacle_vanish | 우회 중 장애물이 사라짐 → 최단 경로 복귀 (우회비율 ≤ 1.04). **v0.41부터 경로 스위칭 기본 꺼짐 → 이 시나리오는 스위칭을 켠 상태에서만 통과 기대** |

새 지표: **최소여유 cm**(라이다 점과 footprint 최단 거리, 음수 = 접촉), **끼임**(10 s 정체 또는 boxed_in), **우회비율**(주행 거리 / 직선 거리, 시나리오별 상한).

### 판단 → 조정
| 결과 | 원인 후보 | 조정 |
|---|---|---|
| w_* 에서 끼임 + 직전 사유 `boxed_in` | 회피 경로가 모서리로 몰아넣음 | `cost_travel_multiplier` 2.5 → 3.0~3.5(전역 경로가 모서리에서 떨어지게), `footprint_padding` 0.03 → 0.04 |
| w_* 에서 끼임 + 사유 `crab_no_room` 반복 | 크랩 탈출이 다시 다른 벽으로 | `escape_speed` 0.25 → 0.15, 정렬 게이트 확인(`align_gate_dist`) |
| w_* 최소여유 < 2 cm인데 통과 | 아슬아슬 | 실기 측위 오차 고려 시 `footprint_padding` 상향 검토 |
| d_gap_choice_w40 / d_gap_full_w20 우회비율 초과 | 전역 경로가 틈을 피함 | 전역 `robot_radius` 확인(narrow 0.28), 전역 팽창 0.7/4.0 → 0.6/5.0, `cost_travel_multiplier` 2.5 → 2.0 |
| 전역 경로는 틈으로 가는데 로봇이 안 들어감 | 컨트롤러 판정 | 이벤트 탭의 `traj_safety`/`boxed_in`/정렬 사유 확인 → footprint 패딩·정렬 허용각 |
| d_gap_offset_w40 실패 | 측면 우회점 탐색 범위 | `side_search_max` 2.0 → 2.5 |
| d_gap_choice_w20 에서 넓은 쪽 선택 | 정책 문제(6 %면 넓은 쪽도 합리적) | 원하면 `cost_travel_multiplier` 2.0. 시나리오 상한(1.03)은 필요에 맞게 조정 |

## 7. 장애물 레이아웃으로 같은 테스트 재현 (v0.62)

1. 장애물 편집 탭에서 장애물을 놓고, 목표를 한 번 줌(목표가 함께 저장됨)
2. '레이아웃 저장'에 이름을 넣고 저장 → `~/midpro_reports/obstacle_layouts/<시각>_<이름>.yaml`
3. 다시 할 때: 목록에서 선택(또는 더블클릭) → 불러오기. '로봇을 저장된 위치로 이동', '저장된 목표 전송'을 켜 두면 같은 출발·같은 목표로 재주행
4. 자동 반복: `ros2 run midpro_dash run_regression --scenarios ~/midpro_reports/obstacle_layouts/<파일>.yaml --repeats 3`

## 8. 경로 정리기 전후 비교 (v0.81)

주차·좁은 구간 시나리오를 정리기 켬/끔으로 각각 돌려 같은 결과(또는 개선)인지 확인한다.

```bash
ros2 run midpro_dash run_regression --label cond_on  --only p_wall_side_park,p_u_pocket_park,p_wall_facing_slot,gap_w10 --repeats 3
ros2 param set /controller_server FollowPath.path_conditioner_enabled false
ros2 run midpro_dash run_regression --label cond_off --only p_wall_side_park,p_u_pocket_park,p_wall_facing_slot,gap_w10 --repeats 3
ros2 run midpro_dash compare_regression ~/midpro_reports/regression/cond_off_* ~/midpro_reports/regression/cond_on_*
```

기대: `p_wall_side_park`·`p_u_pocket_park`·`gap_w10`은 둘 다 통과(정리기가 기존 능력을 깨지 않음), `p_wall_facing_slot`은 정리기 켬에서만 통과.
시나리오 장애물 좌표는 ecoproBM 지도 기준 추정이라 지도와 겹치면 장애물 편집 탭에서 옮긴 뒤 레이아웃으로 저장해 쓴다.

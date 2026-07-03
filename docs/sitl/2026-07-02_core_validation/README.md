# 핵심 검증부

Robotov2.0 RoboParty right arm + AmazingHand를 실제 DM4340P 모터에 연결하기 전에 먼저 고정해야 하는 검증 문서 모음입니다.

목표는 LeRobot leader arm 입력을 실제 하드웨어가 아니라 Isaac Sim SITL follower에 먼저 연결하고, local wrapper 구조, leader output shape, 6D action mapping, URDF joint/limit을 증거 기반으로 확정하는 것입니다.

이 문서는 전자/기계/컴퓨터 전공지식은 있지만 이 프로젝트의 LeRobot, Isaac Sim SITL, Robotov2.0, AmazingHand 연결 구조를 아직 모르는 학생을 기준으로 작성합니다. 각 task는 단순 업무 목록이 아니라 “왜 이 검증을 먼저 하는지”, “무엇을 기록해야 다음 단계가 안전해지는지”, “어떤 상태면 중단해야 하는지”를 함께 배웁니다.

## 학습 흐름

```text
C01 구조 이해
  -> C02 leader arm 원시 출력 확인
  -> C03 6D action mapping 정책 결정
  -> C04 URDF joint/limit 근거 확인
  -> C05 no-op SITL 안전성 확인
  -> C06 joint sweep SITL 반복 동작 확인
  -> C07 SITL-to-real mapping table 작성
  -> C08 hardware-free clamp 정책 검증
  -> C09 dataset schema 확인
  -> C10 debug dataset record/replay 계획
  -> C11 AmazingHand 실제 연결/read-only/tiny-motion 검증
  -> C12 LeRobot sim-real motor angle parity 확인
  -> C13 D435i 기반 쓰레기 물체 파지 pipeline 계획
  -> 이후 실제 DM4340P tiny motion 판단
```

핵심은 실제 모터에 명령을 보내기 전에 입력, 변환, 시뮬레이션, 관측 contract가 맞는지 증거로 확인하는 것입니다. 여기서 contract는 action 길이, joint 순서, 단위, 부호, limit, observation key, topic 이름처럼 팀원이 모두 같은 뜻으로 사용해야 하는 약속입니다.

## 전공별 학습 관점

| 파트 | 이 문서에서 집중할 내용 | 특히 봐야 할 task |
|---|---|---|
| 전자 | 실제 DM4340P, CAN bus, hand servo에 명령이 나가지 않는 조건을 확인하고, 이후 hardware parity로 넘어갈 때 필요한 safety gate를 이해한다. | C02, C05, C06, C07, C08 |
| 기계 | URDF joint, axis, parent/child link, joint limit이 실제 팔의 운동 방향과 어떻게 연결되는지 이해한다. | C03, C04, C06, C07 |
| 컴퓨터 | LeRobot wrapper lifecycle, ROS2 topic, 6D action/state schema, evidence JSON, dataset schema를 기준으로 재현 가능한 검증 절차를 만든다. | C01, C03, C05, C06, C09, C10 |

세 파트는 task를 나눠서 수행하더라도 같은 산출물을 공유해야 합니다. 예를 들어 C04에서 확인한 joint limit은 C03 mapping clamp와 C06 sweep target의 근거가 됩니다.

## Task 목록

| Task | 목적 | 난이도 | 병목도 | 문서 |
|---|---|---:|---:|---|
| C01 | LeRobot checkout과 SITL wrapper 구조 확인 | 중 | 중 | [C01](C01_lerobot_wrapper_layout.ko.md) |
| C02 | leader arm output shape 확인 | 중 | 높음 | [C02](C02_leader_arm_output_shape.ko.md) |
| C03 | leader output을 6D SITL action으로 변환 | 중상 | 높음 | [C03](C03_leader_to_sitl_6d_action.ko.md) |
| C04 | Robotov2.0 + AmazingHand URDF joint/limit audit | 중 | 중 | [C04](C04_robotov2_urdf_joint_limit_audit.ko.md) |
| C05 | SITL no-op action 검증 | 중 | 높음 | [C05](C05_sitl_no_op_test.ko.md) |
| C06 | SITL joint sweep + screenshot 검증 | 중상 | 높음 | [C06](C06_sitl_joint_sweep_test.ko.md) |
| C07 | SITL-to-real mapping table 작성 | 중 | 매우 높음 | [C07](C07_sitl_to_real_mapping_table.ko.md) |
| C08 | DM motor command clamp 구현/검증 계획 | 상 | 매우 높음 | [C08](C08_dm_motor_command_clamp.ko.md) |
| C09 | dataset schema check | 중 | 중 | [C09](C09_dataset_schema_check.ko.md) |
| C10 | debug dataset 10 episode record/replay 계획 | 중상 | 중 | [C10](C10_debug_dataset_record_replay.ko.md) |
| C11 | AmazingHand 실제 하드웨어 연결/제어 테스트 | 중상 | 높음 | [C11](C11_amazinghand_hardware_control_test_plan.ko.md) |
| C12 | LeRobot sim-real motor angle parity | 상 | 매우 높음 | [C12](C12_lerobot_sim_real_motor_angle_parity_plan.ko.md) |
| C13 | D435i 기반 쓰레기 물체 파지 pipeline | 상 | 높음 | [C13](C13_d435i_trash_grasp_pipeline_plan.ko.md) |

## AI 보조 문서

AI는 작업의 주체가 아니라 보조 도구입니다. 사람이 task를 이해하고 판단한 뒤, 로그 분석, 표 정리, 코드/명령 초안 작성이 필요할 때 아래 보조 문서를 참고합니다.

| Task | AI 보조 문서 |
|---|---|
| C01 | [AI C01](ai_agent/C01_lerobot_wrapper_layout.ai.ko.md) |
| C02 | [AI C02](ai_agent/C02_leader_arm_output_shape.ai.ko.md) |
| C03 | [AI C03](ai_agent/C03_leader_to_sitl_6d_action.ai.ko.md) |
| C04 | [AI C04](ai_agent/C04_robotov2_urdf_joint_limit_audit.ai.ko.md) |
| C05 | [AI C05](ai_agent/C05_sitl_no_op_test.ai.ko.md) |
| C06 | [AI C06](ai_agent/C06_sitl_joint_sweep_test.ai.ko.md) |
| C07 | [AI C07](ai_agent/C07_sitl_to_real_mapping_table.ai.ko.md) |
| C08 | [AI C08](ai_agent/C08_dm_motor_command_clamp.ai.ko.md) |
| C09 | [AI C09](ai_agent/C09_dataset_schema_check.ai.ko.md) |
| C10 | [AI C10](ai_agent/C10_debug_dataset_record_replay.ai.ko.md) |
| C11 | [AI C11](ai_agent/C11_amazinghand_hardware_control_test_plan.ai.ko.md) |
| C12 | [AI C12](ai_agent/C12_lerobot_sim_real_motor_angle_parity_plan.ai.ko.md) |
| C13 | [AI C13](ai_agent/C13_d435i_trash_grasp_pipeline_plan.ai.ko.md) |

## 학습 방식

- 학생은 먼저 각 task의 사람용 문서를 읽고, 학습 목표와 승인 기준을 자기 말로 설명한다.
- 실습자는 명령을 실행하기 전에 "이 명령이 실제 hardware를 움직이는가?"를 먼저 확인한다.
- AI 도움을 받을 때는 `ai_agent/` 문서를 참고하고, 산출물 report와 pass/fail 판단은 사람이 다시 검토한다.
- Screenshot은 사람이 보는 증거이고, JSON은 pass/fail을 판단하는 증거다. 둘을 구분해서 기록한다.

## 공통 원칙

- C01-C10은 실제 DM4340P 모터를 움직이지 않는다.
- C11-C13은 사람이 문제를 풀고 판단하는 후속 실험이다. AI는 로그 해석, 표 정리, script 초안 작성 같은 보조 역할만 한다.
- C11은 새 adapter 구현이 아니라 upstream AmazingHandControl repo의 테스트/실행 프로그램을 이 컴퓨터에서 잘 동작시키는 task다.
- 검증/정책 replay는 `IsaacSimRpoArmRobot.send_action()` 경로를 사용한다.
- live leader teleop/recording은 `/leader/joint_commands` 입력을 받고 `teleop_step()`을 통해 `/follower/joint_commands`로 넘기는 경로를 사용한다.
- SITL 1차 contract는 6D action/state를 유지한다.
- AmazingHand는 처음부터 raw 8-servo control로 열지 않고 `amazinghand_grasp.pos` scalar로 시작한다.
- C05-C10은 2026-07-02 핵심 검증부 기준이다. 2026-06-27 문서의 예전 task 번호 의미와 섞지 않는다.
- 각 task 산출물은 `docs/sitl/2026-07-02_core_validation/artifacts/` 아래 markdown으로 남긴다.
  Runtime JSON, screenshot, contact sheet는 `isaacsim_test/artifacts/` 아래에 남긴다.

## 학생용 진행 규칙

1. 먼저 task 문서의 학습 목표와 선수 지식을 읽고 모르는 용어를 표시한다.
2. 진행 절차는 위에서 아래로만 수행한다. 중간 절차를 건너뛰면 결과가 맞아도 승인하지 않는다.
3. 명령을 실행했다면 명령어, 실행 위치, 날짜, 출력 요약, 생성 파일 경로를 산출물에 남긴다.
4. 판단이 필요한 경우에는 느낌이 아니라 evidence path, 숫자, 표, screenshot 이름으로 기록한다.
5. 실패하면 다음 task로 넘어가지 않는다. 실패 원인 후보와 재실행 조건을 산출물에 적는다.
6. 실제 hardware command가 의심되는 명령은 실행하지 않고 문서에 blocker로 남긴다.

## 공통 산출물 양식

각 task 완료 후 아래 형식으로 `docs/sitl/2026-07-02_core_validation/artifacts/`에 markdown 파일을 남깁니다.

````markdown
# CXX 산출물 - <짧은 제목>

## 실행 정보

- Date:
- Name:
- Repo path: `/home/sim/Documents/superarm_ws`
- Related task:

## 실행한 명령

```bash
<실제로 실행한 명령>
```

## 관찰 결과

| 항목 | 결과 | 근거 |
|---|---|---|
| <확인 항목> | pass/fail/blocked | <파일, 숫자, 로그, screenshot> |

## 판단

- Pass/Fail/Blocked:
- 이유:
- 다음 task로 넘어가도 되는지:

## 남은 위험

- <예: leader raw output logger가 아직 없음>
````

## 완료 기준

```text
[ ] C01 wrapper 구조 산출물이 있다.
[ ] C02 leader raw output 산출물이 있다.
[ ] C03 leader-to-SITL mapping 산출물이 있다.
[ ] C04 URDF joint/limit audit 산출물이 있다.
[ ] C05 no-op evidence JSON이 있다.
[ ] C06 joint sweep evidence JSON, screenshots, contact sheet가 있다.
[ ] C07 hardware parity mapping table이 있다.
[ ] C08 clamp 정책과 hardware-free test 결과가 있다.
[ ] C09 dataset schema report가 있다.
[ ] C10 debug dataset record/replay QA report 또는 blocker가 있다.
[ ] C11 AmazingHandControl upstream repo test/run report 또는 blocker가 있다.
[ ] C12 sim-real parity report 또는 blocker가 있다.
[ ] C13 D435i synthetic/SITL grasp report 또는 blocker가 있다.
[ ] C01-C13 결과를 바탕으로 실제 DM4340P/AmazingHand tiny motion과 D435i grasp trial을 진행할지 판단할 수 있다.
```
